#!/usr/bin/env python3
# =============================================================================
#  bdie_sim.py — Banco de simulación SITL de BDI-E
#
#  Aquí no hay lógica de decisión. Este archivo aporta lo que el robot tiene
#  alrededor y el firmware no lleva adentro:
#
#     1. la física           — importada de attabot_sim.py (copia del
#                              repositorio Atta-Bot), con el ruido calibrado en
#                              laboratorio (σ ArUco 30 mm, σ ángulo 2°, deriva
#                              de motor 2 %) y el equivalente de ReactiveNavStep()
#     2. el canal de radio   — UDP sin acuse, con pérdida de paquetes inyectada
#     3. el modelo de sensor — el parche de color bajo el robot
#
#  Todo lo demás (creencias, deseos, planes, feromona, reparto de zonas, cierre
#  de la congregación y protocolo) lo ejecuta libbdie.so, que es el mismo C++
#  que se compila para el ESP32. Aquí solo se dice si el objetivo está al
#  alcance del sensor.
#
#  La congregación es el MEET del laboratorio, y también corre en C++: las
#  funciones del sketch (HandleMeet, MeetSlotIndex, SafeRingSlotAngle,
#  UpdateCongregationGoal, ReactiveNavStep…) se compilan tal cual en
#  lab_cong.cpp (ver extraer_lab.py). Mientras un robot congrega, su cuerpo
#  ejecuta la cola TURN/MOVE/WAIT/REQUEST_POSITION que arma el código del
#  laboratorio.
#
#  El banco es una herramienta de ingeniería: sus resultados dependen de un
#  modelo del entorno y no sustituyen la validación sobre los robots físicos.
#
#  Uso:
#     python3 bdie_sim.py --runs 200
#     python3 bdie_sim.py --runs 100 --loss 0.15 --detect 60
#     python3 bdie_sim.py --campana          # los bloques de la campaña
#     python3 bdie_sim.py --barrido          # sensibilidad a lo no medido
#     python3 bdie_sim.py --fallo            # 20 caídas en búsqueda, robot visible
#     python3 bdie_sim.py --reconexion       # silencio de 20 s y reinicio con JOIN
#     BDIE_LIB=<otra .so> python3 bdie_sim.py --fallo   # contra otra biblioteca
# =============================================================================

import ctypes, math, random, argparse, os, sys, json, base64
from statistics import median

import attabot_sim as phys
from attabot_sim import (SimRobot, SimWorld, Obstacle, ReactiveNav,
                         normalize_angle, CENTER_TO_WHEEL, ROBOT_RADIUS,
                         ARUCO_POS_SIGMA, ARUCO_ANGLE_SIGMA, MOTOR_DRIFT_SIGMA)

# ── Umbral de llegada: 80 mm, no los 50 de fábrica ──────────────────────────
# Es RecommendedArrivalMm(190 mm) de grid.h, y se fija en el robot real con el
# comando NAV_CONFIG|ARRIVAL_THRESHOLD que ya existe. Con σ_ArUco = 30 mm,
# exigirle 50 mm hace que el robot rebote alrededor del punto.
# (se fija más abajo, junto a la cuadrícula: depende de CELL_MM)

# ── Perfil de velocidad, como en AttaBot.ino ────────────────────────────────
# Con samplingTime = 10 ms, baseSpeed ≈ 24,4 mm/s y maxSpeed = 5 × baseSpeed.
MAX_SPEED_MMS = 122.0
MIN_SPEED_MMS = 12.0
DECEL_MM      = 16.0          # speedReductionThreshold
TURN_DPS      = math.degrees(MAX_SPEED_MMS / CENTER_TO_WHEEL)   # ≈ 168 °/s
INSTR_OVERHEAD_S = 0.05       # asentamiento del PID entre instrucciones

# ── El compañero como obstáculo, de AttaBot.ino:2204 (IsRobotObstacle) ──────
#
# ESTA ES LA FÍSICA QUE DECIDE LA CONGREGACIÓN, y conviene leerla entera:
#
#   Cuando un robot detecta algo con el IR, difunde CHECK_OBSTACLE y espera
#   hasta 600 ms. Cada par evalúa IsRobotObstacle(): si está a menos de 260 mm
#   y dentro del sector del sensor que disparó, contesta «soy yo». Entonces el
#   que preguntó NO EJECUTA LA MANIOBRA DE EVASIÓN: simplemente ESPERA
#   (AttaBot.ino:899-903).
#
# La consecuencia es la que importa: dos robots que tienen que quedarse a menos
# de 260 mm uno del otro no se chocan — se quedan esperándose. Y un robot que
# espera nunca llega a su lugar. Sin modelar esto, el banco no puede decir nada
# sobre un radio de estacionamiento.
ROBOT_OBSTACLE_MM = 260.0     # robotDistanceMargin
IDENTIFY_WAIT_S   = 0.60      # ventana de IDENTIFY_OBSTACLE
PEER_SECTOR_DEG   = 45.0      # cono del sensor IR (attabot_sim: ±45°)

# ── Multiplexación temporal de los emisores IR, como en AttaBot.ino ─────────
#
# ReadSensors() parte el tiempo en 18 ranuras de 1600 µs, y cada robot usa una
# para sus IR laterales. Fuera de su ranura los emisores están apagados, así
# que dos robots solo se leen mutuamente si caen en la misma ranura. Si la
# ranura se sortea con random() antes de sembrar el generador, todos los robots
# obtienen la misma y el mecanismo anti-diafonía queda anulado; por eso el
# banco distingue los dos casos (ir="por_id" o sorteada).
IR_RANURAS = 18

# ── Arena y misión, del contrato con el cliente ─────────────────────────────
ARENA_W, ARENA_H = 2500.0, 2000.0        # área útil centrada en el origen
GRID_COLS, GRID_ROWS, CELL_MM = 14, 11, 190.0
# La cuadrícula DESBORDA la arena (14×190 = 2660 y 11×190 = 2090 sobre 2500×2000):
# el origen se centra sobre la cuadrícula, no sobre la arena, para que el desborde
# se reparta por igual y todos los CENTROS de celda queden dentro. Ver grid.h.
ORIGIN_X, ORIGIN_Y = -GRID_COLS * CELL_MM / 2, -GRID_ROWS * CELL_MM / 2

# RecommendedArrivalMm(grid.h) = celda × 0,32. Con 190 mm son 61 mm ≈ 2,0 σ de
# ArUco. Se deriva de la celda a propósito: dejarlo escrito a mano fue lo que
# hizo fallar tres verificaciones al cambiar la resolución.
phys.ARRIVAL_THRESHOLD = CELL_MM * 0.32
MISSION_TIMEOUT_S = 20 * 60
# El MEET del laboratorio trabaja en el marco de la cámara: origen en la
# esquina, arena [0, W] × [0, H] (SafeRingSlotAngle aleja las ranuras de esas
# paredes). El banco está centrado en el origen: se traslada en la frontera.
LAB_DX, LAB_DY = ARENA_W / 2, ARENA_H / 2
NEIGHBOR_PERIOD_S = 1.0       # NEIGHBOR_POSITIONS de la base, 1 Hz
MAX_CELLS_PY = 160          # igual al MAX_CELLS de grid.h

DT = 0.05                     # paso de integración, 20 Hz
POSES_PERIOD_S = 0.10         # broadcast de la base, 10 Hz
SENSOR_PERIOD_S = 0.10        # muestreo del sensor de color, 10 Hz

PH_IDLE, PH_DESPLEGAR, PH_BUSCAR, PH_CONGREGAR, PH_VOLVER, PH_FIN, PH_SEGURO = range(7)
PHASE_NAME = ["IDLE", "DESPLEGAR", "BUSCAR", "CONGREGAR", "VOLVER", "FIN", "SEGURO"]
# Orden del enum Desire de bdi.h
DESIRE_NAME = ["TRANQUILO", "EXPLORAR", "DESPLEGAR", "CONGREGAR",
               "VOLVER", "SEGURO"]
A_GOTO_CELL, A_GOTO_MM, A_HOLD, A_ABORT, A_MEET = 1, 2, 3, 4, 5

# Cadencias de grabación. Las poses se muestrean fino porque es lo que se mira;
# los mapas, a 1 Hz, porque son 80 B por campo por robot y a 20 Hz no cabrían.
GRABA_POSE_HZ = 5
GRABA_MAPA_HZ = 1


# =============================================================================
#  Puente a libbdie.so — el C++ real
# =============================================================================
class COut(ctypes.Structure):
    _fields_ = [("kind", ctypes.c_int), ("cell", ctypes.c_int),
                ("x", ctypes.c_float), ("y", ctypes.c_float),
                ("phase", ctypes.c_int), ("desire", ctypes.c_int),
                ("zone", ctypes.c_int), ("myCell", ctypes.c_int),
                ("coverage", ctypes.c_int), ("targetOwner", ctypes.c_int),
                ("congregationClosed", ctypes.c_int),
                ("arrivedAtPlace", ctypes.c_int), ("iFoundIt", ctypes.c_int)]


class LabOut(ctypes.Structure):
    _fields_ = [("navActive", ctypes.c_int), ("navPending", ctypes.c_int),
                ("meetActive", ctypes.c_int), ("stagingDone", ctypes.c_int),
                ("slotIndex", ctypes.c_int), ("slotTotal", ctypes.c_int),
                ("goalX", ctypes.c_float), ("goalY", ctypes.c_float),
                ("slotX", ctypes.c_float), ("slotY", ctypes.c_float),
                ("ring", ctypes.c_float), ("slotRadius", ctypes.c_float),
                ("queue", ctypes.c_int)]


def load_lib():
    here = os.path.dirname(os.path.abspath(__file__))
    # BDIE_LIB permite correr el banco contra otra biblioteca y comparar dos
        # versiones del agente con el mismo entorno y las mismas semillas.
    path = os.environ.get("BDIE_LIB") or os.path.join(here, "libbdie.so")
    if not os.path.exists(path):
        sys.exit("Falta libbdie.so — corra 'make -f Makefile.mk sitl' primero.")
    lib = ctypes.CDLL(path)
    lib.bdie_new.restype = ctypes.c_void_p
    lib.bdie_new.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float,
                             ctypes.c_float, ctypes.c_int, ctypes.c_int]
    lib.bdie_free.argtypes = [ctypes.c_void_p]
    lib.bdie_seed.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.bdie_feed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint]
    lib.bdie_sensor_raw.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint]
    lib.bdie_sensor_active.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.bdie_sensors.argtypes = [ctypes.c_void_p] * 5
    lib.bdie_arrived.argtypes = [ctypes.c_void_p]
    lib.bdie_slot_reached.argtypes = [ctypes.c_void_p]
    lib.bdie_meet_stale.argtypes = [ctypes.c_void_p]
    lib.bdie_meet_stale.restype = ctypes.c_int
    lib.bdie_block.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.bdie_cycle.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(COut)]
    lib.bdie_pop_wire.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.bdie_pop_wire.restype = ctypes.c_int
    for _f in ("bdie_telem_think", "bdie_telem_map"):
        getattr(lib, _f).argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_int]
        getattr(lib, _f).restype = ctypes.c_int
    lib.bdie_plan.argtypes = [ctypes.c_void_p]
    lib.bdie_plan.restype = ctypes.c_char_p
    for fn in ("bdie_repel", "bdie_visited", "bdie_zone_mask"):
        getattr(lib, fn).argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.bdie_evaporacion.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.bdie_exploracion.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_int]
    lib.bdie_formation.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float,
                                   ctypes.c_uint, ctypes.c_uint]
    lib.bdie_params.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
                                ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    # ── la congregación del laboratorio (lab_cong.cpp) ──
    lib.lab_new.restype = ctypes.c_void_p
    lib.lab_new.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.lab_free.argtypes = [ctypes.c_void_p]
    lib.lab_clock.argtypes = [ctypes.c_ulong]
    lib.lab_start_meet.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.lab_drop_meet.argtypes = [ctypes.c_void_p]
    lib.lab_neighbors.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.lab_position.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.lab_obstacles.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.lab_pop.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int),
                            ctypes.POINTER(ctypes.c_float)]
    lib.lab_pop.restype = ctypes.c_int
    lib.lab_guardian.argtypes = [ctypes.c_void_p]
    lib.lab_guardian.restype = ctypes.c_int
    lib.lab_take_arrival.argtypes = [ctypes.c_void_p]
    lib.lab_take_arrival.restype = ctypes.c_int
    lib.lab_state.argtypes = [ctypes.c_void_p, ctypes.POINTER(LabOut)]
    lib.lab_code.argtypes = [ctypes.c_char_p]
    lib.lab_code.restype = ctypes.c_int
    lib.bdie_alive_count.argtypes = [ctypes.c_void_p]
    lib.bdie_alive_count.restype = ctypes.c_int
    # ── reparto por mensajes (una biblioteca anterior puede no traerlo) ──
    if hasattr(lib, "bdie_comunicar"):
        lib.bdie_comunicar.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        lib.bdie_talking_mask.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        lib.bdie_talking_mask.restype = ctypes.c_uint
        _u = ctypes.POINTER(ctypes.c_uint)
        lib.bdie_rep_info.argtypes = [ctypes.c_void_p, _u, _u, ctypes.POINTER(ctypes.c_int),
                                      _u, _u, _u, _u, _u, _u]
        lib.bdie_rep_info.restype = ctypes.c_int
        lib.bdie_zone_of.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.bdie_zone_of.restype = ctypes.c_int
        lib.bdie_reparto_params.argtypes = [ctypes.c_void_p] + [ctypes.c_uint] * 6
    return lib


LIB = load_lib()
HAS_REP = hasattr(LIB, "bdie_comunicar")      # ¿la biblioteca reparte por mensajes?


def rep_info(h):
    """Estado del reparto de un agente (solo con la biblioteca nueva)."""
    v = [ctypes.c_uint() for _ in range(2)]
    ad = ctypes.c_int()
    w = [ctypes.c_uint() for _ in range(6)]
    zona = LIB.bdie_rep_info(h, ctypes.byref(v[0]), ctypes.byref(v[1]), ctypes.byref(ad),
                             *[ctypes.byref(x) for x in w])
    return dict(zona=zona, rep=v[0].value, mask=v[1].value, acordado=bool(ad.value),
                rondas=w[0].value, enviados=w[1].value, acuerdos=w[2].value,
                respaldos=w[3].value, a_epoch=w[4].value, a_mask=w[5].value)


def popcount(m):
    return bin(m).count("1")
L_WAIT, L_MOVE, L_TURN, L_REQ = (LIB.lab_code(n) for n in
                                 (b"WAIT", b"MOVE", b"TURN", b"REQUEST_POSITION"))


# =============================================================================
#  Un robot del banco: cuerpo físico + agente C++
# =============================================================================
class Bot:
    def __init__(self, rid, x, y, ang, seed, params):
        self.id = rid
        self.h = LIB.bdie_new(rid, ORIGIN_X, ORIGIN_Y, CELL_MM, GRID_ROWS, GRID_COLS)
        LIB.bdie_seed(self.h, ctypes.c_uint(seed))
        LIB.bdie_params(self.h, params["alpha"], params["beta"], params["rho"],
                        params["deposit"], params["commitment"],
                        params["no_progress_ms"], params["congregation_ms"],
                        int(MISSION_TIMEOUT_S * 1000))
        LIB.bdie_formation(self.h, params["formacion"], params["banda_mm"],
                           params["espera_ms"], params["gracia_ms"])
        LIB.bdie_evaporacion(self.h, params["evapora_ms"])
        LIB.bdie_exploracion(self.h, params["novedad"], params["frontera"])
        # El cuerpo del laboratorio para la congregación: su propio estado
        # (congregation, nav, disperse…) y el umbral de llegada de BdieInitGrid.
        self.lh = LIB.lab_new(rid, ARENA_W, ARENA_H, phys.ARRIVAL_THRESHOLD)
        self.lab_on = False          # ¿hay un MEET de la misión en curso?
        self.lab_req = False         # REQUEST_POSITION esperando que se libre el cono
        self.lab_idle = False        # ¿ya se le avisó al agente que el cuerpo quedó libre?
        self.lab_out = LabOut()
        self.rb = SimRobot(x=x, y=y, angle=ang, robot_id=rid)
        self.nav = ReactiveNav()
        self.alive = True

        # cuerpo: instrucción en curso
        self.turn_left = 0.0      # grados pendientes
        self.move_left = 0.0      # mm pendientes
        self.overhead = 0.0
        self.goal = None

        # telemetría
        self.out = COut()
        self.phase = PH_IDLE
        self.coverage = 0
        self.min_dist_target = 1e9
        self.min_dist_base = 1e9
        self.reached_place = False
        self.returned = False
        self.found_it = False
        self.esperas = 0            # veces que se detuvo por un compañero
        self.esperando = False      # ¿detenido AHORA por un compañero?
        self.ranura_ir = 0          # ranura temporal de los emisores IR
        self.t_closed = None
        self.t_arrived = None

    def busy(self):
        if self.turn_left != 0.0 or self.move_left > 0.0 or self.overhead > 0.0:
            return True
        if self.lab_on:
            LIB.lab_state(self.lh, ctypes.byref(self.lab_out))
            o = self.lab_out
            return bool(o.queue or o.navActive or o.navPending or self.lab_req)
        return False

    def drop_meet(self):
        """BdieDropMeet + BdieReleaseBody: apaga el MEET y suelta el cuerpo."""
        LIB.lab_drop_meet(self.lh)
        self.lab_on = False
        self.lab_req = False

    def reiniciar(self, params, seed):
        """El robot se reinicia: agente de fábrica (mapa vacío, sin sesión ni ID
        de misión), cuerpo quieto. La sesión le llega después por JOIN."""
        LIB.bdie_free(self.h)
        self.h = LIB.bdie_new(self.id, ORIGIN_X, ORIGIN_Y, CELL_MM, GRID_ROWS, GRID_COLS)
        LIB.bdie_seed(self.h, ctypes.c_uint(seed))
        LIB.bdie_params(self.h, params["alpha"], params["beta"], params["rho"],
                        params["deposit"], params["commitment"],
                        params["no_progress_ms"], params["congregation_ms"],
                        int(MISSION_TIMEOUT_S * 1000))
        LIB.bdie_formation(self.h, params["formacion"], params["banda_mm"],
                           params["espera_ms"], params["gracia_ms"])
        LIB.bdie_evaporacion(self.h, params["evapora_ms"])
        LIB.bdie_exploracion(self.h, params["novedad"], params["frontera"])
        self.drop_meet()
        self.nav.is_active = False
        self.turn_left = self.move_left = self.overhead = 0.0
        self.goal = None
        self.phase = PH_IDLE

    def free(self):
        LIB.bdie_free(self.h)
        LIB.lab_free(self.lh)


# =============================================================================
#  Una corrida
# =============================================================================
def par_en_cono(b, bots):
    """Réplica de IsRobotObstacle(): ¿algún par vivo está a menos de 260 mm y
    dentro del cono de alguno de los tres sensores IR?  Solo cuentan los pares
    que comparten ranura temporal de IR (ver IR_RANURAS)."""
    for o in bots:
        if o is b or not o.alive:
            continue
        if o.ranura_ir != b.ranura_ir:
            continue
        dx = o.rb.x - b.rb.x
        dy = o.rb.y - b.rb.y
        d = math.hypot(dx, dy)
        if d > ROBOT_OBSTACLE_MM:
            continue
        ang = math.degrees(math.atan2(dy, dx)) % 360
        for sect in (b.rb.angle, b.rb.angle - 30.0, b.rb.angle + 30.0):
            if abs(((ang - (sect % 360)) + 180) % 360 - 180) < PEER_SECTOR_DEG:
                return True
    return False


def run_once(seed, params, n_robots=5, loss=0.0, detect_mm=100.0,
             obstacles=0, kill_at_s=None, kill_id=None, bdi_mode="instruccion",
             trace=False, grabar=None, ir_modo="por_id", tope_s=None,
             telemetria=None, seguir_s=0.0, muerto_visible=True,
             silencio=None, reinicio=None, muerto_obstaculo=None,
             muerto_en_vecinos=None):
    """muerto_visible: un robot apagado sigue en la arena. La cámara lo sigue
        viendo (la base lo sigue difundiendo en POSES y en NEIGHBOR_POSITIONS) y para
        los demás es un obstáculo físico, como en el laboratorio. False hace que el
        robot apagado desaparezca de la difusión.
        silencio=(id, t0, t1): ese robot no envía ni recibe nada entre t0 y t1 (el
        canal del enjambre cae; sigue navegando). Equivale a BDIE_SILENCIO.
        reinicio=(id, t0, t1): se apaga en t0 y en t1 vuelve con el estado de
        fábrica; la base le envía SESSION|START solo a él (BASE.SESSION|JOIN).
        muerto_obstaculo / muerto_en_vecinos: solo para diagnóstico; separan los
        tres efectos de muerto_visible (en POSES, como obstáculo físico y en la
        lista NEIGHBOR_POSITIONS con que el MEET del laboratorio reparte ranuras).
        None = lo mismo que muerto_visible.

        telemetria: callable(t_s, id, texto) que recibe los THINK (2 Hz) y BMAP
        (1 Hz) de cada agente vivo, codificados por el C++ de telemetry.h, lo mismo
        que el robot le envía a la base. Además recibe cada POSES (id=-1).
        seguir_s: segundos que la simulación sigue después de que todos volvieron
        (0 = cortar de inmediato), para que el registro no termine en el instante
        exacto del último arribo."""
    rnd = random.Random(seed)
    random.seed(seed)          # attabot_sim usa random global para su ruido

    # ── Escenario aleatorizado: base en un borde, objetivo lejos de la base ──
    side = rnd.randrange(4)
    m = 300.0
    if side == 0:   bx, by = rnd.uniform(-ARENA_W/2+m, ARENA_W/2-m), -ARENA_H/2+m
    elif side == 1: bx, by = rnd.uniform(-ARENA_W/2+m, ARENA_W/2-m),  ARENA_H/2-m
    elif side == 2: bx, by = -ARENA_W/2+m, rnd.uniform(-ARENA_H/2+m, ARENA_H/2-m)
    else:           bx, by =  ARENA_W/2-m, rnd.uniform(-ARENA_H/2+m, ARENA_H/2-m)

    while True:
        tx = rnd.uniform(-ARENA_W/2+200, ARENA_W/2-200)
        ty = rnd.uniform(-ARENA_H/2+200, ARENA_H/2-200)
        if math.hypot(tx-bx, ty-by) > 900.0:
            break

    world = SimWorld(width=ARENA_W, height=ARENA_H)
    obs_cells = set()
    for _ in range(obstacles):
        while True:
            ox = rnd.uniform(-ARENA_W/2+300, ARENA_W/2-300)
            oy = rnd.uniform(-ARENA_H/2+300, ARENA_H/2-300)
            if math.hypot(ox-tx, oy-ty) > 500 and math.hypot(ox-bx, oy-by) > 600:
                break
        world.obstacles.append(Obstacle(x=ox, y=oy, radius=120.0))
        c = int((ox - ORIGIN_X) // CELL_MM); r = int((oy - ORIGIN_Y) // CELL_MM)
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            obs_cells.add(r * GRID_COLS + c)

    # ── Los robots salen agrupados junto a la base ───────────────────────────
    bots = []
    for i in range(n_robots):
        a = 2*math.pi*i/n_robots
        sx = max(-ARENA_W/2+150, min(ARENA_W/2-150, bx + 320*math.cos(a)))
        sy = max(-ARENA_H/2+150, min(ARENA_H/2-150, by + 320*math.sin(a)))
        b = Bot(i, sx, sy, rnd.uniform(0, 360), seed*97 + i*7 + 1, params)
        # ranura de IR: con el defecto del punto #10, todos sacan la misma
        # ranura de IR:
        #   "bug"       — el defecto del punto #10: random() sin sembrar, todos
        #                 sacan la misma y el anti-diafonía queda anulado
        #   "arreglado" — sorteo independiente por robot: con 5 robots sobre 18
        #                 ranuras, el 46 % de las corridas tiene ALGÚN par
        #                 compartiendo ranura, y ese par se estorba toda la corrida
        #   "por_id"    — ranura = robotID: con 5 ≤ 18 NINGÚN par coincide nunca.
        #                 Cuesta exactamente lo mismo que sortearla.
        if ir_modo == "por_id":   b.ranura_ir = i
        elif ir_modo == "arreglado": b.ranura_ir = rnd.randrange(IR_RANURAS)
        else:                     b.ranura_ir = 0
        bots.append(b)
    for b in bots:
        for c in obs_cells:
            LIB.bdie_block(b.h, c)

    buf = ctypes.create_string_buffer(160)

    # ── SESSION|START, repetido 3 veces porque UDP no garantiza entrega ──────
    start_msg = ("SESSION|START|1|%.0f|%.0f" % (bx, by)).encode()
    for b in bots:
        for _ in range(3):
            if rnd.random() >= loss:
                LIB.bdie_feed(b.h, start_msg, 1)
    if telemetria is not None:
        telemetria(0.0, -1, start_msg.decode())
        telemetria(0.0, -2, "OBJETIVO|%.0f|%.0f" % (tx, ty))   # verdad del banco

    # ── grabador de trazas ───────────────────────────────────────────────────
    if grabar is not None:
        grabar.update(dict(
            meta=dict(seed=seed, arena=[ARENA_W, ARENA_H],
                      origen=[ORIGIN_X, ORIGIN_Y], celda=CELL_MM,
                      filas=GRID_ROWS, cols=GRID_COLS,
                      base=[round(bx, 1), round(by, 1)],
                      objetivo=[round(tx, 1), round(ty, 1)],
                      radio_deteccion=detect_mm, perdida=loss,
                      n_robots=n_robots, bdi_mode=bdi_mode,
                      obstaculos=[[round(o.x, 1), round(o.y, 1), o.radius]
                                  for o in world.obstacles],
                      congregacion="MEET del laboratorio", ir_modo=ir_modo,
                      robot_obstaculo_mm=ROBOT_OBSTACLE_MM,
                      fases=PHASE_NAME, deseos=DESIRE_NAME,
                      pose_hz=GRABA_POSE_HZ, mapa_hz=GRABA_MAPA_HZ),
            planes=[""],                      # tabla de nombres; 0 = sin plan
            cuadros=dict(t=[], x=[], y=[], th=[], fase=[], deseo=[], plan=[],
                         gx=[], gy=[], cob=[], vivo=[], esp=[], nesp=[]),
            mapas=dict(t=[], pool=[], repel=[], visto=[], zona=[]),
            eventos=[], mensajes=[]))
        _ultimo = [dict(plan=None, fase=None, deseo=None) for _ in range(n_robots)]
        _t_pose = 0.0
        _t_mapa = 0.0
        _mapbuf = ctypes.create_string_buffer(MAX_CELLS_PY)
        _pool_idx = {}

    t = 0.0
    epoch = 1
    t_poses = 0.0
    t_vecinos = 0.0
    t_sensor = 0.0
    t_detect = None
    closed_at = None
    swarm_radii = []
    radio_al_abrir = None
    traza = []

    # ── métricas del reparto ─────────────────────────────────────────────────
    def mudo(b):
        """¿El canal del enjambre de este robot está caído ahora?"""
        return silencio is not None and b.id == silencio[0] and silencio[1] <= t < silencio[2]

    eventos_rep = []            # (t, tipo): instantes que exigen un reparto nuevo
    t_acuerdo0 = None           # primer acuerdo de toda la sesión
    t_reacuerdo = []            # (tipo, segundos desde el evento)
    _pend = []                  # eventos todavía sin re-acuerdo
    pasos_bus = pasos_desac = pasos_dup = 0
    msgs_rep = 0
    kz_al_hallazgo = None
    obst_muerto = {}
    if muerto_obstaculo is None:
        muerto_obstaculo = muerto_visible
    if muerto_en_vecinos is None:
        muerto_en_vecinos = muerto_visible

    limite = MISSION_TIMEOUT_S if tope_s is None else min(tope_s, MISSION_TIMEOUT_S)
    _t_tel = 0.0
    _t_todos = None
    _telbuf = ctypes.create_string_buffer(1024)
    while t < limite:
        now_ms = int(t * 1000) + 1

        # ── caída inducida ───────────────────────────────────────────────────
        def apagar(b, txt):
            b.alive = False
            if muerto_obstaculo:
                # sigue en la arena: un obstáculo más para los otros
                o = Obstacle(x=b.rb.x, y=b.rb.y, radius=ROBOT_RADIUS)
                world.obstacles.append(o)
                obst_muerto[b.id] = o
            eventos_rep.append((t, "caida"))
            _pend.append((t, "caida"))
            if grabar is not None:
                grabar["eventos"].append(dict(t=round(t, 2), id=b.id, tipo="caida", txt=txt))

        if kill_at_s is not None and t >= kill_at_s and bots[kill_id].alive \
                and not getattr(bots[kill_id], "_apagado", False):
            bots[kill_id]._apagado = True
            apagar(bots[kill_id], "FALLO INDUCIDO: se apaga")

        if reinicio is not None:
            rb_ = bots[reinicio[0]]
            if reinicio[1] <= t and rb_.alive and not getattr(rb_, "_reinicio", 0):
                rb_._reinicio = 1
                apagar(rb_, "REINICIO: se apaga")
            elif reinicio[2] <= t and getattr(rb_, "_reinicio", 0) == 1:
                rb_._reinicio = 2
                o = obst_muerto.pop(rb_.id, None)
                if o is not None and o in world.obstacles:
                    world.obstacles.remove(o)
                rb_.reiniciar(params, seed * 97 + rb_.id * 7 + 1001)
                rb_.alive = True
                for c in obs_cells:
                    LIB.bdie_block(rb_.h, c)
                join = ("SESSION|START|1|%.0f|%.0f" % (bx, by)).encode()
                for _ in range(3):                      # BASE.SESSION|JOIN, unicast
                    if rnd.random() >= loss:
                        LIB.bdie_feed(rb_.h, join, now_ms)
                eventos_rep.append((t, "reinicio"))
                _pend.append((t, "reinicio"))
                if grabar is not None:
                    grabar["eventos"].append(dict(t=round(t, 2), id=rb_.id, tipo="caida",
                                                  txt="REINICIO: vuelve con JOIN"))

        if silencio is not None:
            if abs(t - silencio[1]) < DT / 2:
                eventos_rep.append((t, "silencio")); _pend.append((t, "silencio"))
            if abs(t - silencio[2]) < DT / 2:
                eventos_rep.append((t, "vuelve")); _pend.append((t, "vuelve"))

        # ── broadcast de poses, con ruido de ArUco y pérdida ─────────────────
        if t >= t_poses:
            t_poses += POSES_PERIOD_S
            epoch += 1
            blob = []
            for b in bots:
                # La cámara ve al robot apagado igual que al vivo (muerto_visible)
                if not b.alive and not muerto_visible:
                    continue
                nx, ny, na = b.rb.get_pose_noisy()
                blob.append("%d:%.0f:%.0f:%.0f" % (b.id, nx, ny, na))
            msg = ("POSES|%d|%s;" % (epoch, ";".join(blob))).encode()
            if telemetria is not None:
                telemetria(t, -1, msg.decode())
            for b in bots:
                if b.alive and not mudo(b) and rnd.random() >= loss:
                    LIB.bdie_feed(b.h, msg, now_ms)

        # ── NEIGHBOR_POSITIONS a 1 Hz (base → cada robot, unicast, con pérdida)
        #    Es el insumo de MeetSlotIndex: todos reparten ranuras sobre el
        #    MISMO barrido. Como en AttaBot_Base.py, va la pose de cámara de
        #    cada robot, incluido el propio, en el marco de la cámara.
        if t >= t_vecinos:
            t_vecinos += NEIGHBOR_PERIOD_S
            items = []
            for b in bots:
                if not b.alive and not muerto_en_vecinos:
                    continue
                nx, ny, _ = b.rb.get_pose_noisy()
                items.append("%d,%.1f,%.1f" % (b.id, nx + LAB_DX, ny + LAB_DY))
            if len(items) >= 2:
                vmsg = ("NEIGHBOR_POSITIONS|" + ";".join(items)).encode()
                LIB.lab_clock(now_ms)
                for b in bots:
                    if b.alive and not mudo(b) and rnd.random() >= loss:
                        LIB.lab_neighbors(b.lh, vmsg)

        # ── sensor de color: LECTURA CRUDA, el voto 3-de-5 lo hace el C++ ────
        if t >= t_sensor:
            t_sensor += SENSOR_PERIOD_S
            for b in bots:
                if not b.alive:
                    continue
                activo = b.phase in (PH_IDLE, PH_DESPLEGAR, PH_BUSCAR)
                LIB.bdie_sensor_active(b.h, 1 if activo else 0)
                raw = 1 if math.hypot(b.rb.x - tx, b.rb.y - ty) <= detect_mm else 0
                LIB.bdie_sensor_raw(b.h, raw, now_ms)

        # ── agentes ──────────────────────────────────────────────────────────
        for b in bots:
            if not b.alive:
                continue
            corre = (not b.busy()) if bdi_mode == "instruccion" else True
            if corre:
                LIB.bdie_cycle(b.h, now_ms, ctypes.byref(b.out))
                b.phase = b.out.phase
                b.coverage = max(b.coverage, b.out.coverage)   # el agente borra la cobertura al terminar (FIN)
                if b.out.iFoundIt:
                    b.found_it = True

                # ── registro de DECISIONES: lo que el agente eligió y cuándo ──
                if grabar is not None:
                    _plan = LIB.bdie_plan(b.h).decode("utf-8", "replace") or "—"
                    if _plan not in grabar["planes"]:
                        grabar["planes"].append(_plan)
                    b.plan_txt = _plan
                    u = _ultimo[b.id]
                    if u["fase"] != b.out.phase:
                        grabar["eventos"].append(dict(
                            t=round(t, 2), id=b.id, tipo="fase",
                            txt="fase → %s" % PHASE_NAME[b.out.phase]))
                        u["fase"] = b.out.phase
                    if u["deseo"] != b.out.desire:
                        grabar["eventos"].append(dict(
                            t=round(t, 2), id=b.id, tipo="deseo",
                            txt="deseo → D_%s" % DESIRE_NAME[b.out.desire]))
                        u["deseo"] = b.out.desire
                    if u["plan"] != _plan:
                        grabar["eventos"].append(dict(
                            t=round(t, 2), id=b.id, tipo="plan",
                            txt="plan → %s" % _plan))
                        u["plan"] = _plan

                if b.out.congregationClosed:
                    if b.t_closed is None:
                        b.t_closed = t
                    if closed_at is None:
                        closed_at = t
                if b.out.arrivedAtPlace and not b.reached_place:
                    b.reached_place = True
                    b.t_arrived = t
                # StateDeliberate: fuera de la congregación el MEET se apaga
                if b.lab_on and b.out.phase != PH_CONGREGAR:
                    b.drop_meet()
                if not b.busy():
                    k = b.out.kind
                    if k in (A_GOTO_CELL, A_GOTO_MM):
                        g = (b.out.x, b.out.y)
                        if b.goal != g:
                            b.goal = g
                            b.nav.start(g[0], g[1])
                    elif k in (A_HOLD, A_ABORT):
                        b.nav.is_active = False
                        b.goal = None
                    elif k == A_MEET:                     # BdieStartMeet
                        b.nav.is_active = False
                        b.goal = (b.out.x, b.out.y)
                        LIB.lab_clock(now_ms)
                        LIB.lab_start_meet(b.lh, b.out.x + LAB_DX, b.out.y + LAB_DY)
                        b.lab_on = True
                        b.lab_idle = False

        # ── BdieLoopHook §2: latido y reparto por reloj propio, en CADA paso ─
        #    (en el robot, en cada loop(); no solo cuando el agente delibera)
        if HAS_REP:
            for b in bots:
                if b.alive:
                    LIB.bdie_comunicar(b.h, now_ms)

        # ── mensajes entre pares: UDP sin acuse, con pérdida ─────────────────
        for b in bots:
            if not b.alive:
                continue
            while True:
                n = LIB.bdie_pop_wire(b.h, buf, 160)
                if n == 0:
                    break
                if n < 0 or not buf.value:
                    continue
                _txt = buf.value.decode("utf-8", "replace")
                if _txt[:1] == "Z":
                    msgs_rep += 1
                if mudo(b):
                    continue                          # BDIE_SILENCIO: no sale
                _perdidos = 0
                for o in bots:
                    if o is b or not o.alive or mudo(o):
                        continue
                    if rnd.random() >= loss:
                        LIB.bdie_feed(o.h, buf.value, now_ms)
                    else:
                        _perdidos += 1
                if grabar is not None and not _txt.startswith("HB") and not _txt.startswith("ZC"):
                    grabar["mensajes"].append(dict(t=round(t, 2), de=b.id,
                                                   txt=_txt, perdidos=_perdidos))

        # ── ¿el enjambre está de acuerdo en el reparto? ─────────────────────
        #    Solo mientras se busca y NADIE conoce todavía el objetivo: después
        #    las franjas no importan. «De acuerdo» = todos los que buscan y se
        #    oyen dieron por acordada la MISMA ronda, con EXACTAMENTE ese
        #    conjunto, y franjas distintas. «Duplicado» = dos con la misma
        #    franja de la misma partición: el estado peligroso. La fracción en
        #    desacuerdo se cuenta DESPUÉS del primer acuerdo (el arranque se
        #    reporta aparte, como t_acuerdo_inicial).
        if HAS_REP:
            nadie_sabe = not any(b.alive and b.out.targetOwner >= 0 for b in bots)
            bus = [b for b in bots if b.alive and not mudo(b)
                   and b.phase in (PH_DESPLEGAR, PH_BUSCAR)] if nadie_sabe else []
            if bus:
                info = {b.id: rep_info(b.h) for b in bus}
                esperado = sum(1 << b.id for b in bus)
                reps = {i["rep"] for i in info.values()}
                zonas = [i["zona"] for i in info.values()]
                acuerdo = (len(reps) == 1 and all(i["acordado"] and i["mask"] == esperado
                                                  for i in info.values())
                           and len(set(zonas)) == len(zonas) and min(zonas) >= 0)
                dup = False
                vistos = {}
                for i in info.values():
                    if i["zona"] < 0:
                        continue
                    clave = (popcount(i["a_mask"]), i["zona"])
                    if clave in vistos:
                        dup = True
                    vistos[clave] = 1
                if len(bus) >= 2 and t_acuerdo0 is not None:
                    pasos_bus += 1
                    pasos_desac += 0 if acuerdo else 1
                    pasos_dup += 1 if dup else 0
                if acuerdo:
                    if t_acuerdo0 is None:
                        t_acuerdo0 = t
                    for ev in list(_pend):
                        if t > ev[0]:
                            t_reacuerdo.append((ev[1], round(t - ev[0], 2)))
                            _pend.remove(ev)

        # HandleBdieMessage: cambió el objetivo con un MEET en curso hacia el
        # punto viejo → se suelta el cuerpo; el agente pedirá el MEET nuevo.
        for b in bots:
            if b.alive and b.lab_on and LIB.bdie_meet_stale(b.h):
                b.drop_meet()
                b.turn_left = 0.0
                b.move_left = 0.0

        # ── cuerpo: máquina de estados de movimiento ─────────────────────────
        for b in bots:
            if not b.alive:
                continue
            # BdieLoopHook §1, en cada loop(): el guardián de la navegación
            if b.lab_on:
                LIB.lab_clock(now_ms)
                if LIB.lab_guardian(b.lh):
                    b.lab_on = False
                    b.lab_req = False
                    b.turn_left = b.move_left = b.overhead = 0.0
                    b.lab_idle = True
                    LIB.bdie_arrived(b.h)
            if b.overhead > 0.0:
                b.overhead = max(0.0, b.overhead - DT)
                if b.overhead == 0.0:
                    b.esperando = False
                continue

            if b.turn_left != 0.0:
                paso = min(abs(b.turn_left), TURN_DPS * DT)
                b.rb.angle = (b.rb.angle + math.copysign(paso, b.turn_left)) % 360
                b.turn_left -= math.copysign(paso, b.turn_left)
                if abs(b.turn_left) < 1e-6:
                    b.turn_left = 0.0
                continue

            if b.move_left > 0.0:
                v = MAX_SPEED_MMS if b.move_left > DECEL_MM else MIN_SPEED_MMS
                d = min(b.move_left, v * DT)
                rad = math.radians(b.rb.angle)
                nx = b.rb.x + math.cos(rad) * d
                ny = b.rb.y + math.sin(rad) * d
                # paredes y obstáculos
                fuera = not (-ARENA_W/2 + ROBOT_RADIUS <= nx <= ARENA_W/2 - ROBOT_RADIUS and
                             -ARENA_H/2 + ROBOT_RADIUS <= ny <= ARENA_H/2 - ROBOT_RADIUS)
                b.rb.x, b.rb.y = nx, ny
                choque = world.collides(b.rb)
                if fuera or choque:
                    b.rb.x -= math.cos(rad) * d
                    b.rb.y -= math.sin(rad) * d
                    b.move_left = 0.0
                    b.overhead = INSTR_OVERHEAD_S
                else:
                    b.move_left -= d
                    if b.move_left <= 1e-6:
                        b.move_left = 0.0
                        b.overhead = INSTR_OVERHEAD_S
                continue

            # ── congregación: el cuerpo ejecuta la cola del LABORATORIO ─────
            if b.lab_on:
                LIB.lab_clock(now_ms)
                if not b.lab_req:
                    kind, val = ctypes.c_int(), ctypes.c_float()
                    if LIB.lab_pop(b.lh, ctypes.byref(kind), ctypes.byref(val)):
                        b.lab_idle = False
                        k, v = kind.value, val.value
                        if k == L_TURN:
                            deg = math.degrees(v / CENTER_TO_WHEEL) + random.gauss(0, 0.5)
                            b.turn_left = deg
                        elif k == L_MOVE:
                            b.move_left = max(0.0, v * (1 + random.gauss(0, MOTOR_DRIFT_SIGMA)))
                        elif k == L_WAIT:
                            b.overhead = max(b.overhead, v / 1000.0)
                        elif k == L_REQ:
                            b.lab_req = True
                        continue
                if b.lab_req:
                    # El mismo modelo de «el compañero no se evade: se espera»
                    # que usa la navegación de búsqueda.
                    if ir_modo != "sin" and par_en_cono(b, bots):
                        b.overhead = IDENTIFY_WAIT_S
                        b.esperas += 1
                        b.esperando = True
                        continue
                    b.esperando = False
                    ir = world.check_ir(b.rb)
                    LIB.lab_obstacles(b.lh, int(ir['left']), int(ir['front']), int(ir['right']))
                    nx, ny, na = b.rb.get_pose_noisy()
                    b.lab_req = False
                    LIB.lab_position(b.lh, nx + LAB_DX, ny + LAB_DY, na)
                    if LIB.lab_take_arrival(b.lh):
                        LIB.bdie_slot_reached(b.h)
                    continue
                # cola vacía y sin navegación: el MEET terminó (o se rindió)
                LIB.lab_state(b.lh, ctypes.byref(b.lab_out))
                if not (b.lab_out.navActive or b.lab_out.navPending) and not b.lab_idle:
                    b.lab_idle = True
                    LIB.bdie_arrived(b.h)
                continue

            # cuerpo libre: pedirle el siguiente tramo a ReactiveNav
            if b.nav.is_active:
                ir = world.check_ir(b.rb)

                # ¿hay un COMPAÑERO en el cono de un sensor, a menos de 260 mm?
                # Si lo hay, el firmware no evade: espera. Se modela igual.
                if ir_modo != "sin" and par_en_cono(b, bots):
                    b.overhead = IDENTIFY_WAIT_S
                    b.esperas += 1
                    b.esperando = True
                    continue
                b.esperando = False

                res = b.nav.step(b.rb, ir)
                if res is None:
                    b.nav.is_active = False
                    b.goal = None
                    LIB.bdie_arrived(b.h)
                else:
                    arc, seg = res
                    deg = math.degrees(arc / CENTER_TO_WHEEL) + random.gauss(0, 0.5)
                    if abs(deg) > 5.0:
                        b.turn_left = deg
                    b.move_left = max(0.0, seg * (1 + random.gauss(0, MOTOR_DRIFT_SIGMA)))

        # ── telemetría ───────────────────────────────────────────────────────
        vivos = [b for b in bots if b.alive]
        # «Congregado» = el MEET del laboratorio declaró la llegada a la ranura
        # (agente.arrivedAtPlace, se registra arriba). Es lo mismo que el robot
        # reporta en THINK (lleg=1), así que banco y laboratorio miden igual.
        for b in vivos:
            dT = math.hypot(b.rb.x - tx, b.rb.y - ty)
            dB = math.hypot(b.rb.x - bx, b.rb.y - by)
            b.min_dist_target = min(b.min_dist_target, dT)
            if closed_at is not None:
                b.min_dist_base = min(b.min_dist_base, dB)
                if dB <= 350.0:
                    b.returned = True

        if t_detect is None and any(b.found_it for b in vivos):
            t_detect = t
            # ¿en cuántas franjas estaba repartida el área al hallar el objetivo?
            # Con un robot apagado, 4 si se re-repartió y 5 si no.
            if HAS_REP:
                kz = [popcount(rep_info(b.h)["a_mask"]) for b in vivos]
            else:
                kz = [LIB.bdie_alive_count(b.h) for b in vivos]
            kz_al_hallazgo = median(kz) if kz else None
            if grabar is not None:
                quien = next(b.id for b in vivos if b.found_it)
                grabar["eventos"].append(dict(
                    t=round(t, 2), id=quien, tipo="hallazgo",
                    txt="CONFIRMA el objetivo (voto 3-de-5)"))

        # RADIO DEL ENJAMBRE — se mide SOLO mientras todos los vivos están de
        # regreso. Medirlo desde el primer cierre metía en la cuenta a robots
        # que todavía venían llegando al objetivo, y eso no es «volver juntos»,
        # es «llegar y volver». La primera versión de esta métrica daba ~1 200 mm
        # por ese motivo, no por un defecto del plan de formación.
        if len(vivos) > 1 and all(b.phase == PH_VOLVER for b in vivos):
            r = max(math.hypot(p.rb.x - q.rb.x, p.rb.y - q.rb.y)
                    for i, p in enumerate(vivos) for q in vivos[i+1:])
            if radio_al_abrir is None:
                radio_al_abrir = r      # con cuánta dispersión ARRANCA el retorno
                foto = [(round(math.hypot(b.rb.x-tx, b.rb.y-ty)),
                         round(math.hypot(b.rb.x-bx, b.rb.y-by))) for b in vivos]
                traza.append(("FOTO_APERTURA t=%.1f" % t, foto))
            swarm_radii.append(r)

        if trace and abs(t % 10.0) < DT:
            traza.append((round(t, 1), [PHASE_NAME[b.phase] for b in vivos],
                          max(b.coverage for b in vivos)))

        # ── captura de la traza ──────────────────────────────────────────────
        if grabar is not None:
            if t >= _t_pose:
                _t_pose += 1.0 / GRABA_POSE_HZ
                C = grabar["cuadros"]
                C["t"].append(round(t, 2))
                for k, f in (("x", lambda b: round(b.rb.x, 1)),
                             ("y", lambda b: round(b.rb.y, 1)),
                             ("th", lambda b: round(b.rb.angle, 1)),
                             ("fase", lambda b: int(b.phase)),
                             ("deseo", lambda b: int(b.out.desire)),
                             ("cob", lambda b: int(b.coverage)),
                             ("vivo", lambda b: 1 if b.alive else 0),
                             ("esp", lambda b: 1 if b.esperando else 0),
                             ("nesp", lambda b: int(b.esperas))):
                    C[k].append([f(b) for b in bots])
                C["plan"].append([grabar["planes"].index(getattr(b, "plan_txt", ""))
                                  if getattr(b, "plan_txt", "") in grabar["planes"] else 0
                                  for b in bots])
                C["gx"].append([round(b.goal[0], 1) if b.goal else None for b in bots])
                C["gy"].append([round(b.goal[1], 1) if b.goal else None for b in bots])

            if t >= _t_mapa:
                _t_mapa += 1.0 / GRABA_MAPA_HZ
                M = grabar["mapas"]
                M["t"].append(round(t, 2))
                ncel = GRID_ROWS * GRID_COLS
                for campo, fn in (("repel", LIB.bdie_repel),
                                  ("visto", LIB.bdie_visited), ("zona", LIB.bdie_zone_mask)):
                    porbot = []
                    for b in bots:
                        fn(b.h, _mapbuf, ncel)
                        e = base64.b64encode(_mapbuf.raw[:ncel]).decode("ascii")
                        i = _pool_idx.get(e)
                        if i is None:
                            i = len(M["pool"])
                            _pool_idx[e] = i
                            M["pool"].append(e)
                        porbot.append(i)
                    M[campo].append(porbot)

        # ── telemetría a la "base" (solo observación) ────────────────────────
        if telemetria is not None and t >= _t_tel:
            _t_tel += 0.5
            _mapa = (int(round(t * 2)) % 2 == 0)
            for b in bots:
                if not b.alive:
                    continue
                if LIB.bdie_telem_think(b.h, now_ms, _telbuf, 1024) > 0:
                    telemetria(t, b.id, _telbuf.value.decode())
                if _mapa and LIB.bdie_telem_map(b.h, now_ms, _telbuf, 1024) > 0:
                    telemetria(t, b.id, _telbuf.value.decode())

        # ── terminación temprana ─────────────────────────────────────────────
        if closed_at is not None and all(b.returned for b in vivos):
            if _t_todos is None:
                _t_todos = t
            if t - _t_todos >= seguir_s:
                break

        t += DT

    vivos = [b for b in bots if b.alive]
    n_viv = len(vivos)
    n_cong = sum(1 for b in vivos if b.reached_place)
    n_ret = sum(1 for b in vivos if b.returned)
    umbral = math.ceil(0.80 * n_viv)

    localizado = t_detect is not None
    completa = (localizado and closed_at is not None and
                n_cong >= umbral and n_ret >= umbral and t < MISSION_TIMEOUT_S)

    # ¿La congregación cerró por quórum o la cerró el temporizador (5 min desde
    # el primer arribo)? Con un robot apagado que se sigue contando, solo el
    # temporizador puede cerrarla.
    _arr = [b.t_arrived for b in bots if b.t_arrived is not None]
    cierre_por_reloj = (closed_at is not None and bool(_arr) and
                        closed_at - min(_arr) >= params["congregation_ms"] / 1000.0 - 1.0)
    reparto = None
    if HAS_REP:
        _inf = [rep_info(b.h) for b in bots]
        reparto = dict(
            t_acuerdo_inicial=t_acuerdo0, t_reacuerdo=t_reacuerdo,
            sin_reacuerdo=[e[1] for e in _pend],
            frac_desacuerdo=(pasos_desac / pasos_bus) if pasos_bus else None,
            frac_duplicado=(pasos_dup / pasos_bus) if pasos_bus else None,
            mensajes=msgs_rep, rondas_max=max(i["rondas"] for i in _inf),
            respaldos=sum(i["respaldos"] for i in _inf))

    res = {
        "seed": seed,
        "base": (round(bx), round(by)), "objetivo": (round(tx), round(ty)),
        "dist_base_objetivo": round(math.hypot(tx - bx, ty - by)),
        "localizado": localizado, "t_detect": t_detect,
        "cerro": closed_at is not None, "t_cierre": closed_at,
        "n_vivos": n_viv, "n_congregados": n_cong, "n_retornados": n_ret,
        "frac_congregados": n_cong / n_viv, "frac_retornados": n_ret / n_viv,
        "mision_completa": completa, "t_fin": t,
        "cobertura": max((b.coverage for b in vivos), default=0),
        "esperas_por_par": sum(b.esperas for b in bots),
        "por_bot": [dict(id=b.id, vivo=b.alive, llego=b.reached_place,
                         volvio=b.returned, dmin=round(b.min_dist_target),
                         fase=PHASE_NAME[b.phase], deseo=DESIRE_NAME[b.out.desire],
                         plan=(LIB.bdie_plan(b.h) or b"").decode("utf-8","replace"),
                         sabia=bool(b.out.targetOwner >= 0), esperas=b.esperas)
                     for b in bots],
        "dist_min_al_lugar": [round(b.min_dist_target) for b in bots],
        "radio_enjambre_max": max(swarm_radii) if swarm_radii else None,
        "radio_enjambre_med": median(swarm_radii) if swarm_radii else None,
        "radio_al_abrir": radio_al_abrir,
        "t_cierre_por_bot": [b.t_closed for b in bots],
        "t_llegada_por_bot": [b.t_arrived for b in bots],
        "cierre_por_reloj": cierre_por_reloj,
        "kz_al_hallazgo": kz_al_hallazgo,
        "reparto": reparto,
        "traza": traza,
    }
    for b in bots:
        b.free()
    return res


# =============================================================================
#  Línea base: el RANDOM_WALK que YA está en el firmware — costo cero
#  (AttaBot.ino:804-825). No usa el agente: es el control experimental.
# =============================================================================
def run_baseline(seed, n_robots=5, detect_mm=100.0):
    rnd = random.Random(seed)
    random.seed(seed)
    side = rnd.randrange(4); m = 300.0
    if side == 0:   bx, by = rnd.uniform(-ARENA_W/2+m, ARENA_W/2-m), -ARENA_H/2+m
    elif side == 1: bx, by = rnd.uniform(-ARENA_W/2+m, ARENA_W/2-m),  ARENA_H/2-m
    elif side == 2: bx, by = -ARENA_W/2+m, rnd.uniform(-ARENA_H/2+m, ARENA_H/2-m)
    else:           bx, by =  ARENA_W/2-m, rnd.uniform(-ARENA_H/2+m, ARENA_H/2-m)
    while True:
        tx = rnd.uniform(-ARENA_W/2+200, ARENA_W/2-200)
        ty = rnd.uniform(-ARENA_H/2+200, ARENA_H/2-200)
        if math.hypot(tx-bx, ty-by) > 900.0:
            break

    rbs = []
    for i in range(n_robots):
        a = 2*math.pi*i/n_robots
        rbs.append(SimRobot(x=max(-ARENA_W/2+150, min(ARENA_W/2-150, bx+320*math.cos(a))),
                            y=max(-ARENA_H/2+150, min(ARENA_H/2-150, by+320*math.sin(a))),
                            angle=rnd.uniform(0, 360), robot_id=i))
    t = 0.0
    move_left = [0.0]*n_robots
    turn_left = [0.0]*n_robots
    hits = [0]*n_robots
    while t < MISSION_TIMEOUT_S:
        for i, rb in enumerate(rbs):
            if turn_left[i] != 0.0:
                paso = min(abs(turn_left[i]), TURN_DPS*DT)
                rb.angle = (rb.angle + math.copysign(paso, turn_left[i])) % 360
                turn_left[i] -= math.copysign(paso, turn_left[i])
                if abs(turn_left[i]) < 1e-6: turn_left[i] = 0.0
            elif move_left[i] > 0.0:
                d = min(move_left[i], MAX_SPEED_MMS*DT)
                rad = math.radians(rb.angle)
                nx, ny = rb.x + math.cos(rad)*d, rb.y + math.sin(rad)*d
                if not (-ARENA_W/2+ROBOT_RADIUS <= nx <= ARENA_W/2-ROBOT_RADIUS and
                        -ARENA_H/2+ROBOT_RADIUS <= ny <= ARENA_H/2-ROBOT_RADIUS):
                    move_left[i] = 0.0
                    turn_left[i] = rnd.uniform(90, 270)
                else:
                    rb.x, rb.y = nx, ny
                    move_left[i] -= d
            else:
                turn_left[i] = rnd.uniform(-180, 180)
                move_left[i] = rnd.uniform(150, 400)
            # sensor: mismo voto 3-de-5, 10 Hz
            if math.hypot(rb.x-tx, rb.y-ty) <= detect_mm:
                hits[i] += 1
                if hits[i] >= 3:
                    return {"t_detect": t, "localizado": True}
            else:
                hits[i] = 0
        t += DT
    return {"t_detect": None, "localizado": False}


# =============================================================================
#  Agregación y reporte
# =============================================================================
# Parámetros por omisión de la exploración (ver pheromone_map.h).
# novedad=1.0 y frontera=0 desactivan el bono de novedad y la búsqueda de
# frontera.
DEFAULTS = dict(alpha=1.5, beta=0.5, rho=8, deposit=120, novedad=0.3, frontera=1,
                # Período de evaporación. 4 s, no 1 s: con deposit = 40 un
                # depósito duraba ~34 s y se borraba antes de servir para
                # repeler a nadie. Y es el período, no rho, porque rho no es
                # palanca en este rango (ver pheromone_map.h).
                evapora_ms=4000,
                commitment=1, no_progress_ms=12000, congregation_ms=300000,
                formacion=1, banda_mm=900.0, espera_ms=8000, gracia_ms=3000)


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z*z/n
    c = (p + z*z/(2*n)) / d
    h = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / d
    return (max(0.0, c-h), min(1.0, c+h))


def resumen(rs, etiqueta):
    n = len(rs)
    loc = sum(1 for r in rs if r["localizado"])
    cer = sum(1 for r in rs if r["cerro"])
    com = sum(1 for r in rs if r["mision_completa"])
    td = [r["t_detect"] for r in rs if r["t_detect"] is not None]
    tf = [r["t_fin"] for r in rs if r["mision_completa"]]
    fc = [r["frac_congregados"] for r in rs]
    fr = [r["frac_retornados"] for r in rs]
    rad = [r["radio_enjambre_max"] for r in rs if r["radio_enjambre_max"] is not None]
    cov = [r["cobertura"] for r in rs]
    lo, hi = wilson(com, n)
    return dict(etiqueta=etiqueta, n=n,
                localizacion=loc/n, cierre=cer/n, completa=com/n,
                ic_lo=lo, ic_hi=hi,
                t_detect_med=median(td) if td else None,
                t_detect_p90=sorted(td)[int(0.9*(len(td)-1))] if td else None,
                t_fin_med=median(tf) if tf else None,
                frac_cong=sum(fc)/n, frac_ret=sum(fr)/n,
                radio_med=median(rad) if rad else None,
                cobertura_med=median(cov) if cov else None)


def fila(s):
    f = lambda v, d=1: ("%.*f" % (d, v)) if v is not None else "—"
    return ("  %-26s %4d   %5.1f%%   %5.1f%%   %5.1f%%  [%4.1f–%4.1f]   %6s   %6s   %5.1f%%  %5.1f%%"
            % (s["etiqueta"], s["n"], 100*s["localizacion"], 100*s["cierre"],
               100*s["completa"], 100*s["ic_lo"], 100*s["ic_hi"],
               f(s["t_detect_med"]), f(s["t_fin_med"]),
               100*s["frac_cong"], 100*s["frac_ret"]))


CABECERA = ("  %-26s %4s   %6s   %6s   %6s  %12s   %6s   %6s   %6s %6s"
            % ("bloque", "n", "local.", "cierre", "COMPL.", "IC 95%",
               "t_det", "t_fin", "cong.", "ret."))


def tabla_fallo(rs):
    """Lo que el fallo le hace al reparto y al cierre, corrida por corrida."""
    kz = [r["kz_al_hallazgo"] for r in rs if r["kz_al_hallazgo"] is not None]
    reloj = sum(1 for r in rs if r["cierre_por_reloj"])
    cer = sum(1 for r in rs if r["cerro"])
    print("     franjas al hallar el objetivo: %s · cierres por TEMPORIZADOR: %d de %d"
          % (", ".join("%g" % k for k in kz) if kz else "—", reloj, cer))
    reps = [r["reparto"] for r in rs if r.get("reparto")]
    if reps:
        tre = [x[1] for p in reps for x in p["t_reacuerdo"]]
        sin = sum(len(p["sin_reacuerdo"]) for p in reps)
        des = [p["frac_desacuerdo"] for p in reps if p["frac_desacuerdo"] is not None]
        dup = [p["frac_duplicado"] for p in reps if p["frac_duplicado"] is not None]
        t0 = [p["t_acuerdo_inicial"] for p in reps if p["t_acuerdo_inicial"] is not None]
        print("     primer acuerdo: mediana %s s · re-acuerdo tras cada evento: mediana %s s, "
              "máx %s s · eventos que el hallazgo dejó sin re-reparto: %d"
              % ("%.2f" % median(t0) if t0 else "—", "%.2f" % median(tre) if tre else "—",
                 "%.2f" % max(tre) if tre else "—", sin))
        print("     búsqueda en desacuerdo tras el primer acuerdo: mediana %.1f %% · con franja repetida: "
              "mediana %.2f %%, máx %.2f %% · mensajes ZC+ZA por corrida: mediana %d · "
              "respaldos: %d"
              % (100 * median(des) if des else 0, 100 * median(dup) if dup else 0,
                 100 * max(dup) if dup else 0, median([p["mensajes"] for p in reps]),
                 sum(p["respaldos"] for p in reps)))


# =============================================================================
#  Verificación del diseño en diez escenarios aleatorizados (--indicador)
#
#  Criterio: completar con éxito al menos el 80 % de las misiones en diez
#  escenarios aleatorizados de simulación SITL. Las semillas y los parámetros
#  están fijados de antemano; si se pudieran elegir después de ver el
#  resultado, el número no sería evidencia de nada.
# =============================================================================

# Las diez semillas son 1..10. No se eligieron: son los diez primeros enteros.
# Es la declaración más simple posible de «no se escogieron a conveniencia», y
# es verificable por cualquiera que lea esta línea.
SEMILLAS_INDICADOR = list(range(1, 11))

# Parámetros congelados. Los dos primeros todavía no están medidos y por eso se
# DECLARAN aquí con su justificación; se vuelven a correr los diez escenarios
# cuando la visita de medición los fije.
INDICADOR = dict(
    robots=5,
    loss=0.05,        # supuesto declarado: 5 % de pérdida de paquetes UDP
    detect=100.0,     # supuesto declarado: radio efectivo del sensor de color
    ir="por_id",      # ranura IR derivada del ID del robot
    umbral=0.80,      # el 80 % del enunciado → 8 de 10
)


def correr_indicador(salida):
    P = dict(DEFAULTS)
    print("\n  INDICADOR DE OE2 — protocolo congelado (ver PROTOCOLO_Indicador_OE2.md)")
    print("  «Completa con éxito al menos el 80 % de las misiones en diez")
    print("   escenarios aleatorizados de simulación con el SITL.»\n")
    print("  Congelado antes de correr:  semillas %s" % SEMILLAS_INDICADOR)
    print("                              %d robots · pérdida %.0f %% · radio de detección %.0f mm"
          % (INDICADOR["robots"], 100*INDICADOR["loss"], INDICADOR["detect"]))
    print("                              ranura IR = %s · timeout 20 min · umbral 8 de 10\n"
          % INDICADOR["ir"])

    print("  %-3s %-16s %-16s %6s   %-9s %-11s %-9s %-9s  %s"
          % ("#", "base (mm)", "objetivo (mm)", "sep.", "localiza", "congrega", "retorna",
             "t_fin", "MISIÓN"))
    print("  " + "-" * 112)

    filas = []
    exitos = 0
    for i, sd in enumerate(SEMILLAS_INDICADOR, 1):
        r = run_once(sd, P, INDICADOR["robots"], INDICADOR["loss"], INDICADOR["detect"],
                     ir_modo=INDICADOR["ir"])
        ok = r["mision_completa"]
        exitos += 1 if ok else 0
        filas.append(r)
        print("  %-3d %-16s %-16s %6d   %-9s %-11s %-9s %8.1f s  %s"
              % (i, "%s" % (r["base"],), "%s" % (r["objetivo"],), r["dist_base_objetivo"],
                 "sí" if r["localizado"] else "NO",
                 "%d/%d" % (r["n_congregados"], r["n_vivos"]),
                 "%d/%d" % (r["n_retornados"], r["n_vivos"]),
                 r["t_fin"], "COMPLETA" if ok else "— parcial"))

    n = len(SEMILLAS_INDICADOR)
    tasa = exitos / n
    lo, hi = wilson(exitos, n)
    cumple = tasa >= INDICADOR["umbral"]

    print("  " + "-" * 112)
    print("\n  RESULTADO:  %d de %d misiones completas  =  %.0f %%" % (exitos, n, 100*tasa))
    print("  UMBRAL:     80 % (8 de 10)")
    print("  VEREDICTO:  %s" % ("✅ SE CUMPLE" if cumple else "❌ NO SE CUMPLE"))
    print("\n  Intervalo de confianza de Wilson al 95 %%: [%.1f %% – %.1f %%]" % (100*lo, 100*hi))
    if lo < INDICADOR["umbral"]:
        print("  ⚠️  Con n = 10 el límite inferior del intervalo queda POR DEBAJO del umbral")
        print("      declarado, incluso con resultado perfecto. El indicador se cumple sobre el")
        print("      valor puntual, no sobre el intervalo, y el informe debe decirlo así.")

    tds = [r["t_detect"] for r in filas if r["t_detect"] is not None]
    print("\n  Detalle:  t de detección mediana %.1f s · duración mediana %.1f s · "
          "cobertura mediana %d/%d celdas"
          % (median(tds) if tds else -1,
             median([r["t_fin"] for r in filas]),
             int(median([r["cobertura"] for r in filas])),
             GRID_COLS * GRID_ROWS))

    salida["indicador"] = dict(
        enunciado="Completa con exito al menos el 80 % de las misiones en diez "
                  "escenarios aleatorizados de simulacion con el SITL.",
        semillas=SEMILLAS_INDICADOR, parametros=INDICADOR,
        exitos=exitos, n=n, tasa=tasa, cumple=cumple, ic=[lo, hi],
        escenarios=[{k: v for k, v in r.items() if k != "traza"} for r in filas])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=40)
    ap.add_argument("--robots", type=int, default=5)
    ap.add_argument("--loss", type=float, default=0.05)
    ap.add_argument("--detect", type=float, default=100.0)
    ap.add_argument("--bdi", choices=["instruccion", "timer"], default="instruccion")
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--campana", action="store_true")
    ap.add_argument("--barrido", action="store_true")
    ap.add_argument("--formacion", type=int, choices=[0, 1], default=None,
                    help="0 = vecino mas cercano (original), 1 = cota de radio")
    ap.add_argument("--banda", type=float, default=None)
    ap.add_argument("--ir", choices=["bug","arreglado","por_id","sin"], default="por_id",
                    help="asignación de la ranura temporal de los emisores IR")
    ap.add_argument("--traza-hasta", type=float, default=None,
                    help="corta la traza a N segundos (para no grabar 20 min de bloqueo)")
    ap.add_argument("--traza", type=str, default=None,
                    help="graba la traza completa de UNA corrida a un .json para el visor")
    ap.add_argument("--indicador", action="store_true",
                    help="corre el protocolo del indicador de OE2, congelado")
    ap.add_argument("--obstaculos", type=int, default=0,
                    help="obstáculos en el bloque nominal (el bloque donde muerden "
                         "los defectos de acercamiento 4 y 6)")
    ap.add_argument("--fallo", action="store_true",
                    help="20 corridas con un robot que se apaga EN BÚSQUEDA (a los 30 s) "
                         "y que la cámara sigue viendo")
    ap.add_argument("--reconexion", action="store_true",
                    help="20 corridas con silencio de 20 s y 20 con reinicio + JOIN")
    ap.add_argument("--muerto-invisible", action="store_true",
                    help="reproduce el banco anterior: el robot apagado desaparece de POSES")
    ap.add_argument("--json", type=str, default=None)
    a = ap.parse_args()
    MV = not a.muerto_invisible

    if a.formacion is not None:
        DEFAULTS["formacion"] = a.formacion
    if a.banda is not None:
        DEFAULTS["banda_mm"] = a.banda

    salida = {}
    print()
    print("=" * 118)
    print("  BDI-E · banco de pruebas — el MISMO C++ que corre en el ESP32, por ctypes")
    print("  Física: attabot_sim.py (copia verbatim) · σ_ArUco 30 mm · σ_ángulo 2° · deriva de motor 2 %")
    print("=" * 118)

    if a.traza:
        P = dict(DEFAULTS)
        g = {}
        r = run_once(a.seed0, P, a.robots, a.loss, a.detect, grabar=g, ir_modo=a.ir,
                     tope_s=a.traza_hasta)
        g["meta"]["resultado"] = {k: v for k, v in r.items()
                                  if k not in ("traza", "t_cierre_por_bot", "t_llegada_por_bot")}
        with open(a.traza, "w") as f:
            json.dump(g, f, separators=(",", ":"))
        kb = os.path.getsize(a.traza) / 1024.0
        print("\n  Traza de la semilla %d → %s  (%.0f kB)" % (a.seed0, a.traza, kb))
        print("  %d cuadros a %d Hz · %d instantáneas de mapa · %d eventos de decisión · %d mensajes"
              % (len(g["cuadros"]["t"]), GRABA_POSE_HZ, len(g["mapas"]["t"]),
                 len(g["eventos"]), len(g["mensajes"])))
        print("  misión %s · %d/%d congregan · %d/%d retornan · %.1f s"
              % ("COMPLETA" if r["mision_completa"] else "parcial",
                 r["n_congregados"], r["n_vivos"], r["n_retornados"], r["n_vivos"], r["t_fin"]))

    elif a.indicador:
        correr_indicador(salida)

    elif a.barrido:
        print("\n  BARRIDO DE SENSIBILIDAD — las dos entradas que la S5 todavía no midió\n")
        print(CABECERA)
        tabla = []
        for loss in (0.0, 0.05, 0.15, 0.30):
            for det in (60.0, 100.0, 150.0):
                rs = [run_once(a.seed0+i, DEFAULTS, a.robots, loss, det,
                               bdi_mode=a.bdi, ir_modo=a.ir) for i in range(a.runs)]
                s = resumen(rs, "pérdida %2d%% · radio %3d mm" % (int(100*loss), int(det)))
                tabla.append(s)
                print(fila(s))
        salida["barrido"] = tabla

    elif a.campana:
        print("\n  LA CAMPAÑA, TAL COMO ESTÁ DIMENSIONADA PARA OE4 (predicha en el banco)\n")
        print(CABECERA)
        tabla = []
        rs = [run_once(a.seed0+i, DEFAULTS, a.robots, a.loss, a.detect, bdi_mode=a.bdi,
                       ir_modo=a.ir) for i in range(20)]
        tabla.append(resumen(rs, "misión nominal")); print(fila(tabla[-1]))
        # La caída ocurre a los 30 s, en plena búsqueda, para ejercitar el fallo
                # que el bloque mide; más tarde, el robot se apagaría casi siempre con la
                # misión ya terminada. El robot apagado sigue en la arena
                # (muerto_visible), como en el laboratorio.
        rs = [run_once(a.seed0+500+i, DEFAULTS, a.robots, a.loss, a.detect,
                       kill_at_s=30.0, kill_id=2, bdi_mode=a.bdi, ir_modo=a.ir,
                       muerto_visible=MV) for i in range(5)]
        tabla.append(resumen(rs, "fallo inducido (R2, 30 s)")); print(fila(tabla[-1]))
        rs = [run_once(a.seed0+700+i, DEFAULTS, a.robots, a.loss, a.detect,
                       obstacles=3, bdi_mode=a.bdi, ir_modo=a.ir) for i in range(5)]
        tabla.append(resumen(rs, "con obstáculos")); print(fila(tabla[-1]))
        bl = [run_baseline(a.seed0+900+i, a.robots, a.detect) for i in range(10)]
        td = [r["t_detect"] for r in bl if r["t_detect"] is not None]
        print("\n  Línea base RANDOM_WALK (ya en el firmware, costo de implementación cero):")
        print("     localización %.0f%% de 10 corridas · t_detect mediana %s s"
              % (100*len(td)/len(bl), ("%.1f" % median(td)) if td else "—"))
        salida["campana"] = tabla
        salida["linea_base"] = dict(n=len(bl), localizacion=len(td)/len(bl),
                                    t_detect_med=median(td) if td else None)

    elif a.fallo:
        print("\n  FALLO INDUCIDO EN BÚSQUEDA — R2 se apaga a los 30 s y queda en la arena"
              "%s\n" % ("" if MV else " (INVISIBLE: banco anterior)"))
        print("  agente: %s" % ("reparto por mensajes (vivo = lo oigo)" if HAS_REP
                                else "reparto por conocimiento común (vivo = lo ve la cámara)"))
        rs = [run_once(a.seed0+500+i, DEFAULTS, a.robots, a.loss, a.detect,
                       kill_at_s=30.0, kill_id=2, bdi_mode=a.bdi, ir_modo=a.ir,
                       muerto_visible=MV) for i in range(20)]
        print(CABECERA)
        s = resumen(rs, "fallo en búsqueda (n=20)"); print(fila(s))
        tabla_fallo(rs)
        salida["fallo"] = dict(resumen=s, corridas=[{k: v for k, v in r.items() if k != "traza"}
                                                     for r in rs])

    elif a.reconexion:
        print("\n  DESCONEXIÓN Y RECONEXIÓN — R2, 20 corridas por caso\n")
        print(CABECERA)
        out = {}
        for nombre, kw in (("silencio 30–50 s", dict(silencio=(2, 30.0, 50.0))),
                           ("reinicio 30 s, JOIN 60 s", dict(reinicio=(2, 30.0, 60.0)))):
            rs = [run_once(a.seed0+600+i, DEFAULTS, a.robots, a.loss, a.detect,
                           bdi_mode=a.bdi, ir_modo=a.ir, muerto_visible=MV, **kw)
                  for i in range(20)]
            s = resumen(rs, nombre); print(fila(s))
            tabla_fallo(rs)
            out[nombre] = dict(resumen=s, corridas=[{k: v for k, v in r.items() if k != "traza"}
                                                     for r in rs])
        salida["reconexion"] = out

    else:
        rs = [run_once(a.seed0+i, DEFAULTS, a.robots, a.loss, a.detect, bdi_mode=a.bdi,
                       ir_modo=a.ir, obstacles=a.obstaculos) for i in range(a.runs)]
        s = resumen(rs, "%s (pérdida %d%%, radio %d mm)"
                    % ("con %d obstáculos" % a.obstaculos if a.obstaculos else "nominal",
                       int(100*a.loss), int(a.detect)))
        print("\n" + CABECERA)
        print(fila(s))
        print(("\n  cobertura mediana %s de " + str(GRID_COLS*GRID_ROWS) + " celdas · RADIO DEL ENJAMBRE en el retorno: mediana %s mm")
              % (s["cobertura_med"], ("%.0f" % s["radio_med"]) if s["radio_med"] else "—"))
        salida["nominal"] = [s]

    print()
    print("-" * 118)
    print("  ⚠️  Esto es una PREDICCIÓN del banco, no un resultado de OE4. El banco es medio de")
    print("      ingeniería, nunca entregable (R1). La validación es el enjambre físico.")
    print("-" * 118)
    print()

    if a.json:
        with open(a.json, "w") as f:
            json.dump(salida, f, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
