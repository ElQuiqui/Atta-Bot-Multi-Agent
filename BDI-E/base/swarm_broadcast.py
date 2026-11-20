"""
swarm_broadcast.py — la base como servicio de localización
===========================================================

En la arquitectura BDI-E la PC no decide. Tiene dos responsabilidades:

  1. Difundir por broadcast las poses que ve con ArUco (~10 Hz).
  2. Dar la señal de inicio (y de aborto) de la sesión.

Toda la deliberación (dónde explorar, cuándo anunciar, quién ocupa qué lugar,
cuándo volver) ocurre en los ESP32. La base es reemplazable: cualquier fuente de
poses (UWB, captura de movimiento, otra cámara) sirve sin tocar el firmware. El
formato de los mensajes coincide byte a byte con firmware/mission.h §1.

Un solo paquete de poses por cuadro, en vez de un unicast por robot, le dice a
cada robot dónde están todos los demás, que es lo que permite coordinarse sin
negociar.

Integración en AttaBot_Base.py (los tres puntos están marcados «BDI-E»):

1) En Base.__init__, después de configurar el socket:

       from swarm_broadcast import SwarmBroadcaster
       self.swarm = SwarmBroadcaster(self.sock, self.broadcastIP, self.port,
                                     log=self.log)

2) Al enviar las poses, se acumulan y se difunden una sola vez por cuadro:

       poses[robot.id] = (x, y, angle)
       ...
       self.swarm.broadcast_poses(poses)

3) Los comandos de sesión:

       elif robotId == 'SESSION':
           if instruction.upper().startswith('START'):
               self.swarm.start_session(homeX=..., homeY=...)
           elif instruction.upper().startswith('ABORT'):
               self.swarm.abort_session()

ArUco, calibración, registros y video quedan intactos, y POSE|x|y|angle se
puede seguir enviando en paralelo.
"""

import time


# El firmware lee en `char receivedPacket[255]` (AttaBot.ino), así que el
# paquete completo debe quedar holgadamente por debajo de ese límite.
MAX_PACKET_BYTES = 240


class SwarmBroadcaster:
    """Servicio de difusión de poses e inicio de sesión."""

    def __init__(self, sock, broadcast_ip, port, log=None):
        self.sock = sock
        self.broadcast_ip = broadcast_ip
        self.port = port
        self.epoch = 0
        self.session = 0
        self._log = log or (lambda m: print(m))
        self._last_sent = 0.0
        self.min_interval = 0.05          # 20 Hz máximo

    # ── Construcción del paquete ──────────────────────────────────────────
    @staticmethod
    def build_poses_packet(epoch, poses):
        """
        poses: dict {robot_id: (x_mm, y_mm, angle_deg)}

        Formato:  POSES|<epoch>|<id>:<x>:<y>:<a>;<id>:<x>:<y>:<a>;...

        Son 3 campos separados por '|', así que pasa por SeparateCommand() del
        firmware sin ampliar su std::array<String,5>. El blob interno se analiza
        aparte dentro de la rama POSES. Idéntico a mission.h §1 y verificado por
        la prueba T10.
        """
        chunks = []
        for rid in sorted(poses, key=lambda k: int(k)):
            x, y, a = poses[rid]
            chunks.append("%d:%.0f:%.0f:%.0f" % (int(rid), x, y, a))
        return "POSES|%d|%s" % (epoch, ";".join(chunks))

    # ── Difusión ──────────────────────────────────────────────────────────
    def broadcast_poses(self, poses):
        """Difunde las poses visibles. Ignora robots sin detección ArUco."""
        visible = {rid: p for rid, p in poses.items()
                   if p is not None and p[0] != -1}
        if not visible:
            return False

        now = time.time()
        if now - self._last_sent < self.min_interval:
            return False
        self._last_sent = now

        # La época avanza en cada paquete. Roster::Alive() (assignment.h) declara
                # caído a un robot cuando epoch - p.epoch > staleEpochs, y ese contador es
                # el de la difusión, no el reloj local, para que todos los robots den la
                # baja con el mismo paquete. Con una época fija la resta daría siempre
                # cero y nadie se declararía caído.
                #
                # Con 30 épocas de tolerancia y ~10 Hz de cámara, 3 s de ausencia.
        self.epoch += 1

        packet = self.build_poses_packet(self.epoch, visible)

        # Si entraran más robots de los que caben en el buffer del firmware, se
        # parte en varios paquetes en vez de truncar en silencio. Las dos partes
        # comparten epoch a propósito: son el mismo instante.
        if len(packet.encode()) > MAX_PACKET_BYTES:
            for part in self._split(visible):
                self._send(self.build_poses_packet(self.epoch, part))
            return True

        self._send(packet)
        return True

    def _split(self, poses):
        ids = sorted(poses, key=lambda k: int(k))
        half = max(1, len(ids) // 2)
        return [{i: poses[i] for i in ids[:half]},
                {i: poses[i] for i in ids[half:]}]

    def _send(self, packet):
        self.sock.sendto(packet.encode(), (self.broadcast_ip, self.port))

    # ── Control de sesión ─────────────────────────────────────────────────
    def start_session(self, homeX=0.0, homeY=0.0, repeats=3):
        """
        Arranca una sesión. Es la ÚNICA orden que la base da en toda la misión.
        Se repite porque UDP no garantiza entrega y un robot que se pierda el
        inicio se queda en IDLE toda la corrida.
        """
        # La época se reinicia al abrir la sesión y no da la vuelta dentro de
                # ella: un contador que diera la vuelta rompería la resta sin signo del
                # firmware y todo el enjambre se declararía caído.
        self.epoch = 0
        self.session += 1
        msg = self.session_start_packet(homeX, homeY)
        for _ in range(repeats):
            self._send(msg)
            time.sleep(0.05)
        self._log("[Sesión] %d iniciada. Base en (%.0f, %.0f). "
                  "La deliberación es de los robots." % (self.session, homeX, homeY))
        return self.session

    def session_start_packet(self, homeX=0.0, homeY=0.0):
        """El SESSION|START de la sesión vigente. Lo reusa BASE.SESSION|JOIN para
        mandárselo SOLO a un robot que se reinició (unicast)."""
        return "SESSION|START|%d|%.0f|%.0f" % (self.session, homeX, homeY)

    def abort_session(self, repeats=3):
        for _ in range(repeats):
            self._send("SESSION|ABORT")
            time.sleep(0.05)
        self._log("[Sesión] Aborto difundido.")


# ── Observador pasivo ─────────────────────────────────────────────────────
class MissionObserver:
    """
        Registra la misión sin participar en ella. Escucha los latidos (HB) y los
        anuncios (TGT) que los robots se difunden entre sí y calcula métricas. Es
        pasivo a propósito: si se apaga, la misión sigue igual, lo que muestra que
        la base no decide nada.

        Se engancha en readUdpConnection() de AttaBot_Base.py:

            elif command == 'HB':
                self.observer.on_heartbeat(parts, timeLog)
            elif command == 'TGT':
                self.observer.on_target(parts, timeLog)
        """

    # Las fases son las de bdi.h (siete). Si esta tabla se desalinea del enum del
        # firmware, el registro queda mal etiquetado sin que nadie lo note.
    PHASES = ["IDLE", "DESPLEGAR", "BUSCAR", "CONGREGAR", "VOLVER", "FIN", "SEGURO"]

    def __init__(self, log=None):
        self.phases = {}
        self.zones = {}          # id → (ronda, participantes, franja) del último ZA
        self.events = []
        self.t_found = None
        self.t_congregated = None
        self.t_home = None
        self._log = log or (lambda m: print(m))

    def on_heartbeat(self, parts, t):
        # HB|<id>|<fase>|<flags>|<seq>
        if len(parts) < 4:
            return
        rid, ph = parts[1], int(parts[2])
        name = self.PHASES[ph] if 0 <= ph < len(self.PHASES) else str(ph)
        if self.phases.get(rid) != name:
            self.phases[rid] = name
            self.events.append((t, rid, name))
            self._log("[%7.1fs] Robot %s -> %s" % (t, rid, name))
            if name == "CONGREGAR" and self._all_in(("CONGREGAR", "VOLVER", "FIN")):
                self.t_congregated = self.t_congregated or t
            if name == "FIN" and self._all_in(("FIN",)):
                self.t_home = self.t_home or t

    def on_target(self, parts, t):
        # TGT|<epoch>|<id>|<x>|<y>
        if len(parts) < 5 or self.t_found is not None:
            return
        self.t_found = t
        self._log("[%7.1fs] OBJETIVO anunciado por robot %s en (%s, %s)"
                  % (t, parts[2], parts[3], parts[4]))

    def on_zone(self, parts, t):
        """ZA|ronda:part|id|franja|dist — un robot dio por ACORDADO el reparto y
        lo reporta. Se registra una vez por (robot, ronda, franja): el ZA se
        repite cada 2 s. Devuelve el texto del evento si es nuevo."""
        if len(parts) < 5:
            return None
        try:
            rep_s, _, part_s = parts[1].partition(":")
            rep, part = int(rep_s), int(part_s or "0", 16)
            rid, zona = parts[2], int(parts[3])
        except ValueError:
            return None
        clave = (rep, part, zona)
        if self.zones.get(rid) == clave:
            return None
        self.zones[rid] = clave
        k = bin(part).count("1")
        quien = ",".join(str(i) for i in range(8) if part >> i & 1)
        txt = "ronda %d · franja %d de %d · participantes {%s}" % (rep, zona, k, quien)
        self.events.append((t, rid, "ZA " + txt))
        self._log("[%7.1fs] Robot %s: reparto acordado, %s" % (t, rid, txt))
        return txt

    def _all_in(self, names):
        return bool(self.phases) and all(v in names for v in self.phases.values())
