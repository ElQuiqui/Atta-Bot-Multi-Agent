"""
bdie_monitor.py — qué está pensando cada robot, visto y guardado desde la base

Recibe la telemetría del firmware (THINK a 2 Hz, BMAP a 1 Hz, ver
firmware/telemetry.h), guarda lo último de cada robot para la GUI, escribe los
logs de la sesión y corre EN VIVO las verificaciones de bdie_checks.py.

Es solo observación: si esto se cae, la misión sigue igual. La base lo
alimenta desde readUdpConnection() (THINK, BMAP), _broadcastSwarmPoses() (la
pose de cámara de cada robot) y handleSession() (inicio de sesión).

Logs, en BdieLogs/, un juego por sesión con el mismo prefijo BDIE_<fecha>_:
  think.csv     cada THINK recibido, una fila, columnas fijas
  eventos.csv   solo los CAMBIOS: fase, deseo, plan, meta, zona, objetivo,
                cierre, vivos… y las alertas de las verificaciones
  mapas.jsonl   cada BMAP decodificado (feromonas, cobertura y zonas por robot)
  poses.csv     la pose de cámara de cada robot, hasta 20 Hz
La misma carpeta la lee herramientas/verificar_bdie.py para el reporte.
"""
import csv
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

import bdie_checks as CK

# rep / racu / zfb: ronda del reparto por mensajes, si el robot la dio por
# acordada y cuántas veces usó el reparto de respaldo. vivos = los robots que el
# robot oye.
CAMPOS_THINK = ["tlog", "id", "t", "fase", "deseo", "plan", "meta", "celda", "zona",
                "enzona", "x", "y", "obj", "dueno", "cobj", "cerr", "lleg", "meet",
                "vivos", "delib", "cob", "ev", "tev", "reint", "sinprog",
                "rep", "racu", "zfb"]
NUMERICOS = {"id", "t", "meta", "celda", "zona", "enzona", "obj", "dueno", "cobj",
             "cerr", "lleg", "meet", "delib", "cob", "tev", "reint", "sinprog",
             "rep", "racu", "zfb"}
# Lo que se registra como EVENTO cuando cambia.
CAMBIOS = ["fase", "deseo", "plan", "meta", "zona", "obj", "dueno", "cerr", "lleg",
           "vivos", "ev", "rep", "racu", "zfb"]


def parse_kv(texto):
    """'THINK|id=2|fase=BUSCAR' → ('THINK', {'id': 2, 'fase': 'BUSCAR'})."""
    partes = texto.strip().split("|")
    d = {}
    for p in partes[1:]:
        if "=" not in p:
            continue
        k, v = p.split("=", 1)
        if k in NUMERICOS:
            try:
                v = int(v)
            except ValueError:
                pass
        elif k in ("x", "y"):
            try:
                v = float(v)
            except ValueError:
                pass
        d[k] = v
    return partes[0], d


def _hex_bytes(h):
    return list(bytes.fromhex(h)) if h else []


def _bits(h, n):
    b = _hex_bytes(h)
    return [1 if (i >> 3) < len(b) and (b[i >> 3] >> (i & 7)) & 1 else 0 for i in range(n)]


def decode_map(d):
    """Un BMAP (ya partido por parse_kv) → estructura para la GUI y los checks."""
    filas, cols, celda, ox, oy = d["g"].split(",")
    filas, cols = int(filas), int(cols)
    n = filas * cols
    zonas = []
    if d.get("z"):
        for z in d["z"].split(";"):
            zonas.append(tuple(int(v) for v in z.split(".")))
    de = {}
    if d.get("of"):
        for par in d["of"].split(","):
            rid, zi = par.split(":")
            de[int(rid)] = int(zi)
    return dict(id=int(d["id"]), t=int(d["t"]), filas=filas, cols=cols,
                celda=float(celda), ox=float(ox), oy=float(oy), zonas=zonas, de=de,
                R=_hex_bytes(d.get("R", ""))[:n],
                V=_bits(d.get("V", ""), n), B=_bits(d.get("B", ""), n))


class BdieMonitor:
    def __init__(self, carpeta="BdieLogs", log=print, escribir=True):
        self.carpeta = carpeta
        self.log = log
        self.escribir = escribir
        self._lock = threading.Lock()
        self.reset()

    # ── sesión ────────────────────────────────────────────────────────────────
    def reset(self):
        self.think = {}          # id → último THINK (dict) con 'tlog'
        self.mapas = {}          # id → último BMAP decodificado con 'tlog'
        self.camara = {}         # id → (x, y, ang, tlog) de la cámara
        self.home = None         # (x, y) de SESSION START
        self.objetivo_real = None  # solo en el banco
        self.color = None          # color objetivo de la sesión (TARGET_COLOR)
        self.alertas = deque(maxlen=200)
        self.eventos = deque(maxlen=400)
        self._ultimaAlerta = {}  # (código, texto) → tlog, para no repetir
        self._cobMax = {}
        self.t0 = time.time()
        self.sesion = False
        self._files = None
        self._tPoseLog = 0.0
        self._racha = {}         # clave → cuántas veces seguidas se vio
        self._silencio = set()
        self._prog = {}          # id → (meta, mejor distancia, tlog)

    def new_session(self, hx, hy, tlog=None, color=None):
        with self._lock:
            self._cerrar()
            self.reset()
            self.home = (hx, hy)
            self.color = color
            self.sesion = True
            self._abrir()
            extra = f" color={color}" if color else ""
            self._evento(tlog or 0.0, -1, "sesion", f"START base=({hx:.0f},{hy:.0f}){extra}")

    def _abrir(self):
        if not self.escribir:
            return
        os.makedirs(self.carpeta, exist_ok=True)
        pref = os.path.join(self.carpeta, "BDIE_" + datetime.now().strftime("%d-%m_%H-%M-%S") + "_")
        self.prefijo = pref
        f_th = open(pref + "think.csv", "w", newline="", encoding="utf-8")
        f_ev = open(pref + "eventos.csv", "w", newline="", encoding="utf-8")
        f_po = open(pref + "poses.csv", "w", newline="", encoding="utf-8")
        f_ma = open(pref + "mapas.jsonl", "w", encoding="utf-8")
        w_th = csv.DictWriter(f_th, CAMPOS_THINK, extrasaction="ignore")
        w_th.writeheader()
        w_ev = csv.writer(f_ev); w_ev.writerow(["tlog", "id", "tipo", "detalle"])
        w_po = csv.writer(f_po); w_po.writerow(["tlog", "id", "x", "y", "ang"])
        meta = dict(tipo="meta", home=self.home, objetivo_real=self.objetivo_real,
                    color=self.color,
                    creado=datetime.now().isoformat(timespec="seconds"))
        f_ma.write(json.dumps(meta) + "\n")
        self._files = dict(th=(f_th, w_th), ev=(f_ev, w_ev), po=(f_po, w_po), ma=f_ma)

    def _cerrar(self):
        if self._files:
            for v in self._files.values():
                (v[0] if isinstance(v, tuple) else v).close()
        self._files = None

    def cerrar(self):
        with self._lock:
            self._cerrar()

    def _flush(self):
        if self._files:
            for v in self._files.values():
                (v[0] if isinstance(v, tuple) else v).flush()

    # ── entradas ──────────────────────────────────────────────────────────────
    def set_objetivo_real(self, x, y):
        """Solo en el banco: dónde está de verdad el objetivo, para el reporte."""
        with self._lock:
            self.objetivo_real = (x, y)
            if self._files:
                self._files["ma"].write(json.dumps(dict(tipo="meta", home=self.home,
                                                        objetivo_real=[x, y])) + "\n")

    def on_message(self, texto, tlog):
        """THINK o BMAP tal como llegó por UDP."""
        try:
            cmd, d = parse_kv(texto)
            if cmd == "THINK":
                self.on_think(d, tlog)
            elif cmd == "BMAP":
                self.on_map(decode_map(d), tlog)
        except (ValueError, KeyError) as e:
            self._alerta(tlog, -1, "V0", f"telemetría malformada ({e}): {texto[:60]}")

    def on_think(self, d, tlog):
        with self._lock:
            if not self.sesion and d.get("fase") not in ("IDLE", None):
                self.sesion = True          # la base arrancó con la misión en curso
                if self._files is None:
                    self._abrir()
            d["tlog"] = tlog
            rid = d["id"]
            prev = self.think.get(rid)
            self.think[rid] = d
            if self._files:
                self._files["th"][1].writerow(d)
            self._cambios(prev, d, tlog)
            self._checks_think(prev, d, tlog)

    def on_map(self, m, tlog):
        with self._lock:
            m["tlog"] = tlog
            self.mapas[m["id"]] = m
            if self._files:
                reg = dict(m); reg["tipo"] = "mapa"
                self._files["ma"].write(json.dumps(reg, separators=(",", ":")) + "\n")
            self._checks_mapa(m, tlog)

    def on_poses(self, poses, tlog):
        """poses: {id(str|int): (x, y, ang)} de la cámara."""
        with self._lock:
            for rid, (x, y, a) in poses.items():
                self.camara[int(rid)] = (x, y, a, tlog)
            if self._files and tlog - self._tPoseLog >= 0.05:    # hasta 20 Hz: el retorno se decide por decenas de mm
                self._tPoseLog = tlog
                w = self._files["po"][1]
                for rid, (x, y, a) in poses.items():
                    w.writerow([f"{tlog:.2f}", rid, f"{x:.0f}", f"{y:.0f}", f"{a:.0f}"])

    def on_poses_text(self, texto, tlog):
        """El mismo POSES que difunde la base: 'POSES|epoch|id:x:y:a;…'."""
        partes = texto.split("|")
        if len(partes) < 3:
            return
        poses = {}
        for it in partes[2].split(";"):
            if it.count(":") == 3:
                rid, x, y, a = it.split(":")
                poses[int(rid)] = (float(x), float(y), float(a))
        self.on_poses(poses, tlog)

    # ── eventos y alertas ─────────────────────────────────────────────────────
    def _evento(self, tlog, rid, tipo, detalle):
        self.eventos.append((tlog, rid, tipo, detalle))
        if self._files:
            self._files["ev"][1].writerow([f"{tlog:.2f}", rid, tipo, detalle])

    def _alerta(self, tlog, rid, codigo, texto, cada_s=5.0):
        clave = (codigo, rid)       # una alerta por código y robot cada `cada_s`
        if tlog - self._ultimaAlerta.get(clave, -1e9) < cada_s:
            return
        self._ultimaAlerta[clave] = tlog
        self.alertas.append((tlog, rid, codigo, texto))
        self._evento(tlog, rid, "ALERTA " + codigo, texto)
        try:
            self.log(f"⚠ BDI-E {codigo}: {texto}")
        except Exception:
            pass

    def _cambios(self, prev, d, tlog):
        rid = d["id"]
        if prev is None:
            self._evento(tlog, rid, "aparece", f"fase={d.get('fase')} zona={d.get('zona')}")
            return
        for k in CAMBIOS:
            if k == "ev" and d.get("tev") == prev.get("tev"):
                continue
            if d.get(k) != prev.get(k):
                self._evento(tlog, rid, k, f"{prev.get(k)} → {d.get(k)}")

    # ── verificaciones en vivo ────────────────────────────────────────────────
    def _checks_think(self, prev, d, tlog):
        rid = d["id"]
        if prev is not None and not CK.transicion_legal(prev.get("fase"), d.get("fase")):
            self._alerta(tlog, rid, "V5", f"R{rid}: transición ilegal {prev['fase']} → {d['fase']}",
                         cada_s=0)
        cob = d.get("cob", 0)
        # Al abortar o terminar (SEGURO/FIN/IDLE) el robot borra a propósito la
                # memoria de la corrida; que la cobertura baje ahí no es un defecto.
        if d.get("fase") in ("SEGURO", "FIN", "IDLE"):
            self._cobMax[rid] = cob
        if prev is not None and d.get("fase") == prev.get("fase") and cob < self._cobMax.get(rid, 0):
            self._alerta(tlog, rid, "V4", f"R{rid}: la cobertura acumulada BAJÓ "
                         f"({self._cobMax[rid]} → {cob})")
        self._cobMax[rid] = max(cob, self._cobMax.get(rid, 0))
        # V7 · atascado: misma meta y la distancia REAL a ella (pose del
        # broadcast, que se actualiza siempre) no mejora una celda en 2×12 s.
        # No se usa `sinprog` del agente: su reloj solo avanza cuando el agente
        # delibera, y mientras el cuerpo navega no delibera.
        m = self.mapas.get(rid)
        meta = d.get("meta", -1)
        if m and meta is not None and meta >= 0 and isinstance(d.get("x"), float):
            r, c = divmod(meta, m["cols"])
            gx = m["ox"] + (c + 0.5) * m["celda"]
            gy = m["oy"] + (r + 0.5) * m["celda"]
            dist = math.hypot(d["x"] - gx, d["y"] - gy)
            p = self._prog.get(rid)
            if p is None or p[0] != meta or dist < p[1] - m["celda"]:
                self._prog[rid] = (meta, dist, tlog)
            elif tlog - p[2] > 2 * CK.NO_PROGRESO_MS / 1000.0:
                self._alerta(tlog, rid, "V7", f"R{rid}: {tlog - p[2]:.0f} s sin acercarse a la "
                             f"celda {meta} (plan {d.get('plan')}, a {dist:.0f} mm)", cada_s=10)
        else:
            self._prog.pop(rid, None)
        # V3 se exige PERSISTENTE (3 THINK seguidos): THINK y BMAP se toman en
        # instantes distintos y en una reasignación uno puede ir por delante.
        m = self.mapas.get(rid)
        falla = CK.zona_propia(d, m) if (m and tlog - m["tlog"] < 2.5) else []
        if self._persiste(("V3", rid), bool(falla), 3):
            for c, t in falla:
                self._alerta(tlog, rid, c, t)
        for c, t in CK.dueno_objetivo(self._recientes(self.think, tlog)):
            self._alerta(tlog, -1, c, t)

    def _checks_mapa(self, m, tlog):
        rid = m["id"]
        for c, t in CK.particion(m) + CK.saturacion(m):
            self._alerta(tlog, rid, c, t)
        # El acuerdo solo se exige entre réplicas del MISMO instante y con el
        # mismo conjunto de vivos: una caída tarda staleEpochs en propagarse y
        # durante esa ventana el desacuerdo es esperado, no un defecto.
        recientes = {i: x for i, x in self._recientes(self.mapas, tlog, 1.2).items()}
        vivos = {i: self.think[i].get("vivos") for i in recientes if i in self.think}
        falla = CK.acuerdo_zonas(recientes) if len(set(vivos.values())) <= 1 else []
        if self._persiste(("V1",), bool(falla), 2 * max(1, len(recientes))):
            for c, t in falla:
                self._alerta(tlog, rid, c, t)

    def _persiste(self, clave, cond, n):
        self._racha[clave] = self._racha.get(clave, 0) + 1 if cond else 0
        return self._racha[clave] >= n

    def tick(self, tlog):
        """Lo que no se detecta al llegar un mensaje sino por su AUSENCIA.
        La GUI lo llama periódicamente; el verificador, al recorrer el log."""
        with self._lock:
            for rid, t in self.think.items():
                mudo = (self.sesion and t.get("fase") not in ("IDLE", "FIN")
                        and tlog - t["tlog"] > CK.SILENCIO_S)
                if mudo and rid not in self._silencio:
                    self._silencio.add(rid)
                    self._alerta(tlog, rid, "V10", f"R{rid} sin telemetría hace "
                                 f"{tlog - t['tlog']:.1f} s (¿caído, sin WiFi, reiniciado?)", cada_s=0)
                elif not mudo and rid in self._silencio:
                    self._silencio.discard(rid)
                    self._evento(tlog, rid, "vuelve", "telemetría de nuevo")

    @staticmethod
    def _recientes(dic, tlog, ventana=3.0):
        return {i: v for i, v in dic.items() if tlog - v["tlog"] <= ventana}

    # ── para la GUI ───────────────────────────────────────────────────────────
    def silenciosos(self, ahora):
        return [i for i, t in self.think.items()
                if self.sesion and ahora - t["tlog"] > CK.SILENCIO_S and t.get("fase") != "IDLE"]

    def snapshot(self):
        with self._lock:
            self._flush()
            return dict(think={k: dict(v) for k, v in self.think.items()},
                        mapas=dict(self.mapas), camara=dict(self.camara),
                        home=self.home, alertas=list(self.alertas),
                        eventos=list(self.eventos), sesion=self.sesion,
                        objetivo_real=self.objetivo_real)

    def ahora(self):
        return time.time() - self.t0
