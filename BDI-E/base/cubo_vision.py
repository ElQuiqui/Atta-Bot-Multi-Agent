"""Detección del objetivo con la cámara cenital, solo como verdad de terreno.

Registra dónde está realmente el objetivo (objetivo_real en el registro de la
sesión y en el mapa del panel), sin tener que medirlo a mano. Con eso el
análisis puede medir el error de ubicación que reporta el robot y cada pasada
cerca del objetivo que no terminó en detección.

No le dice a los robots dónde está el objetivo: la detección es local a cada
robot (sensor de color + MatchColor) y el hallazgo se difunde entre ellos. Si
la base se lo comunicara, la misión dejaría de ser descentralizada.

Método: máscara HSV del color de la sesión → componentes conexas → filtro por
área y forma cuadrada (tamaño físico del cubo) → se descartan las manchas
pegadas a un ArUco (carcasas de los robots) y las que caen fuera del área útil
→ el centro pasa a mm con el mismo modelo de cámara que usan las poses de los
robots: el detector aprende, de cada cuadro, pares (centro del ArUco en px →
pose en mm que publicó la base). Así el objetivo queda en el mismo marco que las
poses del registro, que es lo que hace falta para decir «el robot pasó a X mm
del objetivo». Mientras no haya pares suficientes, usa la escala lineal
(mmPixel del ArUco de referencia). Como el objetivo no se mueve, la posición se
confirma con N cuadros seguidos y después se conserva aunque un robot lo tape.
"""
import math
import cv2
import numpy as np

# Rangos HSV de OpenCV (H 0–179). El cubo rojo en la arena da H≈174, S≈253,
# V≈154; las carcasas de los robots y el piso tienen S < 120.
RANGOS = {
    'rojo':  [((0, 120, 70), (10, 255, 255)), ((168, 120, 70), (179, 255, 255))],
    'verde': [((40, 100, 60), (85, 255, 255))],
    'azul':  [((95, 120, 50), (130, 255, 255))],
}


class MapaPxMm:
    """px → mm aprendido de los ArUco de los robots (interpolación por vecinos).
    Rejilla de 20 px: una muestra por celda (mediana móvil corta), así el mapa no
    crece y una pose mala aislada no lo arrastra."""
    def __init__(self, celda_px=20, k=6, radio_px=120):
        self.celda = celda_px
        self.k = k
        self.radio = radio_px
        self.m = {}                      # (cx, cy) -> lista de (u, v, x, y)
        self.H = None
        self._nuevos = 0
        self.error_mm = None

    def aprender(self, u, v, x, y):
        c = (int(u // self.celda), int(v // self.celda))
        l = self.m.setdefault(c, [])
        l.append((u, v, x, y))
        self._nuevos += 1
        if len(l) > 9:
            del l[0]

    def __len__(self):
        return len(self.m)

    def _ajustar(self):
        """Homografía global px→mm con RANSAC sobre la mediana de cada celda. Un
                ajuste afín local con pocos vecinos extrapola mal cuando los robots
                todavía no pasaron cerca del objetivo; la homografía usa toda la
                arena."""
        self._nuevos = 0
        if len(self.m) < 40:
            self.H = None
            return
        src, dst = [], []
        for l in self.m.values():
            a = np.median(np.array(l), axis=0)
            src.append(a[:2]); dst.append(a[2:])
        src = np.array(src, np.float32); dst = np.array(dst, np.float32)
        # que cubra área: si todos los pares están en un rincón, no sirve aún
        if (np.ptp(src[:, 0]) < 400) or (np.ptp(src[:, 1]) < 250):
            self.H = None
            return
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 25.0)
        if H is None or mask.sum() < 10:
            self.H = None
            return
        pr = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
        e = np.linalg.norm(pr - dst, axis=1)[mask.ravel() == 1]
        self.error_mm = float(np.median(e))
        self.H = H

    def a_mm(self, u, v):
        if self.H is None or self._nuevos > 200:
            self._ajustar()
        if self.H is None:
            return None
        p = cv2.perspectiveTransform(np.array([[[u, v]]], np.float32), self.H).reshape(2)
        return float(p[0]), float(p[1])


class CuboDetector:
    def __init__(self, lado_mm=55.0, confirmar=5, tol_mm=40.0,
                 radio_robot_mm=110.0):
        self.lado_mm = lado_mm          # lado aproximado del cubo
        self.confirmar = confirmar      # cuadros seguidos para darlo por visto
        self.tol_mm = tol_mm
        self.radio_robot_mm = radio_robot_mm
        self.color = 'rojo'
        self.posicion = None            # (x_mm, y_mm) confirmada
        self._cand = None
        self._n = 0
        self.ultimo_px = None           # para dibujar
        self.mapa = MapaPxMm()
        self.px_fijo = None
        self.zona_px = None             # opcional: función (u, v) -> bool

    def reiniciar(self, color=None):
        if color:
            self.color = color
        self.posicion = None
        self.px_fijo = None
        self._cand = None
        self._n = 0

    def _a_mm(self, u, v, ref_px, mm_px):
        r = self.mapa.a_mm(u, v)
        if r is not None:
            return r
        if ref_px is None:
            return None
        return (u - ref_px[0]) * mm_px, (v - ref_px[1]) * mm_px

    def aprender(self, u, v, x, y):
        """Un robot visto en (u, v) px con pose (x, y) mm publicada por la base."""
        self.mapa.aprender(u, v, x, y)

    def procesar(self, frame, ref_px, mm_px, marcadores_px, arena_mm):
        """frame: BGR ya corregido (el mismo que ve detectMarkers).
                ref_px: centro del ArUco de referencia o None. marcadores_px: centros de
                los ArUco del cuadro (para descartar carcasas).

                El objetivo se detecta una vez al principio y queda fijo. Primero se
                confirma su posición en píxeles, que no depende del mapa px→mm y sirve
                desde el primer segundo; después ya no se vuelve a buscar. Devuelve
                (x, y) en mm, o None mientras el mapa no esté listo."""
        if self.posicion is not None and self.mapa._nuevos < 200:
            return self.posicion
        if self.px_fijo is None:
            px = self._buscar_px(frame, mm_px, marcadores_px, ref_px, arena_mm)
            if px is None:
                return None
            if self._cand and math.hypot(px[0] - self._cand[0], px[1] - self._cand[1]) < 6:
                self._n += 1
            else:
                self._cand, self._n = px, 1
            self.ultimo_px = px
            if self._n < self.confirmar:
                return None
            self.px_fijo = self._cand
        # El píxel queda fijo; los mm se recalculan cada vez que el mapa aprendió
                # 200 muestras nuevas, porque la primera conversión, con pocos robots
                # vistos, puede tener un error de cientos de milímetros.
        r = self.mapa.a_mm(*self.px_fijo)
        if r is None:
            return self.posicion
        if not (-50 <= r[0] <= arena_mm[0] + 50 and -50 <= r[1] <= arena_mm[1] + 50):
            self.px_fijo = None              # era otra cosa: se vuelve a buscar
            self._cand, self._n = None, 0
            return None
        self.posicion = (round(r[0]), round(r[1]))
        return self.posicion

    def _buscar_px(self, frame, mm_px, marcadores_px, ref_px, arena_mm):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m = None
        for lo, hi in RANGOS.get(self.color, RANGOS['rojo']):
            mi = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            m = mi if m is None else cv2.bitwise_or(m, mi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, _, st, cen = cv2.connectedComponentsWithStats(m)
        lado_px = self.lado_mm / mm_px
        a_min, a_max = (0.4 * lado_px) ** 2, (1.8 * lado_px) ** 2
        lejos_px = self.radio_robot_mm / mm_px
        mejor = None
        for i in range(1, n):
            x, y, w, h, a = st[i]
            if not (a_min <= a <= a_max):
                continue
            if max(w, h) > 1.8 * min(w, h) or a < 0.6 * w * h:
                continue                                    # no es un cuadrado lleno
            u, v = float(cen[i][0]), float(cen[i][1])
            if any(math.hypot(u - mu, v - mv) < lejos_px for mu, mv in marcadores_px):
                continue                                    # carcasa de un robot
            if ref_px is not None:
                # filtro grueso con la escala lineal (±15 %): saca a los robots
                # estacionados a la derecha (x ≈ 2260 mm) y lo de fuera de la arena
                xl, yl = (u - ref_px[0]) * mm_px, (v - ref_px[1]) * mm_px
                if not (-200 <= xl <= arena_mm[0] * 1.1 and -200 <= yl <= arena_mm[1] * 1.1):
                    continue
            if self.zona_px is not None and not self.zona_px(u, v):
                continue
            err = abs(a - lado_px * lado_px)            # el más parecido al tamaño del cubo
            if mejor is None or err < mejor[0]:
                mejor = (err, (u, v))
        return None if mejor is None else mejor[1]
