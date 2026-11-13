"""sigma_aruco.py — σ de la pose ArUco a partir de un Position_Log de la base.

Uso:
    python sigma_aruco.py ../PositionLogs/Position_Log_XX.csv [--min-s 20] [--tol 25]

Busca solo los tramos en que cada robot estuvo QUIETO (la pose no se aleja más
de --tol mm de la mediana del tramo durante al menos --min-s segundos) y en
cada tramo calcula:
    σx, σy      desviación estándar de x e y (mm)
    σr          sqrt(σx² + σy²): el radio de ruido que entra al simulador
    p95         percentil 95 de la distancia a la media (mm)
    σθ          desviación estándar CIRCULAR del ángulo (°)
El valor que manda es el PEOR tramo (el ruido crece lejos del centro óptico).

Protocolo (guía de puesta en marcha §3 · prueba 3): robot quieto 60 s en el
centro y en las cuatro esquinas, base grabando. No hace falta anotar tiempos:
el script encuentra los tramos solo.
"""
import csv, math, sys, argparse, statistics as st
from collections import defaultdict

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--min-s", type=float, default=20.0, help="duración mínima de un tramo quieto (s)")
ap.add_argument("--tol", type=float, default=25.0, help="radio máx. alrededor de la mediana (mm)")
ap.add_argument("--recortar", type=float, default=2.0, help="s que se descartan a cada extremo del tramo")
a = ap.parse_args()

por_robot = defaultdict(list)
with open(a.log, newline="") as f:
    for row in csv.DictReader(f):
        try:
            por_robot[row["robot"]].append((float(row["time"]), float(row["x"]),
                                            float(row["y"]), float(row["angle"])))
        except (ValueError, KeyError):
            pass

def tramos_quietos(m):
    """Ventanas maximales en que todos los puntos caen a ≤ tol de su mediana."""
    out, i, n = [], 0, len(m)
    while i < n:
        j = i + 1
        while j < n:
            xs = [p[1] for p in m[i:j+1]]; ys = [p[2] for p in m[i:j+1]]
            mx, my = st.median(xs), st.median(ys)
            if max(math.hypot(p[1]-mx, p[2]-my) for p in m[i:j+1]) > a.tol:
                break
            j += 1
        if m[j-1][0] - m[i][0] >= a.min_s:
            out.append(m[i:j]); i = j
        else:
            i += 1
    return out

def sigma_circ(grados):
    s = sum(math.sin(math.radians(g)) for g in grados) / len(grados)
    c = sum(math.cos(math.radians(g)) for g in grados) / len(grados)
    R = min(1.0, math.hypot(s, c))
    return math.degrees(math.sqrt(-2 * math.log(R))) if R > 0 else float("inf")

peor = None
print(f"{'robot':8} {'t0–t1 (s)':>14} {'n':>5} {'x̄':>7} {'ȳ':>7} {'σx':>5} {'σy':>5} {'σr':>5} {'p95':>5} {'σθ°':>5}")
for rob, m in por_robot.items():
    m.sort()
    for t in tramos_quietos(m):
        t0, t1 = t[0][0] + a.recortar, t[-1][0] - a.recortar
        t = [p for p in t if t0 <= p[0] <= t1]
        if len(t) < 10:
            continue
        xs, ys = [p[1] for p in t], [p[2] for p in t]
        sx, sy = st.pstdev(xs), st.pstdev(ys)
        mx, my = st.mean(xs), st.mean(ys)
        d = sorted(math.hypot(x-mx, y-my) for x, y in zip(xs, ys))
        p95 = d[int(0.95 * (len(d)-1))]
        sr = math.hypot(sx, sy)
        sth = sigma_circ([p[3] for p in t])
        print(f"{rob:8} {t0:6.1f}–{t1:6.1f} {len(t):5d} {mx:7.0f} {my:7.0f} {sx:5.1f} {sy:5.1f} {sr:5.1f} {p95:5.1f} {sth:5.1f}")
        if peor is None or sr > peor[0]:
            peor = (sr, sth, rob, mx, my)

if peor is None:
    sys.exit("Ningún tramo quieto: bajar --min-s o subir --tol.")
sr, sth, rob, mx, my = peor
print(f"\nPEOR CASO: σr = {sr:.1f} mm, σθ = {sth:.1f}°  ({rob} en {mx:.0f}, {my:.0f})")
print(f"  → ARUCO_POS_SIGMA ≈ {sr/math.sqrt(2):.1f} mm por eje, ARUCO_ANGLE_SIGMA ≈ {sth:.1f}°")
umbral = 0.32 * 190
print(f"  → con celda de 190 mm el umbral de llegada es {umbral:.0f} mm = {umbral/max(sr,1e-9):.1f} σr")
cel_min = max(150.0, 3 * sr / 0.32)
print(f"  → celda mínima que deja el umbral a ≥ 3 σr (y ≥ diámetro del robot): {cel_min:.0f} mm")
