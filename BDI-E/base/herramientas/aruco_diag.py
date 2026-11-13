#!/usr/bin/env python3
"""
aruco_diag.py — ¿cuánto titila cada marker, y dónde? Diagnóstico con la cámara real

Abre la cámara con la MISMA configuración que la base (configSystem.json:
índice, resolución, exposición, foco, calibración) y el MISMO detector, sin red
y sin robots registrados. Durante N segundos cuenta, por marker:
  · % de cuadros detectado por la pasada general
  · % con la recuperación por ROI de la base V2
  · la racha más larga sin verlo (lo que de verdad deja a un robot sin pose)
  · el lado del marker en píxeles (menos de ~30 px ya es frágil)
Guarda en herramientas/aruco_diag/ el cuadro en que más markers faltaban, con
los rechazados del detector dibujados: ahí se ve si es reflejo, desenfoque o
tamaño.

    cd base
    python3 herramientas/aruco_diag.py            # 30 s, los markers que aparezcan
    python3 herramientas/aruco_diag.py 60 2 3     # 60 s, vigilando los markers 2 y 3

⚠ Cerrá la GUI/base antes: la cámara solo la puede abrir un proceso.
"""
import json
import os
import sys
import time

import cv2
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(AQUI)
sys.path.insert(0, BASE)
os.chdir(BASE)
import AttaBot_Base as AB   # noqa: E402

args = [a for a in sys.argv[1:]]
dur = float(args[0]) if args else 30.0
vigilar = args[1:]

cfg = json.load(open('configSystem.json'))
b = AB.base
b.configVisionSystem(cfg['vision_system'])
b.robots = {m: None for m in vigilar}        # los que la ROI tiene que perseguir

stats = {}          # id → dict(gen, roi, lado[], racha, peor)
peor = (-1, None, None)
n = 0
t_fin = time.time() + dur
print(f'Midiendo {dur:.0f} s… movés los robots por toda la arena si podés.')
while time.time() < t_fin:
    ok, fr = b.camera.read()
    if not ok:
        continue
    fr = b.cameraCorrection(fr)
    gray = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
    c, ids, rech = b.arucoDetector.detectMarkers(gray)
    gen = set() if ids is None else {str(i) for i in ids.flatten()}
    c2, ids2 = b._recuperarPorROI(gray, c, ids)
    con = set() if ids2 is None else {str(i) for i in ids2.flatten()}
    if not vigilar:                           # sin lista: se vigila lo que aparezca
        for m in gen:
            b.robots.setdefault(m, None)
    n += 1
    for m in set(b.robots) | gen:
        s = stats.setdefault(m, dict(gen=0, roi=0, total=0, lado=[], racha=0, peor=0))
        s['total'] += 1
        s['gen'] += m in gen
        s['roi'] += m in con
        s['racha'] = 0 if m in con else s['racha'] + 1
        s['peor'] = max(s['peor'], s['racha'])
    if ids2 is not None:
        for k, m in enumerate(ids2.flatten()):
            q = c2[k][0]
            stats[str(m)]['lado'].append(float(np.mean([np.linalg.norm(q[j] - q[(j + 1) % 4]) for j in range(4)])))
    faltan = len(set(b.robots) - con)
    if faltan > peor[0]:
        img = fr.copy()
        if c2 is not None and len(c2):
            cv2.aruco.drawDetectedMarkers(img, c2, ids2)
        if rech is not None and len(rech):
            cv2.aruco.drawDetectedMarkers(img, rech, borderColor=(0, 0, 255))
        peor = (faltan, img, time.strftime('%H-%M-%S'))

fps = n / dur
print(f'\n{n} cuadros ({fps:.1f} FPS)\n')
print(f"{'marker':>7} {'general':>9} {'con ROI':>9} {'racha máx. sin verlo':>22} {'lado px':>9}")
for m in sorted(stats, key=lambda x: int(x)):
    s = stats[m]
    lado = f"{np.median(s['lado']):.0f}" if s['lado'] else '—'
    print(f"{m:>7} {100 * s['gen'] / s['total']:>8.0f}% {100 * s['roi'] / s['total']:>8.0f}% "
          f"{s['peor']:>10} cuadros ({s['peor'] / max(fps, 1):.1f} s) {lado:>9}")
if peor[1] is not None:
    os.makedirs(os.path.join(AQUI, 'aruco_diag'), exist_ok=True)
    ruta = os.path.join(AQUI, 'aruco_diag', f'peor_{peor[2]}.png')
    cv2.imwrite(ruta, peor[1])
    print(f'\nCuadro con más faltantes ({peor[0]}): {ruta}')
    print('  verde = detectado · rojo = candidato RECHAZADO (el marker estaba, no se pudo leer)')
print('\nCómo leerlo: racha > ~1,5 s deja al robot sin pose (el firmware cae al EKF);'
      ' lado < 30 px → marker más grande o cámara más baja;'
      ' rojos sobre el marker → reflejo o desenfoque, no tamaño.')
b.camera.release()
