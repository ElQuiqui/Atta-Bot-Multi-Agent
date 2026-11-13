"""
render_gui.py — la GUI con los paneles BDI-E, alimentada por el BANCO, sin robots

Corre una misión en el banco (el mismo C++ del robot, y el mismo codificador de
telemetría), le pasa los THINK/BMAP/POSES al BdieMonitor de la base real y
dibuja la ventana de la GUI fuera de pantalla. Guarda capturas PNG.

Sirve para dos cosas: probar los paneles sin laboratorio, y ver cómo se tiene
que ver una misión sana antes de ir a mirarla en vivo.

    python3 pruebas_host/render_gui.py [semilla] [segundos] [--caida R@s]
"""
import os, sys
AQUI = os.path.dirname(os.path.abspath(__file__))
V2 = os.path.dirname(AQUI)
sys.path[:0] = [os.path.join(V2, 'base'), os.path.join(V2, 'sitl')]
os.chdir(os.path.join(V2, 'base'))

import AttaBot_GUI as G                      # los dos fijan QT_QPA_PLATFORM=xcb
import AttaBot_Base as AB                    # al importarse…
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # …y acá se pisa: no hay pantalla
from PyQt5.QtWidgets import QApplication
os.chdir(os.path.join(V2, 'sitl'))
import bdie_sim as S

semilla = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 3
tope = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].replace('.', '').isdigit() else 70
kw = {}
if '--caida' in sys.argv:
    r, t = sys.argv[sys.argv.index('--caida') + 1].split('@')
    kw = dict(kill_id=int(r), kill_at_s=float(t))

base = AB.base
base.bdie = AB.BdieMonitor(escribir=False, log=lambda m: None)
base.robots = {}
reloj = [0.0]
base._tlog = lambda: reloj[0]


def cb(t, rid, txt):
    reloj[0] = t
    c = txt.split('|')[0]
    if c == 'SESSION':
        p = txt.split('|')
        base.bdie.new_session(float(p[3]), float(p[4]), t)
    elif c == 'OBJETIVO':
        p = txt.split('|')
        base.bdie.objetivo_real = (float(p[1]), float(p[2]))
    elif c == 'POSES':
        base.bdie.on_poses_text(txt, t)
    else:
        base.bdie.on_message(txt, t)
    base.bdie.tick(t)


S.run_once(semilla, dict(S.DEFAULTS), telemetria=cb, tope_s=tope, **kw)

app = QApplication(sys.argv)
gui = G.AttaBotGUI(base)
gui.resize(1500, 950)
gui._vistas.setCurrentIndex(1)
gui._paneles.setCurrentIndex(1)
gui._mapaBdie.refrescar()
gui._menteBdie.refrescar()
salida = os.path.join(AQUI, 'capturas')
os.makedirs(salida, exist_ok=True)
for capa in ['Repelente', 'Atractivo', 'Cobertura', 'Zonas']:
    gui._mapaBdie.cbCapa.setCurrentText(capa)
    gui._mapaBdie.refrescar()
    app.processEvents()
    gui.grab().save(os.path.join(salida, f'gui_s{semilla}_{int(tope)}s_{capa}.png'))
print('capturas en', salida)
