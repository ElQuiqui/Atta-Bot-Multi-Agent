"""
prueba_base_bdie.py — la base parcheada, probada sin cámara, sin red y sin robots
=================================================================================

Qué verifica (B0–B4 de AttaBot_Base.py):

  P1  el broadcast lleva a TODOS los robots visibles de la prueba, en un paquete
  P2  excluye el ArUco de referencia y los marcadores que no son robots
  P3  un robot QUIETO sigue apareciendo (no pasa por el filtro de 4 mm / 4°)
  P4  el epoch avanza en cada paquete
  P5  SESSION.START exige la posición de la base; sin ella no envía nada
  P6  SESSION.START reinicia el epoch y se repite 3 veces; ABORT también
  P7  readUdpConnection entrega HB y TGT al observador
  P8  (opcional) cada paquete lo decodifica el mismo mission.h del robot
  P11 el reparto acordado (ZA) llega al observador; el reclamo (ZC) se calla
  P9  V2: BASE.SESSION|… por el despachador único del laboratorio, y el esquema
  P10 V2: THINK y BMAP llegan al monitor BDI-E y se decodifican

Uso, desde Codigo/BDI-E V2/base:
    python3 prueba_base_bdie.py            # P1–P7
    python3 prueba_base_bdie.py --firmware # + P8, compila mission.h con g++
"""
import os
import subprocess
import sys
import tempfile
import threading
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
os.chdir(AQUI)

import AttaBot_Base as AB                      # noqa: E402  (crea AB.base)
from swarm_broadcast import SwarmBroadcaster   # noqa: E402


class SocketFalso:
    """Captura lo que se envía; entrega una lista de paquetes al recibir."""
    def __init__(self, entrantes=()):
        self.enviados = []
        self._entrantes = list(entrantes)

    def sendto(self, data, dest):
        self.enviados.append((data.decode(), dest))

    def settimeout(self, _):
        pass

    def recvfrom(self, _):
        if not self._entrantes:
            raise SystemExit      # corta el while True del hilo
        return self._entrantes.pop(0)


class RobotFalso:
    def __init__(self, rid):
        self.id, self.name, self.IP = rid, 'Atta_' + rid, '10.0.0.' + rid


FALLAS = []


def check(nombre, cond, detalle=''):
    print(('  ✓ ' if cond else '  ✗ ') + nombre + ('' if cond else '  ← ' + detalle))
    if not cond:
        FALLAS.append(nombre)


def nueva_base():
    b = AB.base
    b.sock = SocketFalso()
    b.broadcastIP, b.port, b.baseIP = '10.0.0.255', 6060, '10.0.0.100'
    b.swarm = SwarmBroadcaster(b.sock, b.broadcastIP, b.port, log=lambda m: None)
    b.log = lambda m: None
    b.referenceMarkerId = '0'
    b.robots = {r: RobotFalso(r) for r in ('1', '2', '3')}
    b.bdie = AB.BdieMonitor(escribir=False, log=lambda m: None)   # sin archivos
    return b


def poses_enviadas(b):
    return [p for p, _ in b.sock.enviados if p.startswith('POSES|')]


def main():
    todos = []
    b = nueva_base()

    print('Broadcast de poses')
    b.currentArucoDetections = {
        '0': (0.0, 0.0, 12.0),              # referencia
        '1': (100.4, 200.6, 90.0),
        '2': (1300.0, 900.0, 180.0),
        '7': (5.0, 5.0, 5.0),               # marcador suelto, no es robot
    }                                        # el '3' no está a la vista
    b._broadcastSwarmPoses()
    p = poses_enviadas(b)
    check('P1 un solo paquete por cuadro', len(p) == 1, str(p))
    check('P1 contiene a los robots visibles',
          p and p[0] == 'POSES|1|1:100:201:90;2:1300:900:180', p[0] if p else '')
    check('P2 sin referencia ni marcador suelto',
          p and ';0:' not in p[0] and '|0:' not in p[0] and '7:' not in p[0])
    check('P1 va a la dirección de broadcast',
          b.sock.enviados[-1][1] == ('10.0.0.255', 6060), str(b.sock.enviados[-1][1]))

    time.sleep(0.06)                          # respeta min_interval (20 Hz)
    b._broadcastSwarmPoses()                  # misma pose: robots QUIETOS
    p = poses_enviadas(b)
    check('P3 robot quieto sigue en el broadcast', len(p) == 2 and '1:100:201:90' in p[1],
          str(p))
    check('P4 el epoch avanza', len(p) == 2 and p[1].startswith('POSES|2|'), str(p))
    todos += p

    print('Sesión')
    n0 = len(b.sock.enviados)
    b.handleSession('START')
    check('P5 START sin posición no envía nada', len(b.sock.enviados) == n0)
    b.handleSession('START 1250 abc')
    check('P5 START con basura no envía nada', len(b.sock.enviados) == n0)

    b.handleSession('start 1250 1000')
    s = [m for m, _ in b.sock.enviados[n0:]]
    check('P6 START repetido 3 veces', s == ['SESSION|START|1|1250|1000'] * 3, str(s))
    time.sleep(0.06)
    b._broadcastSwarmPoses()
    check('P6 el epoch se reinicia al abrir sesión',
          poses_enviadas(b)[-1].startswith('POSES|1|'), poses_enviadas(b)[-1])
    todos += s[:1] + poses_enviadas(b)[-1:]

    n1 = len(b.sock.enviados)
    b.handleSession('ABORT')
    s = [m for m, _ in b.sock.enviados[n1:]]
    check('P6 ABORT repetido 3 veces', s == ['SESSION|ABORT'] * 3, str(s))
    todos += s[:1]

    print('Gramática del laboratorio (BASE.VERBO|args) — V2')
    n2 = len(b.sock.enviados)
    b.dispatch('BASE', 'SESSION|START|1250|1000', log=lambda m: None)
    s = [m for m, _ in b.sock.enviados[n2:]]
    check('P9 BASE.SESSION|START|x|y llega por dispatch (consola y GUI)',
          len(s) == 3 and all(m.startswith('SESSION|START|') and m.endswith('|1250|1000')
                              for m in s), str(s))
    import comandos_schema as CS
    check('P9 el esquema acepta BASE.SESSION|START|x|y',
          CS.validar('BASE.SESSION|START|1250|1000')[0], str(CS.validar('BASE.SESSION|START|1250|1000')))
    check('P9 el esquema rechaza BASE.SESSION|START sin posición',
          not CS.validar('BASE.SESSION|START')[0])
    check('P9 TARGET_COLOR está en el vocabulario del robot',
          CS.validar('1.TARGET_COLOR|azul')[0] and 'TARGET_COLOR' in AB._ROBOT_CMDS)
    check('P9 vocabulario y esquema consistentes', CS.revisar_vocabulario() == [],
          str(CS.revisar_vocabulario()))

    print('Observador (readUdpConnection)')
    b.startTime = time.time()
    b.createConcoleLog = lambda: None
    b.addConcoleLog = lambda *a: None
    b.robots['1'].IP, b.robots['2'].IP = '10.0.0.1', '10.0.0.2'
    b.observer = AB.MissionObserver(log=lambda m: None)
    b.sock._entrantes = [
        (b'HB|1|2|0|10', ('10.0.0.1', 6060)),      # 2 = BUSCAR
        (b'HB|2|2|0|11', ('10.0.0.2', 6060)),
        (b'TGT|40|2|1300|900', ('10.0.0.2', 6060)),
        (b'HB|1|3|0|12', ('10.0.0.1', 6060)),      # 3 = CONGREGAR
        (b'POSES|9|1:0:0:0', ('10.0.0.100', 6060)),  # eco propio: se ignora
        (b'THINK|id=1|t=5|fase=BUSCAR|deseo=EXPLORAR|plan=P_EXPLORAR_ACO|meta=3|celda=2|'
         b'zona=0|enzona=1|x=100|y=200|obj=0|dueno=-1|cobj=-1|cerr=0|lleg=0|dir=0|'
         b'vivos=3|delib=9|cob=4|ev=ARRIVED|tev=4|reint=0|sinprog=0', ('10.0.0.1', 6060)),
        (b'BMAP|id=1|t=5|g=2,2,150.0,0.00,0.00|kz=2|z=0.1.0.0;0.1.1.1|of=1:0,2:1|'
         b'R=28000000|A=00000000|V=01|B=00', ('10.0.0.1', 6060)),
        # reparto por mensajes (14-oct-2026): el ZC se calla, el ZA se registra
        (b'ZC|3:6|1|0|2', ('10.0.0.1', 6060)),
        (b'ZA|3:6|1|0|2', ('10.0.0.1', 6060)),
        (b'ZA|3:6|1|0|2', ('10.0.0.1', 6060)),      # repetido cada 2 s: una sola vez
    ]
    _logs = []
    b.log = lambda m, *a, **k: _logs.append(str(m))
    hilo = b.readUdpConnection()
    hilo.join(2)
    o = b.observer
    check('P11 ZA (reparto acordado) llega al observador, una sola vez por ronda',
          o.zones.get('1') == (3, 6, 0) and sum(1 for e in o.events if e[2].startswith('ZA')) == 1,
          str(o.zones))
    check('P11 ZC no ensucia la consola', not any('ZC|' in m for m in _logs), str(_logs[-3:]))
    check('P7 fases por robot', o.phases == {'1': 'CONGREGAR', '2': 'BUSCAR'}, str(o.phases))
    check('P7 hallazgo registrado', o.t_found is not None)
    check('P7 no marca congregación con un robot aún buscando', o.t_congregated is None)
    snap = b.bdie.snapshot()
    check('P10 THINK llega al monitor BDI-E', snap['think'].get(1, {}).get('plan') == 'P_EXPLORAR_ACO',
          str(snap['think']))
    check('P10 BMAP se decodifica (feromona, cobertura, zonas)',
          snap['mapas'].get(1, {}).get('R') == [40, 0, 0, 0] and
          snap['mapas'][1]['V'] == [1, 0, 0, 0] and snap['mapas'][1]['de'] == {1: 0, 2: 1},
          str(snap['mapas'].get(1)))

    if '--firmware' in sys.argv:
        print('Decodificación con el mission.h del robot')
        decodificar_con_firmware(todos)

    print()
    if FALLAS:
        print('FALLAN %d: %s' % (len(FALLAS), ', '.join(FALLAS)))
        sys.exit(1)
    print('Todo en orden.')


PRUEBA_CPP = r'''
#include <cstdio>
#include <cstring>
#include "mission.h"
int main() {
    char line[512];
    while (std::fgets(line, sizeof line, stdin)) {
        line[std::strcspn(line, "\r\n")] = 0;
        bdie::Decoded d;
        if (!bdie::Decode(line, d)) { std::printf("FAIL\n"); continue; }
        std::printf("%d %lu %.0f %.0f %d", (int)d.kind, (unsigned long)d.epoch,
                    d.x, d.y, d.poseCount);
        for (int i = 0; i < d.poseCount; ++i)
            std::printf(" %d:%.0f:%.0f:%.0f", d.pose[i].id, d.pose[i].x,
                        d.pose[i].y, d.pose[i].th);
        std::printf("\n");
    }
}
'''


def decodificar_con_firmware(paquetes):
    firm = os.path.normpath(os.path.join(AQUI, '..', 'robot', 'AttaBot'))
    tmp = tempfile.mkdtemp()
    src, exe = os.path.join(tmp, 'p.cpp'), os.path.join(tmp, 'p')
    open(src, 'w').write(PRUEBA_CPP)
    r = subprocess.run(['g++', '-std=c++11', '-I', firm, src, '-o', exe],
                       capture_output=True, text=True)
    if r.returncode:
        check('P8 compila mission.h', False, r.stderr[:300])
        return
    out = subprocess.run([exe], input='\n'.join(paquetes) + '\n',
                         capture_output=True, text=True).stdout.split('\n')
    esperado = {
        'POSES|1|1:100:201:90;2:1300:900:180': '1 1 0 0 2 1:100:201:90 2:1300:900:180',
        'POSES|2|1:100:201:90;2:1300:900:180': '1 2 0 0 2 1:100:201:90 2:1300:900:180',
        'SESSION|START|1|1250|1000': '2 1 1250 1000 0',
        'SESSION|ABORT': '3 0 0 0 0',
    }
    for pk, got in zip(paquetes, out):
        exp = esperado.get(pk)
        if exp is None and pk.startswith('POSES|1|'):
            exp = esperado['POSES|1|1:100:201:90;2:1300:900:180']
        check('P8 %s' % pk, got == exp, 'firmware leyó «%s», se esperaba «%s»' % (got, exp))


if __name__ == '__main__':
    main()
