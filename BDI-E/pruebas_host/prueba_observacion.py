#!/usr/bin/env python3
"""
prueba_observacion.py — ¿la observación ve lo que tiene que ver?

Corre misiones en el banco (el mismo C++ del robot y el mismo codificador de
telemetría), las pasa por el BdieMonitor real de la base, que escribe los mismos
registros que en el laboratorio, y después por el verificador fuera de línea.

  O1  misiones nominales        el verificador debe decir PASA
  O2  caída inducida            V10 aparece, todos los vivos registran la caída
                                y la misión se completa igual
  O3  defectos inyectados       cada verificación debe dispararse con su
                                defecto: una verificación que nunca falla no
                                prueba nada
      O3a  una réplica con otra asignación de zonas         → V1
      O3b  una transición de fase imposible                 → V5
      O3c  la cobertura acumulada baja                      → V4
      O3d  un robot que deja de hablar sin estar apagado    → V10
      O3e  un robot que ubica el objetivo en otra celda     → V6
      O3f  un dueño del objetivo distinto                   → V6

    python3 pruebas_host/prueba_observacion.py            # todo
    python3 pruebas_host/prueba_observacion.py --rapido   # 3 semillas
"""
import os
import sys
import tempfile

AQUI = os.path.dirname(os.path.abspath(__file__))
V2 = os.path.dirname(AQUI)
sys.path[:0] = [os.path.join(V2, 'base'), os.path.join(V2, 'base', 'herramientas'),
                os.path.join(V2, 'sitl')]
os.chdir(os.path.join(V2, 'sitl'))
import bdie_sim as S                                  # noqa: E402
from bdie_monitor import BdieMonitor                  # noqa: E402
from verificar_bdie import verificar                  # noqa: E402

FALLAS = []


def check(nombre, cond, detalle=''):
    print(('  ✓ ' if cond else '  ✗ ') + nombre + ('' if cond else '  ← ' + detalle))
    if not cond:
        FALLAS.append(nombre)


def correr(seed, inyectar=None, **kw):
    """Una misión del banco → logs en una carpeta temporal → prefijo."""
    carpeta = tempfile.mkdtemp(prefix='bdie_obs_')
    mon = BdieMonitor(carpeta=carpeta, log=lambda m: None)

    def cb(t, rid, txt):
        if inyectar:
            txt = inyectar(t, rid, txt)
            if txt is None:
                return
        c = txt.split('|')[0]
        if c == 'SESSION':
            p = txt.split('|')
            mon.new_session(float(p[3]), float(p[4]), t)
        elif c == 'OBJETIVO':
            p = txt.split('|')
            mon.set_objetivo_real(float(p[1]), float(p[2]))
        elif c == 'POSES':
            mon.on_poses_text(txt, t)
        else:
            mon.on_message(txt, t)
        mon.tick(t)

    r = S.run_once(seed, dict(S.DEFAULTS), telemetria=cb, seguir_s=5.0, **kw)
    mon.cerrar()
    return mon.prefijo, r


def main():
    semillas = [1, 2, 3] if '--rapido' in sys.argv else list(range(1, 11))

    print('O1 · misiones nominales: el verificador tiene que decir PASA')
    for s in semillas:
        pref, r = correr(s)
        pasa, info = verificar(pref, salida=False)
        check(f'O1 semilla {s}: banco {"completa" if r["mision_completa"] else "INCOMPLETA"}, '
              f'verificador {"PASA" if pasa else "NO PASA"}',
              pasa == r['mision_completa'] and pasa,
              f"graves={info['graves']} · {pref}reporte.md")

    print('O2 · caída inducida a los 40 s')
    for s, k in [(semillas[0], 2), (semillas[-1], 0)]:
        pref, r = correr(s, kill_at_s=40, kill_id=k)
        pasa, info = verificar(pref, caidos=[k], salida=False)
        check(f'O2 semilla {s}, cae R{k}: V10 detecta el silencio',
              'V10' in info['alertas'], str(list(info['alertas'])))
        check(f'O2 semilla {s}, cae R{k}: verificador {"PASA" if pasa else "NO PASA"}',
              pasa, f"graves={info['graves']} · {pref}reporte.md")

    print('O3 · defectos inyectados: cada verificación se tiene que disparar')
    s = semillas[0]

    def zonas_cruzadas(t, rid, txt):              # el robot 1 cree otra asignación
        if rid == 1 and txt.startswith('BMAP') and t > 30:
            import re
            m = re.search(r'\|of=([^|]*)', txt)
            pares = [p.split(':') for p in m.group(1).split(',')]
            if len(pares) >= 2:
                pares[0][1], pares[1][1] = pares[1][1], pares[0][1]
                txt = txt.replace(m.group(0), '|of=' + ','.join(':'.join(p) for p in pares))
        return txt

    def fase_imposible(t, rid, txt):
        if rid == 2 and txt.startswith('THINK') and 30 < t < 31:
            txt = txt.replace('fase=BUSCAR', 'fase=VOLVER').replace('fase=DESPLEGAR', 'fase=VOLVER')
        return txt

    def cobertura_baja(t, rid, txt):
        if rid == 0 and txt.startswith('THINK') and 40 < t < 40.6:
            import re
            txt = re.sub(r'\|cob=\d+', '|cob=0', txt)
        return txt

    def mudo(t, rid, txt):
        return None if (rid == 3 and t > 30) else txt

    def otra_celda(t, rid, txt):
        if rid == 4 and txt.startswith('THINK') and '|obj=1|' in txt:
            import re
            txt = re.sub(r'\|cobj=(\d+)', lambda m: f'|cobj={int(m.group(1)) + 3}', txt)
        return txt

    def otro_dueno(t, rid, txt):
        if rid == 4 and txt.startswith('THINK') and '|obj=1|' in txt:
            import re
            txt = re.sub(r'\|dueno=(\d+)', '|dueno=7', txt)
        return txt

    for nombre, fn, cod in [('O3a zonas cruzadas', zonas_cruzadas, 'V1'),
                            ('O3b fase imposible', fase_imposible, 'V5'),
                            ('O3c cobertura que baja', cobertura_baja, 'V4'),
                            ('O3d robot mudo', mudo, 'V10'),
                            ('O3e objetivo en otra celda', otra_celda, 'V6'),
                            ('O3f otro dueño', otro_dueno, 'V6')]:
        pref, _ = correr(s, inyectar=fn)
        pasa, info = verificar(pref, salida=False)
        check(f'{nombre} → {cod}', cod in info['alertas'] and not pasa,
              f"alertas={list(info['alertas'])}, veredicto={'PASA' if pasa else 'NO PASA'}")

    print()
    if FALLAS:
        print(f'FALLAN {len(FALLAS)}:')
        for f in FALLAS:
            print('   ·', f)
        sys.exit(1)
    print('Todo en orden.')


if __name__ == '__main__':
    main()
