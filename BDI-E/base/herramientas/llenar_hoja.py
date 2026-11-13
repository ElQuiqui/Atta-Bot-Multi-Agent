#!/usr/bin/env python3
"""Llena la hoja de corrida de la campaña a partir de los logs de la base (07-nov-2026).

El operador solo corre las misiones EN EL ORDEN DE LA LISTA. Este script:
  · asigna a cada fila nominal pendiente, en orden, el siguiente `BDIE_…_think.csv`
    creado DESPUÉS de congelar el firmware (las corridas de verificación no entran);
  · descarta las sesiones de menos de 60 s sin detección (reinicios) y las lista;
  · llena hora de inicio, posición del cubo según la cámara (si nadie la midió),
    desenlace (el mismo de analisis_corrida.py), acercamientos < 1 diámetro entre
    robots, tiempo de montaje y robots que participaron;
  · detecta INTERVENCIONES por software (cualquier comando manual a un robot durante
    la misión) y las anota; marca para REVISAR lo que podría ser una intervención
    física (un robot que «salta» > 300 mm en < 1 s) o una corrida inválida
    (la cámara sin poses de ningún robot > 5 s);
  · en la línea base, asigna en orden los episodios `SEARCH_OBJECT` de las consolas.

NO decide la validez: `valida_s_n` lo pone el operador (o queda «s» por omisión en
el resumen) según las causas fijadas antes de la campaña. NO pisa celdas que ya
tienen algo. Lo automático va en `notas` con el prefijo [auto].

Uso (desde `Codigo/BDI-E V2/base`, con el Excel de la hoja CERRADO):
    python herramientas/llenar_hoja.py              # llena y muestra qué hizo
    python herramientas/llenar_hoja.py --resumen    # además corre resumen_campana.py
    python herramientas/llenar_hoja.py --seco       # muestra sin escribir
    python herramientas/llenar_hoja.py --desde "07-11 14:10"   # sin congelado, o para forzar
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import re
import subprocess
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(AQUI)
RAIZ = os.path.dirname(BASE)
sys.path.insert(0, AQUI)
import analisis_corrida as ac  # noqa: E402

DUR_MIN_S = 60.0
SALTO_MM, SALTO_S = 300.0, 1.0
HUECO_CAMARA_S = 5.0
DIAM = 150.0
RUTINA = {'GET_STATUS', 'TARGET_COLOR', 'BDIE_TELEM', 'CONFIG', 'POSES', 'TIME_SYNC',
          'NEIGHBOR_POSITIONS', 'SESSION', 'BASE.SESSION'}
RX_THINK = re.compile(r'BDIE_(\d\d)-(\d\d)_(\d\d)-(\d\d)-(\d\d)_think\.csv$')
RX_CONS = re.compile(r'Console_Log_(\d\d)-(\d\d)_(\d\d)-(\d\d)_Robots_\d+\.csv$')


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def stamp(nombre, rx, anio):
    m = rx.search(os.path.basename(nombre))
    if not m:
        return None
    g = [int(v) for v in m.groups()]
    return dt.datetime(anio, g[1], g[0], g[2], g[3], g[4] if len(g) > 4 else 0)


def desde_congelado():
    cands = sorted(glob.glob(os.path.join(RAIZ, '_congelado_*', 'FIRMWARE_CONGELADO.txt')), key=os.path.getmtime)
    if not cands:
        return None, None
    linea = open(cands[-1], encoding='utf-8').readline()
    m = re.search(r'(\d{4}-\d\d-\d\d \d\d:\d\d)', linea)
    return (dt.datetime.strptime(m.group(1), '%Y-%m-%d %H:%M') if m else None), os.path.dirname(cands[-1])


def leer_hoja(ruta):
    with open(ruta, encoding='utf-8-sig', errors='replace') as fh:
        cab = fh.readline()
        fh.seek(0)
        sep = ';' if cab.count(';') > cab.count(',') else ','
        r = csv.DictReader(fh, delimiter=sep)
        return list(r), r.fieldnames, sep


def info_sesion(think):
    filas = list(csv.DictReader(open(think, encoding='utf-8', errors='replace')))
    t0 = next((float(r['tlog']) for r in filas if r['fase'] == 'DESPLEGAR'), None)
    if t0 is None:
        return None
    tlast = max(float(r['tlog']) for r in filas)
    det = any(r['obj'] == '1' for r in filas)
    ids = sorted({r['id'] for r in filas if float(r['tlog']) >= t0}, key=int)
    fins = [float(r['tlog']) for r in filas if r['fase'] == 'FIN']
    t_fin = max(fins) if fins else None
    return dict(t0=t0, tlast=tlast, dur=tlast - t0, det=det, ids=ids,
                t_end=min(t_fin or tlast, t0 + 1200.0))


def objeto_camara(think):
    mapas = think.replace('_think.csv', '_mapas.jsonl')
    obj = None
    if os.path.exists(mapas):
        for linea in open(mapas, encoding='utf-8', errors='replace'):
            if '"objetivo_real"' in linea:
                try:
                    o = json.loads(linea).get('objetivo_real')
                    if o:
                        obj = o
                except ValueError:
                    pass
    return obj


def revisar_poses(think, t0, t_end):
    poses = think.replace('_think.csv', '_poses.csv')
    saltos, acerc, hueco_max = [], 0, 0.0
    if not os.path.exists(poses):
        return saltos, None, None
    P, todas = {}, []
    for r in csv.DictReader(open(poses, encoding='utf-8', errors='replace')):
        t = fnum(r.get('tlog'))
        if t is None or t < t0 or t > t_end:
            continue
        P.setdefault(r['id'], []).append((t, float(r['x']), float(r['y'])))
        todas.append(t)
    todas.sort()
    for a, b in zip(todas, todas[1:]):
        hueco_max = max(hueco_max, b - a)
    for i, v in P.items():
        v.sort()
        for p, q in zip(v, v[1:]):
            if q[0] - p[0] < SALTO_S and math.hypot(q[1] - p[1], q[2] - p[2]) > SALTO_MM:
                saltos.append((i, round(q[0] - t0)))
    # acercamientos < 1 diámetro (episodios), muestreo 0,5 s con la última pose
    ult, idx, prev = {}, {i: 0 for i in P}, False
    t = t0
    while t <= t_end:
        for i, v in P.items():
            while idx[i] < len(v) and v[idx[i]][0] <= t:
                ult[i] = v[idx[i]]
                idx[i] += 1
        pts = [p for p in ult.values() if t - p[0] < 2.0]
        c = any(math.hypot(a[1] - b[1], a[2] - b[2]) < DIAM for k, a in enumerate(pts) for b in pts[k + 1:])
        acerc += c and not prev
        prev = c
        t += 0.5
    return saltos, acerc, hueco_max


def comandos_manuales(consola, t0, t_end):
    out = []
    if not consola or not os.path.exists(consola):
        return out
    for r in csv.DictReader(open(consola, encoding='utf-8', errors='replace')):
        t = fnum(r.get('time'))
        m = r.get('message', '')
        if t is None or not (t0 <= t <= t_end) or not m.startswith('CMD|'):
            continue
        verbo = m[4:].split('|')[0]
        if verbo in RUTINA or (verbo == 'NAV_CONFIG' and '|LAT|' in m):
            continue
        out.append(f"{r.get('robot')} {m[4:][:40]} a los {t - t0:.0f} s")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hoja', default=os.path.join(RAIZ, 'campana_hoja_corrida.csv'))
    ap.add_argument('--desde', help='«dd-mm HH:MM»: solo logs desde ese momento (por omisión: el último congelado)')
    ap.add_argument('--anio', type=int, default=dt.date.today().year)
    ap.add_argument('--seco', action='store_true')
    ap.add_argument('--resumen', action='store_true')
    a = ap.parse_args()

    if a.desde:
        d, h = a.desde.split()
        desde = dt.datetime(a.anio, int(d.split('-')[1]), int(d.split('-')[0]), *map(int, h.split(':')))
        origen = '--desde'
    else:
        desde, carpeta = desde_congelado()
        if not desde:
            sys.exit('No hay firmware congelado (_congelado_*/FIRMWARE_CONGELADO.txt). Congelar primero, '
                     'o indicar --desde "dd-mm HH:MM" para no mezclar las corridas de verificación.')
        origen = os.path.basename(carpeta)
    filas, campos, sep = leer_hoja(a.hoja)
    for c in ('archivo_think', 'archivo_consola', 'notas'):
        if c not in campos:
            campos.append(c)

    usados = {os.path.basename(f.get('archivo_think') or '') for f in filas} | \
             {os.path.basename(f.get('archivo_consola') or '') for f in filas}
    thinks = sorted((p for p in glob.glob(os.path.join(BASE, 'BdieLogs', 'BDIE_*_think.csv'))
                     if (stamp(p, RX_THINK, a.anio) or dt.datetime.min) >= desde),
                    key=lambda p: stamp(p, RX_THINK, a.anio))
    print(f'Desde {desde:%d-%m %H:%M} ({origen}) · {len(thinks)} sesiones BDI-E en BdieLogs\n')

    pendientes = [f for f in filas if (f.get('bloque') or 'nominal') == 'nominal' and not (f.get('archivo_think') or '').strip()]
    pendientes.sort(key=lambda f: int(f.get('orden') or 0))
    descartadas, asignadas = [], []
    fin_anterior = None
    for th in thinks:
        nombre = os.path.basename(th)
        S = info_sesion(th)
        if S is None:
            continue
        if nombre in usados:
            fin_anterior = (stamp(th, RX_THINK, a.anio), S)
            continue
        if S['dur'] < DUR_MIN_S and not S['det']:
            descartadas.append(f'{nombre} ({S["dur"]:.0f} s, sin detección)')
            continue
        if not pendientes:
            descartadas.append(f'{nombre} (no quedan filas nominales pendientes)')
            continue
        f = pendientes.pop(0)
        f['archivo_think'] = nombre
        asignadas.append((f, th, S))
        usados.add(nombre)

    # llenar las filas nominales que tienen think (nuevas y viejas sin completar)
    for f in filas:
        if (f.get('bloque') or 'nominal') != 'nominal':
            continue
        th = ac.ruta_log(f.get('archivo_think'), 'BdieLogs') if hasattr(ac, 'ruta_log') else None
        th = th or (os.path.join(BASE, 'BdieLogs', f['archivo_think']) if f.get('archivo_think') else None)
        if not th or not os.path.exists(th):
            continue
        S = info_sesion(th)
        if S is None:
            continue
        ini = stamp(th, RX_THINK, a.anio)
        notas = [n for n in (f.get('notas') or '').split(' | ') if n and not n.startswith('[auto]')]
        auto = []
        if not (f.get('hora_inicio') or '').strip():
            f['hora_inicio'] = f'{ini:%H:%M:%S}'
        if not (f.get('obj_x_medida') or '').strip():
            o = objeto_camara(th)
            if o:
                f['obj_x_medida'], f['obj_y_medida'] = str(round(o[0])), str(round(o[1]))
                auto.append('cubo según la cámara')
            else:
                auto.append('REVISAR: sin posición del cubo (ni cámara ni medida)')
        # 07-nov 14:31: el operador coloca el cubo en cada corrida; la cámara lo registra. Regla de
        # colocación: ≥ 700 mm de la base y ≥ 200 mm de las paredes del área (1910 × 1400).
        ox, oy = fnum(f.get('obj_x_medida')), fnum(f.get('obj_y_medida'))
        bx, by = fnum(f.get('base_x')) or 300.0, fnum(f.get('base_y')) or 300.0
        if ox is not None and oy is not None:
            d = math.hypot(ox - bx, oy - by)
            if d < 700 or min(ox - 10, 1910 - ox, oy, 1400 - oy) < 200:
                auto.append(f'REVISAR: cubo en ({ox:.0f}, {oy:.0f}) a {d:.0f} mm de la base: fuera de la regla de colocación')
        consola = ac.consola_de(th)
        try:
            R = ac.analizar(th, consola, 0, 1e9)
            if not (f.get('desenlace_script') or '').strip():
                f['desenlace_script'] = R['desenlace']
        except SystemExit:
            pass
        esperados = re.findall(r'\d+', f.get('robots_ids') or '')
        faltan = [i for i in esperados if i not in S['ids']]
        if faltan:
            auto.append('REVISAR: no participó Atta_' + ', Atta_'.join(faltan))
        # Criterio del 07-nov 15:26: un robot que se apaga o queda inmovilizado es «baja» (lo confirma
        # el operador en la columna `bajas`). Acá solo se SUGIERE.
        if not (f.get('bajas') or '').strip():
            filas_th = list(csv.DictReader(open(th, encoding='utf-8', errors='replace')))
            for i in esperados:
                ri = [r for r in filas_th if r['id'] == i and float(r['tlog']) >= S['t0']]
                if not ri or any(r['fase'] == 'FIN' for r in ri):
                    continue
                ult = float(ri[-1]['tlog'])
                if S['tlast'] - ult > 30:
                    auto.append(f'REVISAR posible baja: Atta_{i} dejó de transmitir a los {ult - S["t0"]:.0f} s (¿batería?)')
                    continue
                cola = [(float(r['tlog']), fnum(r['x']), fnum(r['y'])) for r in ri if float(r['tlog']) >= ult - 180]
                xs = [c[1] for c in cola if c[1] is not None]; ys = [c[2] for c in cola if c[2] is not None]
                if cola and ult - cola[0][0] >= 170 and xs and max(xs) - min(xs) < 150 and max(ys) - min(ys) < 150:
                    auto.append(f'REVISAR posible baja: Atta_{i} inmóvil los últimos 3 min en ({xs[-1]:.0f}, {ys[-1]:.0f}), sin llegar a FIN')
        saltos, acerc, hueco = revisar_poses(th, S['t0'], S['t_end'])
        if not (f.get('colisiones') or '').strip() and acerc is not None:
            f['colisiones'] = f'auto: {acerc} acercamientos < {DIAM:.0f} mm'
        if saltos:
            auto.append('REVISAR posible manipulación: ' + ', '.join(f'Atta_{i} a los {t} s' for i, t in saltos[:4]))
        if hueco and hueco > HUECO_CAMARA_S:
            auto.append(f'REVISAR: la cámara no dio poses durante {hueco:.0f} s (¿inválida?)')
        cmds = comandos_manuales(consola, S['t0'], S['t_end'])
        if cmds and not (f.get('intervenciones_hora_robot_fase_que') or '').strip():
            f['intervenciones_hora_robot_fase_que'] = 'auto: ' + '; '.join(cmds[:5])
        f['notas'] = ' | '.join(notas + ['[auto] ' + a2 for a2 in auto])

    # montaje: fin de la corrida anterior → inicio de esta (misma consola de la base)
    con_think = sorted([f for f in filas if f.get('archivo_think')], key=lambda f: f['archivo_think'])
    prev = None
    for f in con_think:
        th = os.path.join(BASE, 'BdieLogs', f['archivo_think'])
        if not os.path.exists(th):
            continue
        ini = stamp(th, RX_THINK, a.anio)
        S = info_sesion(th)
        if prev and S and not (f.get('t_montaje_s') or '').strip():
            pini, pS = prev
            gap = (ini - pini).total_seconds() - (pS['tlast'] - pS['t0']) if pS else None
            if gap is not None and 0 < gap < 3600:
                f['t_montaje_s'] = str(round(gap))
        prev = (ini, S)

    # línea base: episodios SEARCH_OBJECT en las consolas desde el congelado
    lb_pend = sorted([f for f in filas if f.get('bloque') == 'linea_base' and not (f.get('archivo_consola') or '').strip()],
                     key=lambda f: int(f.get('orden') or 0))
    if lb_pend:
        consolas = sorted((p for p in glob.glob(os.path.join(BASE, 'ConsoleLogs', 'Console_Log_*.csv'))
                           if (stamp(p, RX_CONS, a.anio) or dt.datetime.min) >= desde.replace(second=0) - dt.timedelta(minutes=1)),
                          key=lambda p: stamp(p, RX_CONS, a.anio))
        ya = {}
        for f in filas:
            if f.get('bloque') == 'linea_base' and f.get('archivo_consola'):
                ya[f['archivo_consola']] = ya.get(f['archivo_consola'], 0) + 1
        import resumen_campana as rc
        for con in consolas:
            eps = rc.episodios_linea_base(con)
            libres = len(eps) - ya.get(os.path.basename(con), 0)
            for _ in range(max(0, libres)):
                if not lb_pend:
                    break
                f = lb_pend.pop(0)
                f['archivo_consola'] = os.path.basename(con)
                asignadas.append((f, con, None))

    # ── informe ──
    for f, th, S in asignadas:
        print(f"  orden {f['orden']:>2} ({f['bloque']}) ← {os.path.basename(th)}"
              + (f"   {f.get('desenlace_script', '')}" if S else ''))
    if not asignadas:
        print('  (ninguna corrida nueva)')
    if descartadas:
        print('\nNo asignadas: ' + '; '.join(descartadas))
    revisar = [(f['orden'], n) for f in filas for n in (f.get('notas') or '').split(' | ') if 'REVISAR' in n]
    interv = [(f['orden'], f['intervenciones_hora_robot_fase_que']) for f in filas
              if (f.get('intervenciones_hora_robot_fase_que') or '').startswith('auto:')]
    if interv:
        print('\nIntervenciones por software detectadas:')
        for o, t in interv:
            print(f'  orden {o}: {t}')
    if revisar:
        print('\nPara REVISAR (decide el operador; nada se marcó inválido):')
        for o, n in revisar:
            print(f'  orden {o}: {n.replace("[auto] ", "")}')
    hechas = sum(1 for f in filas if f.get('bloque') == 'nominal' and f.get('archivo_think'))
    print(f'\nNominales con log: {hechas} de {sum(1 for f in filas if f.get("bloque") == "nominal")}. '
          f'Próxima: orden {pendientes[0]["orden"]} — cubo donde el operador lo ponga (≥ 700 mm de la base, ≥ 200 mm de las paredes)'
          if pendientes else '\nNominales: todas con log.')

    if a.seco:
        print('\n(--seco: no se escribió nada)')
        return
    try:
        with open(a.hoja, 'w', newline='', encoding='utf-8-sig') as o:
            w = csv.DictWriter(o, fieldnames=campos, delimiter=sep, extrasaction='ignore')
            w.writeheader()
            w.writerows(filas)
    except PermissionError:
        sys.exit(f'\n✗ No se pudo escribir {a.hoja}: ¿está abierta en Excel? Cerrarla y repetir.')
    print(f'\n✓ Hoja actualizada: {a.hoja}')
    if a.resumen:
        subprocess.run([sys.executable, os.path.join(AQUI, 'resumen_campana.py'), '--hoja', a.hoja, '--sin-figuras'])


if __name__ == '__main__':
    main()
