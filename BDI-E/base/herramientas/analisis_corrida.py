#!/usr/bin/env python3
"""Análisis de una corrida del enjambre a partir de los registros de la base.

Uso:
    python analisis_corrida.py                  # la corrida más reciente
    python analisis_corrida.py --think BdieLogs/BDIE_<fecha>_think.csv
    python analisis_corrida.py --desde 300 --hasta 900   # una ventana (s del registro)
    python analisis_corrida.py --csv resumen.csv         # agrega una fila por corrida

Calcula solo lo que está en los registros; lo que se anota a mano (colisiones,
intervenciones, posición medida del objeto) va en la hoja de corrida.

  Misión:   inicio (primer DESPLEGAR), detección (primer obj=1) y descubridor,
            cobertura en la detección, cierre de la congregación (primer
            VOLVER), FIN por robot, ABORT/SEGURO y desenlace.
  Estabilidad (prueba de 10 min): por robot, latidos HB recibidos por la base,
            pérdida estimada por número de secuencia, hueco máximo entre HB,
            huecos de más de 3 s («caído» para el enjambre), reinicios (la
            secuencia vuelve a empezar), huecos de pose de más de 1 s y
            vigilantes de movimiento y de viaje que saltaron.
  Latencia entre robots: las filas LAT|B|rtt|env|rec que reporta cada robot con
            NAV_CONFIG|LAT|1, por par A→B y en total: mediana, p95 y máximo de
            la ida y vuelta, porcentaje ≤ 200 ms y pérdida. `--lat` calcula
            solo esto, desde la consola, sin necesitar el think (la prueba de
            estabilidad puede correr sin misión).
"""
import argparse, csv, glob, os, re, sys
from collections import defaultdict

AQUI = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(AQUI)
CAIDA_S = 3.0      # mismo umbral que el agente: 3 s sin oír a un par = caído
HUECO_POSE_S = 1.0
UMBRAL_LAT_MS = 200   # límite de latencia entre robots
PASADA_MM = 200    # centro del robot a menos de esto del centro del cubo = «pasó al lado»

def ultimo(patron):
    fs = glob.glob(os.path.join(BASE, patron))
    return max(fs, key=os.path.getmtime) if fs else None

def consola_de(think):
    """La consola que cubre el think: la más reciente creada antes que él."""
    t0 = os.path.getmtime(think)
    cs = sorted(glob.glob(os.path.join(BASE, 'ConsoleLogs', '*.csv')), key=os.path.getmtime)
    cand = [c for c in cs if os.path.getmtime(c) >= t0 - 5]
    return cand[0] if cand else (cs[-1] if cs else None)

def f(x, nd=0):
    return '—' if x is None else (f'{x:.{nd}f}')

def percentil(v, q):
    """Percentil por rango más cercano (sin interpolar): un valor que existió."""
    v = sorted(v)
    return v[max(0, min(len(v) - 1, -(-int(q * len(v) * 1000) // 1000) - 1))] if v else None

def latencia_robots(consola, desde=0, hasta=1e9):
    """Filas LAT|B|rtt_ms|enviados|recibidos de la consola → resumen por par y total.

    rtt es la ida y vuelta A→(difusión)→B→(directo)→A medida con el reloj de A:
    cota superior de la latencia de difusión entre robots. La pérdida sale de los
    contadores acumulados (pongs recibidos / pings enviados entre el primer y el
    último reporte del par), así que no la sesga un LAT que se pierda camino a la
    base. Si el robot reinicia el contador (NAV_CONFIG|LAT|1 de nuevo) se suman
    los tramos por separado."""
    if not consola or not os.path.exists(consola):
        return None
    rtt = defaultdict(list)
    tramos = defaultdict(list)          # (a,b) -> [[env0, rec0, env1, rec1], ...]
    for r in csv.reader(open(consola, encoding='utf-8', errors='replace')):
        if len(r) < 4 or not r[3].startswith('LAT|'):
            continue
        try:
            t = float(r[0])
        except ValueError:
            continue
        if not (desde <= t <= hasta):
            continue
        p = r[3].split('|')
        if len(p) < 5:
            continue
        try:
            b, ms, env, rec = p[1], float(p[2]), int(p[3]), int(p[4])
        except ValueError:
            continue
        a = r[1]
        rtt[(a, b)].append(ms)
        tr = tramos[(a, b)]
        if not tr or env < tr[-1][2]:
            tr.append([env, rec, env, rec])
        else:
            tr[-1][2], tr[-1][3] = env, rec
    if not rtt:
        return None
    pares = {}
    for k, v in rtt.items():
        env = sum(x[2] - x[0] for x in tramos[k])
        rec = sum(x[3] - x[1] for x in tramos[k])
        pares[k] = dict(n=len(v), med=percentil(v, .5), p95=percentil(v, .95), max=max(v),
                        bajo=100.0 * sum(1 for x in v if x <= UMBRAL_LAT_MS) / len(v),
                        perdida=100.0 * (1 - rec / env) if env > 0 else None)
    todo = [x for v in rtt.values() for x in v]
    total = dict(n=len(todo), med=percentil(todo, .5), p95=percentil(todo, .95), max=max(todo),
                 bajo=100.0 * sum(1 for x in todo if x <= UMBRAL_LAT_MS) / len(todo))
    return dict(pares=pares, total=total)

def imprimir_lat(L):
    print(f'\n── Latencia entre robots (LAT_PING: ida y vuelta, cota superior de la difusión robot→robot) ──')
    if not L:
        print('  sin filas LAT| en la consola (¿se encendió NAV_CONFIG|LAT|1?)'); return
    print(f"  {'par':<16}{'n':>6}{'mediana':>10}{'p95':>8}{'máx':>8}{'≤'+str(UMBRAL_LAT_MS)+' ms':>10}{'pérdida':>10}")
    for (a, b), e in sorted(L['pares'].items(), key=lambda kv: (int(kv[0][0]), int(kv[0][1]))):
        print(f"  Atta_{a} → Atta_{b:<5}{e['n']:>6}{f(e['med'],1):>8} ms{f(e['p95'],1):>6}{f(e['max'],1):>8}"
              f"{f(e['bajo'],1):>9} %{f(e['perdida'],1):>8} %")
    t = L['total']
    print(f"  {'TODOS':<16}{t['n']:>6}{f(t['med'],1):>8} ms{f(t['p95'],1):>6}{f(t['max'],1):>8}{f(t['bajo'],1):>9} %")
    print(f"  ida estimada ≈ rtt/2: mediana {f(t['med']/2,1)} ms · p95 {f(t['p95']/2,1)} ms "
          f"(supone tramos simétricos; la cota segura es el rtt)")

def analizar(think, consola, desde, hasta):
    R = {'think': os.path.basename(think), 'consola': os.path.basename(consola) if consola else '—'}
    filas = [r for r in csv.DictReader(open(think, encoding='utf-8', errors='replace'))
             if desde <= float(r['tlog']) <= hasta]
    if not filas:
        sys.exit('El think no tiene filas en esa ventana.')
    ids = sorted({r['id'] for r in filas}, key=int)
    R['robots'] = ids
    t_ini = next((float(r['tlog']) for r in filas if r['fase'] == 'DESPLEGAR'), None)
    det = next((r for r in filas if r['obj'] == '1'), None)
    t_det = float(det['tlog']) if det else None
    hallo = None
    t_cierre = next((float(r['tlog']) for r in filas if r['fase'] == 'VOLVER'), None)
    t_seguro = next((float(r['tlog']) for r in filas
                     if r['fase'] == 'SEGURO' and t_ini and float(r['tlog']) > t_ini), None)
    fin = {}
    for r in filas:
        if r['fase'] == 'FIN' and r['id'] not in fin:
            fin[r['id']] = float(r['tlog'])
    cob_det = int(det['cob']) if det else None
    R.update(t_ini=t_ini, t_det=t_det, t_cierre=t_cierre, t_seguro=t_seguro, fin=fin, cob_det=cob_det)

    # consola: hallazgo, vigilantes, HB
    hb = defaultdict(list)          # id -> [(t, seq)]
    warn = defaultdict(lambda: defaultdict(int))
    trazas = defaultdict(lambda: defaultdict(int))
    lecturas = []
    if consola:
        for r in csv.DictReader(open(consola, encoding='utf-8', errors='replace')):
            try:
                t = float(r['time'])
            except ValueError:
                continue
            if not (desde <= t <= hasta):
                continue
            m = r['message']
            if m.startswith('HB|'):
                p = m.split('|')
                if len(p) >= 5 and p[4].isdigit():
                    hb[p[1]].append((t, int(p[4])))
            elif m.startswith('OBJECT_FOUND|') and hallo is None:
                hallo = r['robot']
            elif m.startswith('SEARCH|'):          # trazas de la búsqueda
                q = m.split('|')
                if len(q) >= 4:
                    tipo = re.sub(r'[\d=().,-]+.*$', '', q[3]).strip() or q[3][:20]
                    trazas[r['robot']][tipo] += 1
            elif m.startswith('SEARCH_COLOR'):
                lecturas.append((t, r['robot'], m))
            elif m.startswith('WARNING'):
                k = re.sub(r' en \d+ ms.*| a \(.*', '', m).replace('WARNING: ', '')
                warn[r['robot']][k] += 1
    R.update(hallo=hallo, lecturas=lecturas, warn=warn, trazas=trazas)

    # estabilidad
    est = {}
    for i, v in hb.items():
        v.sort()
        reinicios = sum(1 for a, b in zip(v, v[1:]) if b[1] < a[1])
        esperados, tramo = 0, [v[0]]
        for a, b in zip(v, v[1:]):
            if b[1] < a[1]:
                esperados += tramo[-1][1] - tramo[0][1] + 1
                tramo = [b]
            else:
                tramo.append(b)
        esperados += tramo[-1][1] - tramo[0][1] + 1
        huecos = [b[0] - a[0] for a, b in zip(v, v[1:])]
        est[i] = dict(n=len(v), esperados=esperados,
                      perdida=100.0 * (1 - len(v) / esperados) if esperados else None,
                      hueco_max=max(huecos) if huecos else None,
                      caidas=sum(1 for h in huecos if h > CAIDA_S),
                      reinicios=reinicios, dur=v[-1][0] - v[0][0])
    poses = think.replace('_think.csv', '_poses.csv')
    hp = defaultdict(lambda: [0, 0.0, None])
    if os.path.exists(poses):
        for r in csv.DictReader(open(poses, encoding='utf-8', errors='replace')):
            t = float(r['tlog'])
            if not (desde <= t <= hasta):
                continue
            s = hp[r['id']]
            if s[2] is not None and t - s[2] > HUECO_POSE_S:
                s[0] += 1; s[1] = max(s[1], t - s[2])
            s[2] = t
    # Lo que cuenta para el enjambre: a quién da por vivo CADA agente (columna
    # vivos = máscara de bits por ID, hex). Un bit que se apaga = ese agente dio
    # a un par por caído (3 s sin oír nada de él: HB, ZC, ZA, TGT_REQ).
    todos = 0
    for i in ids:
        todos |= 1 << int(i)
    viv = {}
    prev = {}
    # Solo durante la misión: los primeros 5 s cada uno se oye solo a sí mismo
    # (todavía no llegó ningún HB) y después de FIN/SEGURO ya no importa.
    t_fin_mision = max(list(fin.values()) + ([t_seguro] if t_seguro else [])) if (fin or t_seguro) else 1e9
    t_a = (t_ini or 0) + 5.0
    for r in filas:
        if not (t_a <= float(r['tlog']) <= t_fin_mision):
            continue
        try:
            m = int(r['vivos'], 16)
        except ValueError:
            continue
        v = viv.setdefault(r['id'], dict(n=0, completos=0, bajas=0, detalle=defaultdict(int)))
        v['n'] += 1
        if m & todos == todos:
            v['completos'] += 1
        p = prev.get(r['id'])
        if p is not None:
            perdidos = p & ~m & todos
            for j in range(16):
                if perdidos >> j & 1:
                    v['bajas'] += 1; v['detalle'][j] += 1
        prev[r['id']] = m
    # El objetivo según la cámara (o según «Objeto» en la GUI) y las pasadas
        # cerca de él que no terminaron en detección.
    obj = None
    mapas = think.replace('_think.csv', '_mapas.jsonl')
    if os.path.exists(mapas):
        import json
        for linea in open(mapas, encoding='utf-8', errors='replace'):
            if '"objetivo_real"' in linea:
                try:
                    o = json.loads(linea).get('objetivo_real')
                    if o: obj = tuple(o)
                except ValueError:
                    pass
    pasadas = {}
    if obj and os.path.exists(poses):
        import math
        lim = t_det if t_det else hasta
        cerca = {}
        for r in csv.DictReader(open(poses, encoding='utf-8', errors='replace')):
            t = float(r['tlog'])
            if not ((t_ini or 0) <= t <= lim):
                continue
            d = math.hypot(float(r['x']) - obj[0], float(r['y']) - obj[1])
            p = pasadas.setdefault(r['id'], [])
            if d < PASADA_MM:
                if cerca.get(r['id']) is None:
                    cerca[r['id']] = [t, d]; p.append(cerca[r['id']])
                else:
                    cerca[r['id']][1] = min(cerca[r['id']][1], d)
            else:
                cerca[r['id']] = None
    R.update(est=est, huecos_pose=hp, viv=viv, obj=obj, pasadas=pasadas)
    n_fin = len(fin)
    if t_seguro and n_fin < len(ids):
        desenlace = f'ABORTADA / SEGURO (FIN {n_fin}/{len(ids)})'
    elif n_fin == len(ids):
        desenlace = 'COMPLETA (todos en FIN)'
    else:
        desenlace = f'INCOMPLETA (FIN {n_fin}/{len(ids)})'
    R['desenlace'] = desenlace
    R['lat'] = latencia_robots(consola, desde, hasta)
    return R

def imprimir(R):
    rel = lambda t: None if (t is None or R['t_ini'] is None) else t - R['t_ini']
    print(f"Corrida: {R['think']}   consola: {R['consola']}")
    print(f"Robots: {', '.join('Atta_'+i for i in R['robots'])}")
    print('\n── Misión (s desde el inicio) ─────────────────────')
    print(f"  inicio (DESPLEGAR)      t_log = {f(R['t_ini'],1)}")
    print(f"  detección               {f(rel(R['t_det']),1)} s   por {R['hallo'] or '—'}   cobertura {f(R['cob_det'])} celdas")
    cong = None if (R['t_cierre'] is None or R['t_det'] is None) else R['t_cierre'] - R['t_det']
    print(f"  cierre congregación     {f(rel(R['t_cierre']),1)} s   (congregación {f(cong,1)} s)")
    for i in R['robots']:
        t = R['fin'].get(i)
        ret = None if (t is None or R['t_cierre'] is None) else t - R['t_cierre']
        print(f"  FIN Atta_{i:<3}            {f(rel(t),1)} s   (retorno {f(ret,1)} s)")
    if R['t_seguro']:
        print(f"  SEGURO/ABORT            {f(rel(R['t_seguro']),1)} s")
    print(f"  DESENLACE: {R['desenlace']}")
    if R.get('obj'):
        print(f"  objeto (cámara/GUI) en ({R['obj'][0]:.0f}, {R['obj'][1]:.0f}) mm · pasadas a < {PASADA_MM} mm ANTES de la detección:")
        for i in R['robots']:
            ps = R['pasadas'].get(i, [])
            txt = ', '.join(f"{rel(t):.0f} s a {d:.0f} mm" for t, d in ps) or 'ninguna'
            print(f"    Atta_{i}: {len(ps)} — {txt}")
    else:
        print("  objeto: sin posición real en el log (ni cámara ni «Objeto»)")
    if R['lecturas']:
        print('  lecturas de color:')
        for t, rb, m in R['lecturas']:
            print(f"    {f(rel(t),1):>7} s  {rb}: {m[:90]}")
    print('\n── Estabilidad — lo que ve cada agente durante la misión (columna vivos del THINK) ──')
    for i in R['robots']:
        v = R['viv'].get(i)
        if not v: continue
        det = ', '.join(f'Atta_{j} ×{n}' for j, n in sorted(v['detalle'].items()))
        print(f"  Atta_{i:<3} ve a todos el {100.0*v['completos']/v['n']:.1f} % del tiempo · "
              f"dio por caído a un par {v['bajas']} veces{(' (' + det + ')') if det else ''}")
    print('\n── Estabilidad — lo que recibe la BASE (referencia: el broadcast a la PC pierde más) ──')
    print(f"  {'robot':<8}{'HB':>6}{'esper.':>8}{'pérd.%':>8}{'hueco máx':>11}{'>3 s':>6}{'reinic.':>9}{'huecos pose>1s':>16}")
    for i in R['robots']:
        e = R['est'].get(i)
        hp = R['huecos_pose'].get(i, [0, 0.0])
        if not e:
            print(f"  Atta_{i:<3} sin HB en la consola"); continue
        print(f"  Atta_{i:<3}{e['n']:>6}{e['esperados']:>8}{f(e['perdida'],1):>8}{f(e['hueco_max'],1):>10}s{e['caidas']:>6}{e['reinicios']:>9}{hp[0]:>9} (máx {hp[1]:.1f} s)")
    if R['est']:
        d = max(e['dur'] for e in R['est'].values())
        reinic = sum(e['reinicios'] for e in R['est'].values())
        bajas = sum(v['bajas'] for v in R['viv'].values())
        print(f"  duración observada {d/60:.1f} min · reinicios {reinic} · bajas vistas por los agentes {bajas}")
    if R.get('trazas'):
        print('\n── Búsqueda (trazas SEARCH| del robot) ────────────')
        for rb, d in sorted(R['trazas'].items()):
            print(f"  {rb}: " + ', '.join(f'{k} ×{n}' for k, n in sorted(d.items(), key=lambda kv: -kv[1])))
    if R.get('lat'):
        imprimir_lat(R['lat'])
    if R['warn']:
        print('\n── Vigilantes que saltaron ────────────────────────')
        for rb, d in sorted(R['warn'].items()):
            print(f"  {rb}: " + ', '.join(f'{k} ×{n}' for k, n in sorted(d.items())))

def fila_csv(R, ruta):
    rel = lambda t: '' if (t is None or R['t_ini'] is None) else f"{t - R['t_ini']:.1f}"
    nuevo = not os.path.exists(ruta)
    with open(ruta, 'a', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        if nuevo:
            w.writerow(['think', 'robots', 't_det', 'hallo', 'cob_det', 't_cierre', 'n_fin',
                        't_fin_ultimo', 'desenlace', 'perdida_hb_base_max', 'bajas_agentes', 'reinicios'])
        fins = [R['fin'][i] for i in R['robots'] if i in R['fin']]
        w.writerow([R['think'], len(R['robots']), rel(R['t_det']), R['hallo'] or '', R['cob_det'] or '',
                    rel(R['t_cierre']), len(fins), rel(max(fins)) if fins else '', R['desenlace'],
                    f"{max((e['perdida'] or 0) for e in R['est'].values()):.1f}" if R['est'] else '',
                    sum(v['bajas'] for v in R['viv'].values()), sum(e['reinicios'] for e in R['est'].values())])

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--think'); ap.add_argument('--consola')
    ap.add_argument('--desde', type=float, default=0); ap.add_argument('--hasta', type=float, default=1e9)
    ap.add_argument('--csv')
    ap.add_argument('--lat', action='store_true',
                    help='solo la latencia entre robots, de la consola (no necesita think)')
    a = ap.parse_args()
    if a.lat:
        consola = a.consola and (a.consola if os.path.isabs(a.consola) else os.path.join(BASE, a.consola)) \
                  or ultimo('ConsoleLogs/*.csv')
        print(f'Consola: {os.path.basename(consola) if consola else "—"}')
        imprimir_lat(latencia_robots(consola, a.desde, a.hasta))
        sys.exit(0)
    think = a.think and (a.think if os.path.isabs(a.think) else os.path.join(BASE, a.think)) or ultimo('BdieLogs/*_think.csv')
    consola = a.consola and (a.consola if os.path.isabs(a.consola) else os.path.join(BASE, a.consola)) or consola_de(think)
    R = analizar(think, consola, a.desde, a.hasta)
    imprimir(R)
    if a.csv:
        fila_csv(R, a.csv); print(f'\nFila agregada a {a.csv}')
