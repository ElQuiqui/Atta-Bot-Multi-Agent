#!/usr/bin/env python3
"""Resumen estadístico de una campaña sobre el enjambre físico.

Calcula, con el plan de análisis fijado antes de los datos: tasas por fase con
intervalos de Clopper–Pearson y de Wilson, prueba binomial exacta de la tasa de
misión contra p0 = 0,80, fracción de agentes con bootstrap por conglomerados,
tiempos por fase (mediana con intervalo bootstrap, media ± U con k = 2),
Kaplan–Meier con censura, comparación con una línea base si la hay (log-rank y
Mann–Whitney), métricas de la cámara (radio del enjambre en el retorno,
distancia mínima entre robots, recorrido, error en la base) y tendencia contra
el orden de las corridas. Python puro; matplotlib solo para las figuras.

Entradas
  · la hoja de corrida (`campana_hoja_corrida.csv`): una fila por corrida, con
    `archivo_think` (nominales) o `archivo_consola` (línea base), validez e
    intervenciones. Una corrida inválida que se repite son dos filas con el
    mismo `orden`; la inválida lleva `valida_s_n = n` y su causa.
  · los registros de la base: `base/BdieLogs/*_think.csv`, `*_poses.csv`,
    `base/ConsoleLogs/*.csv`.

Uso (desde BDI-E/base):
    python herramientas/resumen_campana.py
    python herramientas/resumen_campana.py --hoja ../campana_hoja_corrida.csv --salida herramientas/resumen_campana
    python herramientas/resumen_campana.py --prueba-24oct     # ensayo con corridas de prueba

Salidas (en --salida): resumen.md · por_corrida.csv · por_agente.csv ·
tabla_fases.tex · figuras PDF (si hay matplotlib).
"""
import argparse
import csv
import json
import math
import os
import random
import re
import statistics as st
import sys
from collections import defaultdict

AQUI = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(AQUI)
RAIZ = os.path.dirname(BASE)                      # carpeta BDI-E

T_LIM = 1200.0          # tiempo límite de misión (s)
T_LIM_LB = 600.0        # la caminata aleatoria del firmware dura 600 s
P0 = 0.80               # umbral de la tasa de misión exitosa (M1)
ALFA = 0.05
U_T = 0.15 / math.sqrt(3)   # tipo B de un instante: THINK al cambiar (≥ 100 ms) + viaje (~50 ms), rectangular
SIGMA_POS = 8.0         # σ de la posición ArUco por eje (mm)
DIAM = 150.0            # diámetro del robot (mm)
SALTO_POSE = 300.0      # salto entre poses consecutivas que se descarta como error de ArUco (mm)
N_BOOT = 10000
SEMILLA = 20261107

# ════════════════════════════════════════════════════════════════════════════
# Estadística (Python puro)
# ════════════════════════════════════════════════════════════════════════════

def binom_sf(k, n, p):
    """P(X >= k)."""
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(max(k, 0), n + 1))


def binom_cdf(k, n, p):
    """P(X <= k)."""
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(0, k + 1)) if k >= 0 else 0.0


def _bisect(f, lo=0.0, hi=1.0, it=80):
    for _ in range(it):
        m = (lo + hi) / 2
        if f(m):
            hi = m
        else:
            lo = m
    return (lo + hi) / 2


def clopper_pearson(k, n, a=ALFA):
    if n == 0:
        return (float('nan'), float('nan'))
    lo = 0.0 if k == 0 else _bisect(lambda p: binom_sf(k, n, p) >= a / 2)
    hi = 1.0 if k == n else _bisect(lambda p: binom_cdf(k, n, p) <= a / 2)
    return lo, hi


def cp_inferior_unilateral(k, n, a=ALFA):
    return 0.0 if k == 0 else _bisect(lambda p: binom_sf(k, n, p) >= a)


def wilson(k, n, z=1.959964):
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def percentil(v, q):
    v = sorted(v)
    if not v:
        return float('nan')
    i = (len(v) - 1) * q
    a, b = int(math.floor(i)), int(math.ceil(i))
    return v[a] + (v[b] - v[a]) * (i - a)


def boot_ic(datos, estad, rng, n=N_BOOT, a=ALFA):
    if len(datos) < 2:
        return (float('nan'), float('nan'))
    r = []
    for _ in range(n):
        m = [datos[rng.randrange(len(datos))] for _ in datos]
        try:
            r.append(estad(m))
        except (ValueError, ZeroDivisionError, st.StatisticsError):
            pass
    r = [x for x in r if x == x]
    return (percentil(r, a / 2), percentil(r, 1 - a / 2)) if r else (float('nan'), float('nan'))


def kaplan_meier(tiempos, eventos):
    """Devuelve [(t, S(t))] en los tiempos de evento."""
    pares = sorted(zip(tiempos, eventos))
    n = len(pares)
    s, curva, i = 1.0, [(0.0, 1.0)], 0
    while i < n:
        t = pares[i][0]
        d = c = 0
        while i < n and pares[i][0] == t:
            d += pares[i][1]
            c += 1 - pares[i][1]
            i += 1
        en_riesgo = n - (i - d - c)
        if d:
            s *= 1 - d / en_riesgo
            curva.append((t, s))
    return curva


def km_mediana(tiempos, eventos):
    for t, s in kaplan_meier(tiempos, eventos):
        if s <= 0.5:
            return t
    return float('nan')   # no se alcanza: más de la mitad censuradas


def log_rank(t1, e1, t2, e2):
    """Prueba de rangos logarítmicos, dos grupos. Devuelve (chi2, p)."""
    datos = [(t, e, 0) for t, e in zip(t1, e1)] + [(t, e, 1) for t, e in zip(t2, e2)]
    ts = sorted({t for t, e, _ in datos if e})
    O1 = E1 = V = 0.0
    for t in ts:
        n1 = sum(1 for x, _, g in datos if x >= t and g == 0)
        n2 = sum(1 for x, _, g in datos if x >= t and g == 1)
        d1 = sum(1 for x, e, g in datos if x == t and e and g == 0)
        d = sum(1 for x, e, _ in datos if x == t and e)
        n = n1 + n2
        if n < 2:
            continue
        O1 += d1
        E1 += d * n1 / n
        V += d * (n1 / n) * (n2 / n) * (n - d) / (n - 1)
    if V <= 0:
        return float('nan'), float('nan')
    chi2 = (O1 - E1) ** 2 / V
    return chi2, math.erfc(math.sqrt(chi2 / 2))


def mann_whitney(a, b):
    """U de Mann–Whitney con aproximación normal y corrección por empates. (U, p bilateral)."""
    n1, n2 = len(a), len(b)
    if not n1 or not n2:
        return float('nan'), float('nan')
    todos = sorted([(x, 0) for x in a] + [(x, 1) for x in b])
    rangos, i = {}, 0
    rk = [0.0] * len(todos)
    while i < len(todos):
        j = i
        while j < len(todos) and todos[j][0] == todos[i][0]:
            j += 1
        for k in range(i, j):
            rk[k] = (i + j + 1) / 2
        i = j
    R1 = sum(r for r, (_, g) in zip(rk, todos) if g == 0)
    U = R1 - n1 * (n1 + 1) / 2
    N = n1 + n2
    empates = defaultdict(int)
    for x, _ in todos:
        empates[x] += 1
    T = sum(t ** 3 - t for t in empates.values())
    var = n1 * n2 / 12 * ((N + 1) - T / (N * (N - 1)))
    if var <= 0:
        return U, float('nan')
    z = (U - n1 * n2 / 2) / math.sqrt(var)
    return U, math.erfc(abs(z) / math.sqrt(2))


def spearman_perm(x, y, rng, n=10000):
    if len(x) < 4:
        return float('nan'), float('nan')

    def rangos(v):
        o = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(o):
            j = i
            while j < len(o) and v[o[j]] == v[o[i]]:
                j += 1
            for k in range(i, j):
                r[o[k]] = (i + j + 1) / 2
            i = j
        return r

    def pearson(a, b):
        ma, mb = st.mean(a), st.mean(b)
        num = sum((p - ma) * (q - mb) for p, q in zip(a, b))
        den = math.sqrt(sum((p - ma) ** 2 for p in a) * sum((q - mb) ** 2 for q in b))
        return num / den if den else float('nan')

    rx, ry = rangos(x), rangos(y)
    rho = pearson(rx, ry)
    if rho != rho:
        return rho, float('nan')
    cuenta = 0
    ry2 = ry[:]
    for _ in range(n):
        rng.shuffle(ry2)
        if abs(pearson(rx, ry2)) >= abs(rho) - 1e-12:
            cuenta += 1
    return rho, (cuenta + 1) / (n + 1)


# ════════════════════════════════════════════════════════════════════════════
# Lectura de logs
# ════════════════════════════════════════════════════════════════════════════

def ruta_log(nombre, carpeta):
    if not nombre:
        return None
    for c in (nombre, os.path.join(BASE, carpeta, os.path.basename(nombre)),
              os.path.join(BASE, carpeta, '_antes_campana', os.path.basename(nombre))):
        if os.path.exists(c):
            return c
    return None


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def ids_intervenidos(texto):
    """IDs de robot mencionados en la columna de intervenciones; '*' si hubo intervención sin ID."""
    texto = (texto or '').strip()
    if not texto or texto.lower() in ('no', 'ninguna', '-', '0'):
        return set()
    ids = set(re.findall(r'(?:atta[_ ]?|robot\s*|r)(\d+)', texto, re.I))
    return ids or {'*'}


def analizar_nominal(think, fila, dv):
    filas = list(csv.DictReader(open(think, encoding='utf-8', errors='replace')))
    t0 = next((float(r['tlog']) for r in filas if r['fase'] == 'DESPLEGAR'), None)
    if t0 is None:
        return None
    filas = [r for r in filas if float(r['tlog']) >= t0]
    esperados = re.findall(r'\d+', fila.get('robots_ids', '') or '')   # «1,2,3,4» o «Atta_1; Atta_2…»
    vistos = sorted({r['id'] for r in filas}, key=lambda s: int(s))
    ids = esperados or vistos
    t_lim = t0 + T_LIM
    ag = {i: dict(buscar=None, conoce=None, lleg=None, fin=None) for i in ids}
    t_det = desc = t_cierre = t_seguro = None
    det_xy = None
    cob_det = None
    t_ult = t0
    for r in filas:
        t = float(r['tlog'])
        t_ult = max(t_ult, t)
        i = r['id']
        if t > t_lim:
            continue
        a = ag.get(i)
        if r['fase'] == 'SEGURO' and t_seguro is None and t > t0 + 1:
            t_seguro = t
        if a is None:
            continue
        if r['fase'] in ('BUSCAR', 'CONGREGAR', 'VOLVER', 'FIN') and a['buscar'] is None:
            a['buscar'] = t
        if r['obj'] == '1' and a['conoce'] is None:
            a['conoce'] = t
            if t_det is None:
                t_det, desc = t, i
                det_xy = (fnum(r['x']), fnum(r['y']))
                cob_det = fnum(r['cob'])
        if r.get('lleg') == '1' and a['lleg'] is None and t_det is not None:
            a['lleg'] = t
        if r['fase'] == 'VOLVER' and t_cierre is None:
            t_cierre = t
        if r['fase'] == 'FIN' and a['fin'] is None:
            a['fin'] = t

    # objeto real: medido en la hoja > cámara («objetivo_real» del log: cubo_vision o «Objeto» del GUI)
    # > nominal de la lista sorteada
    obj = None
    if fnum(fila.get('obj_x_medida')) is not None and fnum(fila.get('obj_y_medida')) is not None:
        obj = (fnum(fila['obj_x_medida']), fnum(fila['obj_y_medida']))
    mapas = think.replace('_think.csv', '_mapas.jsonl')
    if obj is None and os.path.exists(mapas):
        for linea in open(mapas, encoding='utf-8', errors='replace'):
            if '"objetivo_real"' in linea:
                try:
                    o = json.loads(linea).get('objetivo_real')
                    if o:
                        obj = tuple(o)
                except ValueError:
                    pass
    if obj is None and fnum(fila.get('obj_x_nom')) is not None and fnum(fila.get('obj_y_nom')) is not None:
        obj = (fnum(fila['obj_x_nom']), fnum(fila['obj_y_nom']))
    base = (fnum(fila.get('base_x')) or 300.0, fnum(fila.get('base_y')) or 300.0)
    det_verdadera = None
    if t_det is not None and obj and det_xy and None not in det_xy:
        det_verdadera = math.hypot(det_xy[0] - obj[0], det_xy[1] - obj[1]) <= dv

    interv = ids_intervenidos(fila.get('intervenciones_hora_robot_fase_que'))
    N = len(ids)
    # Criterio de éxito: se detecta el objetivo, la congregación se cierra (quórum o
        # temporizador) y todos los agentes operativos llegan a su ranura y vuelven a la
        # base antes de 20 min, sin intervención. Una «baja» (columna `bajas`) es un robot
        # que se apaga (batería) o queda inmovilizado: no se le exige completar la misión,
        # pero cuenta como agente no exitoso en M2. Se calcula además la tasa sin bajas,
        # con todos los robots en la ranura antes del cierre.
    bajas = set(re.findall(r'\d+', fila.get('bajas', '') or ''))
    ids_m = [i for i in ids if i not in bajas]
    Nm = len(ids_m)
    todos = lambda k: Nm > 0 and all(ag[i][k] is not None for i in ids_m)
    F1 = todos('buscar')
    F2 = t_det is not None and det_verdadera is not False
    F2b = F2 and todos('conoce')
    F3 = F2b and t_cierre is not None and all(ag[i]['lleg'] is not None for i in ids_m)
    F4 = F3 and todos('fin')
    exito_agente = {i: i not in bajas and ag[i]['fin'] is not None and i not in interv and '*' not in interv for i in ids}
    mision = F4 and not interv
    literal = (mision and not bajas and all(ag[i]['lleg'] is not None and ag[i]['lleg'] <= t_cierre + 1.0 for i in ids))
    fin_ult = max((ag[i]['fin'] for i in ids_m if ag[i]['fin'] is not None), default=None)

    rel = lambda t: None if t is None else t - t0
    R = dict(
        robots=N, ids=' '.join(ids), bajas=' '.join(sorted(bajas)), descubridor=desc,
        t_desp=max((rel(ag[i]['buscar']) for i in ids_m if ag[i]['buscar'] is not None), default=None) if F1 else None,
        t_det=rel(t_det),
        t_dif=(max(ag[i]['conoce'] for i in ids_m) - t_det) if F2b else None,
        t_cong=(t_cierre - t_det) if (t_cierre and t_det) else None,
        t_ret=(fin_ult - t_cierre) if (F4 and t_cierre) else None,
        t_mis=rel(fin_ult) if F4 else None,
        cob_det=cob_det, det_verdadera=det_verdadera,
        F1=F1, F2=F2, F2b=F2b, F3=F3, F4=F4,
        intervenida=bool(interv), mision_exitosa=mision, todos_fin=todos('fin'),
        # La columna `exito_autor` = s permite marcar como buena una corrida que no cumple
                # el criterio fijado; se reporta aparte de M1, nunca en su lugar.
        literal=literal,
        exito_autor=mision or (fila.get('exito_autor') or '').strip().lower().startswith('s'),
        reclasificada=(not mision) and (fila.get('exito_autor') or '').strip().lower().startswith('s'),
        agentes_exitosos=sum(exito_agente.values()),
        # censura para Kaplan–Meier
        km_det=(rel(t_det), 1) if t_det is not None else (min(T_LIM, rel(t_seguro) if t_seguro else rel(t_ult)), 0),
        km_mis=(rel(fin_ult), 1) if (F4 and not interv) else (min(T_LIM, rel(t_seguro) if t_seguro else rel(t_ult)), 0),
    )
    agentes = []
    for i in ids:
        a = ag[i]
        agentes.append(dict(id=i, buscar=a['buscar'] is not None, conoce=a['conoce'] is not None,
                            lleg=a['lleg'] is not None, fin=a['fin'] is not None,
                            intervenido=(i in interv or '*' in interv), exitoso=exito_agente[i],
                            t_lleg=(a['lleg'] - t_det) if (a['lleg'] and t_det) else None,
                            t_ret=(a['fin'] - t_cierre) if (a['fin'] and t_cierre) else None))

    # ── cámara ──
    poses = think.replace('_think.csv', '_poses.csv')
    R.update(radio_ret_max=None, radio_ret_med=None, dmin=None, contactos=None, dist_base_fin=None)
    if os.path.exists(poses):
        P = defaultdict(list)
        for r in csv.DictReader(open(poses, encoding='utf-8', errors='replace')):
            t = fnum(r.get('tlog'))
            if t is None or t < t0 or t > (fin_ult or t_lim) + 1:
                continue
            if r['id'] in ag:
                P[r['id']].append((t, float(r['x']), float(r['y'])))
        for i in P:
            P[i].sort()
            limpio = [P[i][0]] if P[i] else []
            for p in P[i][1:]:
                if math.hypot(p[1] - limpio[-1][1], p[2] - limpio[-1][2]) <= SALTO_POSE:
                    limpio.append(p)
            P[i] = limpio
        for a in agentes:
            v = P.get(a['id'], [])
            a['recorrido_mm'] = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(v, v[1:])) if v else None
            tf = ag[a['id']]['fin']
            ult = [p for p in v if tf is not None and p[0] <= tf + 2]
            a['dist_base_fin_mm'] = math.hypot(ult[-1][1] - base[0], ult[-1][2] - base[1]) if ult else None
        # muestreo común cada 0,5 s con la última pose conocida
        if P:
            tini = min(v[0][0] for v in P.values() if v)
            tfin = max(v[-1][0] for v in P.values() if v)
            idx = {i: 0 for i in P}
            ult = {}
            radios, dmin, contacto, cont_prev = [], float('inf'), 0, False
            t = tini
            while t <= tfin:
                for i, v in P.items():
                    while idx[i] < len(v) and v[idx[i]][0] <= t:
                        ult[i] = v[idx[i]]
                        idx[i] += 1
                pts = [(p[1], p[2]) for i, p in ult.items() if t - p[0] < 2.0]
                if len(pts) >= 2:
                    dd = min(math.hypot(a[0] - b[0], a[1] - b[1]) for k, a in enumerate(pts) for b in pts[k + 1:])
                    dmin = min(dmin, dd)
                    c = dd < DIAM
                    contacto += c and not cont_prev
                    cont_prev = c
                    if t_cierre and t_cierre <= t <= (fin_ult or t_lim) and len(pts) == len(P):
                        cx = sum(p[0] for p in pts) / len(pts)
                        cy = sum(p[1] for p in pts) / len(pts)
                        radios.append(max(math.hypot(p[0] - cx, p[1] - cy) for p in pts))
                t += 0.5
            R.update(radio_ret_max=max(radios) if radios else None,
                     radio_ret_med=st.median(radios) if radios else None,
                     dmin=dmin if dmin < float('inf') else None, contactos=contacto)
        fins = [a['dist_base_fin_mm'] for a in agentes if a.get('dist_base_fin_mm') is not None]
        R['dist_base_fin'] = st.median(fins) if fins else None
    return R, agentes


def episodios_linea_base(consola):
    """[(t_inicio, t_deteccion o None)] por cada ráfaga de CMD|SEARCH_OBJECT de la consola."""
    eps = []
    filas = list(csv.DictReader(open(consola, encoding='utf-8', errors='replace')))
    for r in filas:
        t = fnum(r.get('time'))
        m = r.get('message', '')
        if t is None:
            continue
        if m.startswith('CMD|') and 'SEARCH_OBJECT' in m:
            if not eps or t - eps[-1][0] > 30:
                eps.append([t, None])
        elif m.startswith('OBJECT_FOUND') and eps and eps[-1][1] is None and t - eps[-1][0] <= T_LIM_LB:
            eps[-1][1] = t
    return [(a, b) for a, b in eps]


# ════════════════════════════════════════════════════════════════════════════
# Resumen
# ════════════════════════════════════════════════════════════════════════════

def fmt(x, nd=1, pct=False):
    if x is None or (isinstance(x, float) and x != x):
        return '—'
    if pct:
        return f'{100 * x:.{nd}f}'.replace('.', ',')
    return f'{x:.{nd}f}'.replace('.', ',')


def resumir_tiempos(v, rng):
    v = [x for x in v if x is not None]
    if not v:
        return None
    n = len(v)
    med = st.median(v)
    ic = boot_ic(v, st.median, rng)
    media = st.mean(v)
    s = st.stdev(v) if n > 1 else float('nan')
    uA = s / math.sqrt(n) if n > 1 else float('nan')
    U = 2 * math.sqrt(uA ** 2 + (math.sqrt(2) * U_T) ** 2) if n > 1 else float('nan')
    return dict(n=n, mediana=med, ic=ic, q1=percentil(v, .25), q3=percentil(v, .75),
                min=min(v), max=max(v), media=media, s=s, U=U)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hoja', default=os.path.join(RAIZ, 'campana_hoja_corrida.csv'))
    ap.add_argument('--salida', default=os.path.join(AQUI, 'resumen_campana'))
    ap.add_argument('--dv', type=float, default=250.0,
                    help='distancia máx. (mm) del CENTRO del descubridor al objeto para que la detección sea verdadera')
    ap.add_argument('--sin-figuras', action='store_true')
    ap.add_argument('--prueba-24oct', action='store_true',
                    help='ensayo con herramientas/corridas_24oct.csv (todas válidas; 13:31 y 15:06 con intervención)')
    a = ap.parse_args()
    rng = random.Random(SEMILLA)

    if a.prueba_24oct:
        hoja = []
        for k, r in enumerate(csv.DictReader(open(os.path.join(AQUI, 'corridas_24oct.csv'), encoding='utf-8')), 1):
            hoja.append(dict(orden=str(k), bloque='nominal', escenario=str(k), archivo_think=r['think'],
                             base_x='300', base_y='300', valida_s_n='s',
                             intervenciones_hora_robot_fase_que='sí (sin ID)' if any(h in r['think'] for h in ('13-31', '15-06')) else ''))
        a.salida = os.path.join(AQUI, '_tmp', 'resumen_prueba_24oct')
    else:
        if not os.path.exists(a.hoja):
            sys.exit(f'No encuentro la hoja de corrida: {a.hoja}')
        # Excel en español guarda los CSV con «;»: se detecta el separador por el encabezado.
        with open(a.hoja, encoding='utf-8-sig', errors='replace') as fh:
            cab = fh.readline()
            fh.seek(0)
            sep = ';' if cab.count(';') > cab.count(',') else ','
            hoja = list(csv.DictReader(fh, delimiter=sep))
    os.makedirs(a.salida, exist_ok=True)

    nominal, agentes_todos, invalidas, pendientes, sin_log, lb = [], [], [], [], [], []
    for f in hoja:
        bloque = (f.get('bloque') or 'nominal').strip()
        valida = (f.get('valida_s_n') or '').strip().lower()
        if bloque == 'linea_base':
            con = ruta_log(f.get('archivo_consola'), 'ConsoleLogs')
            if not con:
                (pendientes if not f.get('archivo_consola') else sin_log).append(f)
                continue
            if valida.startswith('n'):
                invalidas.append(f)
                continue
            lb.append((f, con))
            continue
        th = ruta_log(f.get('archivo_think'), 'BdieLogs')
        if not th:
            (pendientes if not f.get('archivo_think') else sin_log).append(f)
            continue
        if valida.startswith('n'):
            invalidas.append(f)
            continue
        res = analizar_nominal(th, f, a.dv)
        if res is None:
            sin_log.append(f)
            continue
        R, ags = res
        R.update(orden=int(f.get('orden') or 0), escenario=f.get('escenario'),
                 think=os.path.basename(th), marcada_valida=bool(valida))
        nominal.append(R)
        for g in ags:
            g.update(orden=R['orden'])
            agentes_todos.append(g)

    # línea base: episodios en orden dentro de cada consola
    lb_res = []
    por_consola = defaultdict(list)
    for f, con in lb:
        por_consola[con].append(f)
    for con, fs in por_consola.items():
        eps = episodios_linea_base(con)
        for k, f in enumerate(sorted(fs, key=lambda x: int(x.get('orden') or 0))):
            if k < len(eps):
                ti, td = eps[k]
                lb_res.append(dict(orden=int(f.get('orden') or 0), consola=os.path.basename(con),
                                   t_det=(td - ti) if td else None,
                                   km=((td - ti), 1) if td else (T_LIM_LB, 0)))
            else:
                sin_log.append(f)

    n = len(nominal)
    if not n:
        sys.exit('No hay corridas nominales válidas con log todavía.')
    nominal.sort(key=lambda r: r['orden'])

    # ── tasas por fase (corridas) ──
    fases = [('F1', 'Despliegue: todos llegan a BUSCAR'),
             ('F2', 'Detección del objetivo verdadero'),
             ('F2b', 'Difusión: todos conocen el objetivo'),
             ('F3', 'Congregación: cierra y todos los operativos llegan'),
             ('F4', 'Retorno: todos los operativos en FIN antes de 20 min')]
    tabla_fases = []
    previo = n
    for k, nombre in fases:
        kk = sum(1 for r in nominal if r[k])
        cond = (kk, previo)
        acum = (kk, n)
        tabla_fases.append(dict(fase=k, nombre=nombre, llegan=previo, completan=kk,
                                cond=kk / previo if previo else float('nan'), cond_ic=clopper_pearson(*cond),
                                acum=kk / n, acum_ic=clopper_pearson(*acum), acum_w=wilson(*acum)))
        previo = kk
    k_m = sum(1 for r in nominal if r['mision_exitosa'])
    p_valor = binom_sf(k_m, n, P0)
    m1 = dict(k=k_m, n=n, p=k_m / n, cp=clopper_pearson(k_m, n), w=wilson(k_m, n),
              inf1=cp_inferior_unilateral(k_m, n), p_valor=p_valor, demostrado=p_valor <= ALFA)
    k_l = sum(1 for r in nominal if r['literal'])
    m1l = dict(k=k_l, n=n, p=k_l / n, cp=clopper_pearson(k_l, n))
    k_a = sum(1 for r in nominal if r['exito_autor'])
    m1a = dict(k=k_a, n=n, p=k_a / n, cp=clopper_pearson(k_a, n), p_valor=binom_sf(k_a, n, P0))
    reclas = [r['orden'] for r in nominal if r['reclasificada']]
    intervenidas = sum(1 for r in nominal if r['intervenida'])
    completas_con_interv = sum(1 for r in nominal if r['todos_fin'] and r['intervenida'])

    # ── M2: fracción de agentes, bootstrap por conglomerados ──
    fr = [r['agentes_exitosos'] / r['robots'] for r in nominal if r['robots']]
    m2 = dict(media=st.mean(fr), ic=boot_ic(fr, st.mean, rng),
              agrupada=(sum(r['agentes_exitosos'] for r in nominal), sum(r['robots'] for r in nominal)))
    agentes_fase = {}
    for k in ('buscar', 'conoce', 'lleg', 'fin', 'exitoso'):
        por_corrida = defaultdict(list)
        for g in agentes_todos:
            por_corrida[g['orden']].append(1.0 if g[k] else 0.0)
        v = [st.mean(x) for x in por_corrida.values()]
        agentes_fase[k] = (st.mean(v), boot_ic(v, st.mean, rng))

    # ── tiempos ──
    tiempos = {k: resumir_tiempos([r[k] for r in nominal], rng)
               for k in ('t_desp', 't_det', 't_dif', 't_cong', 't_ret', 't_mis')}
    km_det = [r['km_det'] for r in nominal]
    km_mis = [r['km_mis'] for r in nominal]
    km = dict(det=km_mediana(*zip(*km_det)), mis=km_mediana(*zip(*km_mis)))
    nan2 = (float('nan'), float('nan'))
    km_ic = {k: (boot_ic(v, lambda m: km_mediana(*zip(*m)), rng, n=2000) if km[k] == km[k] else nan2)
             for k, v in (('det', km_det), ('mis', km_mis))}

    # ── cámara ──
    camara = {k: resumir_tiempos([r[k] for r in nominal], rng)
              for k in ('radio_ret_max', 'radio_ret_med', 'dmin', 'dist_base_fin', 'cob_det')}
    U_POS = 2 * math.sqrt(2) * SIGMA_POS

    # ── línea base ──
    comp = None
    if lb_res:
        # misma censura (600 s) para los dos grupos
        tb = [min(t, T_LIM_LB) if e else T_LIM_LB for t, e in km_det]
        eb = [1 if (e and t <= T_LIM_LB) else 0 for t, e in km_det]
        tl = [r['km'][0] for r in lb_res]
        el = [r['km'][1] for r in lb_res]
        chi2, p_lr = log_rank(tb, eb, tl, el)
        det_b = [t for t, e in zip(tb, eb) if e]
        det_l = [t for t, e in zip(tl, el) if e]
        U, p_mw = mann_whitney(det_b, det_l)
        mb, ml = km_mediana(tb, eb), km_mediana(tl, el)
        razon = boot_ic(list(zip(tb, eb, [0] * len(tb))) + list(zip(tl, el, [1] * len(tl))),
                        lambda m: (km_mediana([t for t, e, g in m if g == 0], [e for t, e, g in m if g == 0]) /
                                   km_mediana([t for t, e, g in m if g == 1], [e for t, e, g in m if g == 1])),
                        rng, n=2000)
        comp = dict(n_lb=len(lb_res), det_lb=sum(el), med_b=mb, med_l=ml, razon=(mb / ml if ml else float('nan')),
                    razon_ic=razon, chi2=chi2, p_lr=p_lr, U=U, p_mw=p_mw)

    # ── deriva contra el orden ──
    orden = [r['orden'] for r in nominal]
    deriva = {}
    for k in ('t_det', 't_mis'):
        pares = [(o, r[k]) for o, r in zip(orden, nominal) if r[k] is not None]
        if len(pares) >= 4:
            deriva[k] = spearman_perm([p[0] for p in pares], [p[1] for p in pares], rng)
    pares = [(o, 1.0 if r['mision_exitosa'] else 0.0) for o, r in zip(orden, nominal)]
    deriva['exito'] = spearman_perm([p[0] for p in pares], [p[1] for p in pares], rng)

    # ════════ salidas ════════
    with open(os.path.join(a.salida, 'por_corrida.csv'), 'w', newline='', encoding='utf-8') as o:
        cols = ['orden', 'escenario', 'think', 'robots', 'ids', 'bajas', 'literal', 'exito_autor', 'descubridor', 'det_verdadera', 'F1', 'F2', 'F2b', 'F3', 'F4',
                'intervenida', 'mision_exitosa', 'agentes_exitosos', 't_desp', 't_det', 't_dif', 't_cong', 't_ret', 't_mis',
                'cob_det', 'radio_ret_max', 'radio_ret_med', 'dmin', 'contactos', 'dist_base_fin']
        w = csv.DictWriter(o, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for r in nominal:
            w.writerow({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})
    with open(os.path.join(a.salida, 'por_agente.csv'), 'w', newline='', encoding='utf-8') as o:
        cols = ['orden', 'id', 'buscar', 'conoce', 'lleg', 'fin', 'intervenido', 'exitoso', 't_lleg', 't_ret',
                'recorrido_mm', 'dist_base_fin_mm']
        w = csv.DictWriter(o, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for g in agentes_todos:
            w.writerow({k: (round(v, 1) if isinstance(v, float) else v) for k, v in g.items()})

    L = []
    p = L.append
    p('# Resumen de la campaña — enjambre físico\n')
    p(f'Generado por `resumen_campana.py` · hoja: `{os.path.basename(a.hoja) if not a.prueba_24oct else "corridas_24oct.csv (PRUEBA)"}`'
      f' · d_v = {a.dv:.0f} mm · semilla {SEMILLA} · {N_BOOT} réplicas bootstrap\n')
    if a.prueba_24oct:
        p('> ⚠️ **ENSAYO con las corridas del 24-oct.** No son datos de campaña: firmware distinto, sin lista sorteada,'
          ' intervenciones asignadas a mano. Sirve solo para probar el script.\n')
    p(f'**Corridas nominales analizadas:** {n} · inválidas excluidas: {len(invalidas)} · sin correr: {len(pendientes)}'
      f' · con log faltante: {len(sin_log)}')
    sin_marcar = sum(1 for r in nominal if not r['marcada_valida'])
    if sin_marcar and not a.prueba_24oct:
        p(f'  \n⚠️ {sin_marcar} corrida(s) sin marcar «válida» en la hoja: se tomaron como válidas.')
    if invalidas:
        p('\n**Inválidas (excluidas, con causa):** ' + '; '.join(
            f"orden {f.get('orden')}: {f.get('causa_invalida') or 'SIN CAUSA ANOTADA'}" for f in invalidas))
    p('\n## M1 · Tasa de misión exitosa (corridas)\n')
    p(f"**{m1['k']} de {m1['n']} = {fmt(m1['p'], 1, True)} %** · IC 95 % Clopper–Pearson "
      f"[{fmt(m1['cp'][0], 1, True)} %; {fmt(m1['cp'][1], 1, True)} %] · Wilson "
      f"[{fmt(m1['w'][0], 1, True)} %; {fmt(m1['w'][1], 1, True)} %]")
    p(f"\nPrueba binomial exacta unilateral, H₀: p ≤ {fmt(P0, 2)}: p = {m1['p_valor']:.4f} · límite inferior "
      f"unilateral 95 % = {fmt(m1['inf1'], 1, True)} % → **{'SE DEMUESTRA' if m1['demostrado'] else 'NO se demuestra'}"
      f" que la tasa supera el {fmt(P0, 0, True)} %**.")
    p(f'\nCorridas con intervención del operador: {intervenidas} · de ellas, con todos en FIN: {completas_con_interv} '
      '(se reportan aparte y no cuentan como éxito).')

    p(f"\nReferencia, **criterio literal del cliente** (todos los agentes, sin bajas, en su lugar antes del cierre): "
      f"{m1l['k']} de {m1l['n']} = {fmt(m1l['p'], 1, True)} % · IC 95 % CP [{fmt(m1l['cp'][0], 1, True)} %; {fmt(m1l['cp'][1], 1, True)} %].")
    if reclas:
        p(f"\n**Contando además las que el autor decidió dar por buenas** (órdenes {', '.join(map(str, reclas))}; "
          f"no cumplen el criterio fijado antes de la campaña y se declaran como tales): **{m1a['k']} de {m1a['n']} = "
          f"{fmt(m1a['p'], 1, True)} %** · IC 95 % CP [{fmt(m1a['cp'][0], 1, True)} %; {fmt(m1a['cp'][1], 1, True)} %] · "
          f"p (H₀: p ≤ {fmt(P0, 2)}) = {m1a['p_valor']:.4f}. Este número NO reemplaza a M1.")
    p('\n## Embudo por fase (corridas)\n')
    p('| Fase | Llegan | Completan | Condicional [IC 95 % CP] | Acumulada [IC 95 % CP] |')
    p('|---|---|---|---|---|')
    for t in tabla_fases:
        p(f"| {t['fase']} · {t['nombre']} | {t['llegan']} | {t['completan']} | {fmt(t['cond'], 1, True)} % "
          f"[{fmt(t['cond_ic'][0], 1, True)}; {fmt(t['cond_ic'][1], 1, True)}] | {fmt(t['acum'], 1, True)} % "
          f"[{fmt(t['acum_ic'][0], 1, True)}; {fmt(t['acum_ic'][1], 1, True)}] |")
    dv_desc = sum(1 for r in nominal if r['det_verdadera'] is None and r['t_det'] is not None)
    if dv_desc:
        p(f'\n⚠️ {dv_desc} detección(es) sin posición del objeto para verificar si es la verdadera: se contaron como verdaderas.')

    p('\n## M2 · Agentes (media de las fracciones por corrida; IC bootstrap por conglomerados)\n')
    p(f"Agentes exitosos: **{fmt(m2['media'], 1, True)} %** [IC 95 %: {fmt(m2['ic'][0], 1, True)} %; "
      f"{fmt(m2['ic'][1], 1, True)} %] · agrupado {m2['agrupada'][0]}/{m2['agrupada'][1]}")
    p('\n| Fase (agente) | Fracción media [IC 95 %] |')
    p('|---|---|')
    for k, nombre in (('buscar', 'llega a BUSCAR'), ('conoce', 'conoce el objetivo'), ('lleg', 'llega a la congregación'),
                      ('fin', 'llega a FIN'), ('exitoso', 'exitoso (FIN sin intervención)')):
        m, ic = agentes_fase[k]
        p(f'| {nombre} | {fmt(m, 1, True)} % [{fmt(ic[0], 1, True)}; {fmt(ic[1], 1, True)}] |')

    p('\n## Tiempos por fase (s, condicionales a que la fase se completó)\n')
    p(f'Tipo B por instante u = {fmt(U_T, 3)} s (diferencias: {fmt(math.sqrt(2) * U_T, 2)} s). U = 2·√(s²/n + u_B²).\n')
    p('| Tiempo | n | Mediana [IC 95 %] | Q1–Q3 | Mín–Máx | Media ± U (k = 2) |')
    p('|---|---|---|---|---|---|')
    nombres = dict(t_desp='Despliegue (último en BUSCAR)', t_det='Hasta la detección', t_dif='Difusión del hallazgo',
                   t_cong='Congregación (detección → cierre)', t_ret='Retorno (cierre → último FIN)',
                   t_mis='Misión completa')
    for k, nom in nombres.items():
        T = tiempos[k]
        if not T:
            p(f'| {nom} | 0 | — | — | — | — |')
            continue
        p(f"| {nom} | {T['n']} | {fmt(T['mediana'])} [{fmt(T['ic'][0])}; {fmt(T['ic'][1])}] | {fmt(T['q1'])}–{fmt(T['q3'])}"
          f" | {fmt(T['min'])}–{fmt(T['max'])} | {fmt(T['media'])} ± {fmt(T['U'])} |")
    p(f"\nKaplan–Meier (censura a {T_LIM:.0f} s o al aborto; incluye las que no terminaron): mediana hasta la detección "
      f"**{fmt(km['det'])} s** [{fmt(km_ic['det'][0])}; {fmt(km_ic['det'][1])}] · mediana de la misión "
      f"**{fmt(km['mis'])} s** [{fmt(km_ic['mis'][0])}; {fmt(km_ic['mis'][1])}] (IC por bootstrap; «—» = no se alcanza el 50 %).")

    p('\n## Cámara\n')
    p(f'Tipo B de una distancia entre dos poses: U ≈ {fmt(U_POS, 0)} mm (k = 2, σ ArUco {SIGMA_POS:.0f} mm/eje, preliminar).\n')
    p('| Magnitud | n | Mediana [IC 95 %] | Mín–Máx |')
    p('|---|---|---|---|')
    for k, nom in (('radio_ret_max', 'Radio máx. del enjambre en el retorno (mm, M9)'),
                   ('radio_ret_med', 'Radio mediano en el retorno (mm)'),
                   ('dmin', 'Distancia mínima entre robots en la corrida (mm)'),
                   ('dist_base_fin', 'Distancia a la base en FIN, mediana por corrida (mm)'),
                   ('cob_det', 'Celdas cubiertas al detectar')):
        T = camara[k]
        if T:
            p(f"| {nom} | {T['n']} | {fmt(T['mediana'], 0)} [{fmt(T['ic'][0], 0)}; {fmt(T['ic'][1], 0)}] | {fmt(T['min'], 0)}–{fmt(T['max'], 0)} |")
    cont = sum(r['contactos'] or 0 for r in nominal)
    p(f'\nAcercamientos a menos de un diámetro ({DIAM:.0f} mm) entre centros: {cont} en total '
      '(indicio de contacto; las colisiones se confirman con la hoja de corrida).')

    if comp:
        p('\n## Línea base: caminata aleatoria del firmware\n')
        p(f"Corridas: {comp['n_lb']} · con detección: {comp['det_lb']} · censura común a {T_LIM_LB:.0f} s.\n")
        p(f"Mediana Kaplan–Meier hasta la detección: BDI-E **{fmt(comp['med_b'])} s** · caminata aleatoria "
          f"**{fmt(comp['med_l'])} s** · razón {fmt(comp['razon'], 2)} [IC 95 %: {fmt(comp['razon_ic'][0], 2)}; "
          f"{fmt(comp['razon_ic'][1], 2)}]")
        p(f"\nLog-rank: χ² = {fmt(comp['chi2'], 2)}, p = {fmt(comp['p_lr'], 3)} · Mann–Whitney (solo detectadas, "
          f"aprox. normal): U = {fmt(comp['U'], 1)}, p = {fmt(comp['p_mw'], 3)}")
    else:
        p('\n## Línea base\n\nSin corridas de línea base con log todavía.')

    p('\n## Deriva contra el orden de las corridas (Spearman, p por permutación)\n')
    for k, nom in (('exito', 'misión exitosa'), ('t_det', 'tiempo hasta la detección'), ('t_mis', 'duración de la misión')):
        if k in deriva:
            rho, pv = deriva[k]
            p(f'- {nom}: ρ = {fmt(rho, 2)}, p = {fmt(pv, 3)}')
    p('\n---\nDefiniciones, reglas y su justificación: `Indicadores/PLAN_Analisis_Resultados.md`. '
      'Detalle por corrida y por agente: `por_corrida.csv`, `por_agente.csv`.')
    open(os.path.join(a.salida, 'resumen.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')

    # ── tabla LaTeX del embudo ──
    pc = lambda x: fmt(x, 1, True).replace('—', '---')
    tex = [r'\begin{table}[htbp]', r'  \centering', r'  \footnotesize',
           r'  \caption{Tasa por fase de la misión sobre el enjambre físico (' + str(n) +
           r' corridas nominales; intervalos de confianza del 95\,\% de Clopper--Pearson).}',
           r'  \label{tab:res_fases}', r'  \begin{tabularx}{\textwidth}{@{}Xcccc@{}}', r'    \toprule',
           r'    \textbf{Fase} & \textbf{Llegan} & \textbf{Completan} & \textbf{Condicional (\%)} & \textbf{Acumulada (\%)} \\',
           r'    \midrule']
    for t in tabla_fases:
        tex.append(f"    {t['nombre']} & {t['llegan']} & {t['completan']} & {pc(t['cond'])} "
                   f"[{pc(t['cond_ic'][0])}; {pc(t['cond_ic'][1])}] & {pc(t['acum'])} [{pc(t['acum_ic'][0])}; {pc(t['acum_ic'][1])}] \\\\")
    tex += [r'    \bottomrule', r'  \end{tabularx}', r'\end{table}']
    open(os.path.join(a.salida, 'tabla_fases.tex'), 'w', encoding='utf-8').write(
        '% Generado por resumen_campana.py — NO editar a mano; volver a correr el script.\n'
        '% Requiere usar coma decimal (\\usepackage{icomma} o similar) si se cambia el formato.\n' + '\n'.join(tex) + '\n')

    # ── figuras ──
    figs = []
    if not a.sin_figuras:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            # embudo
            fig, ax = plt.subplots(figsize=(6.3, 3.4))
            x = range(len(tabla_fases))
            ys = [t['acum'] * 100 for t in tabla_fases]
            err = [[y - t['acum_ic'][0] * 100 for y, t in zip(ys, tabla_fases)],
                   [t['acum_ic'][1] * 100 - y for y, t in zip(ys, tabla_fases)]]
            ax.bar(x, ys, color='#4C6A92', width=0.6, label='corridas (acumulada)')
            ax.errorbar(x, ys, yerr=err, fmt='none', ecolor='#222', capsize=4, lw=1)
            ya = [agentes_fase[k][0] * 100 for k in ('buscar', 'conoce', 'conoce', 'lleg', 'fin')]
            ax.plot(x, ya, 'o--', color='#C0703A', label='agentes (media por corrida)')
            ax.axhline(P0 * 100, color='#888', lw=0.8, ls=':')
            ax.set_xticks(list(x))
            ax.set_xticklabels(['Despliegue', 'Detección', 'Difusión', 'Congregación', 'Retorno'], fontsize=8)
            ax.set_ylabel('%')
            ax.set_ylim(0, 105)
            ax.legend(fontsize=8, loc='lower left', frameon=False)
            ax.spines[['top', 'right']].set_visible(False)
            fig.tight_layout()
            fig.savefig(os.path.join(a.salida, 'fig_embudo.pdf'))
            plt.close(fig)
            figs.append('fig_embudo.pdf')

            # Kaplan–Meier de la detección
            fig, ax = plt.subplots(figsize=(6.3, 3.2))

            def escalon(tt, ee, lab, col):
                c = kaplan_meier(tt, ee)
                xs, ys2 = [0], [1.0]
                for t, s in c[1:]:
                    xs += [t, t]
                    ys2 += [ys2[-1], s]
                xs.append(max(tt))
                ys2.append(ys2[-1])
                ax.plot(xs, [1 - v for v in ys2], color=col, label=lab)

            escalon([t for t, _ in km_det], [e for _, e in km_det], 'BDI-E', '#4C6A92')
            if lb_res:
                escalon([r['km'][0] for r in lb_res], [r['km'][1] for r in lb_res], 'caminata aleatoria', '#C0703A')
            ax.set_xlabel('tiempo desde el inicio (s)')
            ax.set_ylabel('fracción de corridas con detección')
            ax.set_ylim(0, 1.02)
            ax.legend(fontsize=8, frameon=False)
            ax.spines[['top', 'right']].set_visible(False)
            fig.tight_layout()
            fig.savefig(os.path.join(a.salida, 'fig_km_deteccion.pdf'))
            plt.close(fig)
            figs.append('fig_km_deteccion.pdf')

            # tiempos por fase
            fig, ax = plt.subplots(figsize=(6.3, 3.2))
            ks = ['t_det', 't_cong', 't_ret', 't_mis']
            datos = [[r[k] for r in nominal if r[k] is not None] for k in ks]
            ax.boxplot([d if d else [float('nan')] for d in datos], widths=0.5, showfliers=False)
            for i, d in enumerate(datos, 1):
                ax.plot([i + rng.uniform(-0.12, 0.12) for _ in d], d, 'o', ms=3.5, color='#4C6A92', alpha=0.8)
            ax.set_xticks(range(1, len(ks) + 1))
            ax.set_xticklabels(['Detección', 'Congregación', 'Retorno', 'Misión'], fontsize=8)
            ax.set_ylabel('s')
            ax.spines[['top', 'right']].set_visible(False)
            fig.tight_layout()
            fig.savefig(os.path.join(a.salida, 'fig_tiempos_fase.pdf'))
            plt.close(fig)
            figs.append('fig_tiempos_fase.pdf')

            # deriva
            fig, ax = plt.subplots(figsize=(6.3, 2.8))
            for r in nominal:
                ax.plot(r['orden'], r['t_mis'] if r['t_mis'] is not None else T_LIM,
                        'o' if r['mision_exitosa'] else 'x', color='#4C6A92' if r['mision_exitosa'] else '#B03030')
            ax.set_xlabel('orden de la corrida')
            ax.set_ylabel('duración (s)')
            ax.spines[['top', 'right']].set_visible(False)
            fig.tight_layout()
            fig.savefig(os.path.join(a.salida, 'fig_orden.pdf'))
            plt.close(fig)
            figs.append('fig_orden.pdf')
        except ImportError:
            print('(sin matplotlib: no se generan figuras)')

    print(open(os.path.join(a.salida, 'resumen.md'), encoding='utf-8').read())
    print(f'Salidas en {a.salida}: resumen.md, por_corrida.csv, por_agente.csv, tabla_fases.tex'
          + (', ' + ', '.join(figs) if figs else ''))


if __name__ == '__main__':
    main()
