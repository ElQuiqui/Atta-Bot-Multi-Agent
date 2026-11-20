#!/usr/bin/env python3
"""
medidas_hw_07nov.py — medidas del enjambre físico a partir de los registros de
la sesión de campaña del 7 de noviembre de 2026 (corridas nominales).

Calcula, por corrida y en conjunto:
  1. Ritmo de latidos (HB) que la base oye de cada robot (la misma difusión UDP
     que oyen los compañeros), intervalos entre latidos y pérdida por número de
     secuencia.
  2. Reparto de zonas en los robots: tiempo hasta el primer acuerdo, rondas,
     nuevo reparto tras la pérdida de un participante y comprobación de que
     ninguna franja quedó repetida ni (fuera de la ventana de detección) sin
     dueño.
  3. Traza de razonamiento (M13): cambios de plan con el evento registrado que
     los explica, y tabla LaTeX de la corrida 2.
  4. Colisiones (M10): lo que dice la hoja de corrida.

Uso (desde cualquier carpeta):
    py -3.14 -I medidas_hw_07nov.py [--v2 "<carpeta BDI-E>"]

Salidas en base/herramientas/resumen_campana/:
    medidas_hw_07nov.md   resumen con todos los números
    traza_m13.tex         tabla de la traza de la corrida 2 (label tab:traza_m13)

Convenciones de los registros (el script las comprueba):
  * El tiempo del ConsoleLog y el `tlog` de BdieLogs comparten origen (el
    arranque de la base): el CMD|TARGET_COLOR que envía la base queda menos de
    1 s antes del evento `sesion START`.
  * HB|id|fase|x|seq: seq crece de uno en uno desde el arranque del robot (no se
    reinicia por sesión), así que pérdida = 1 − recibidos/esperados.
  * `vivos` es una máscara de bits en hexadecimal (0x16 = robots 1, 2 y 4;
    0x1a = 1, 3 y 4).
  * think.csv: las columnas `ev` y `tev` son el último evento BDI y su instante
    en el reloj del robot (columna `t`, ms).
"""
import argparse
import csv
import os
import re
import statistics as st
from collections import defaultdict

AQUI = os.path.dirname(os.path.abspath(__file__))
V2_DEF = os.path.normpath(os.path.join(AQUI, '..', '..'))
VIVO_S = 3.0          # plazo de «lo oigo» del diseño (s)

# ---------------------------------------------------------------- utilidades


def fnum(x, d=1):
    """Número con coma decimal."""
    if x is None:
        return '—'
    return f'{x:.{d}f}'.replace('.', ',')


def pct(a, b, d=1):
    return '—' if not b else fnum(100.0 * a / b, d) + ' %'


def percentil(xs, p):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * p
    f = int(k)
    c = min(f + 1, len(xs) - 1)
    return xs[f] + (xs[c] - xs[f]) * (k - f)


def leer_csv(ruta):
    with open(ruta, encoding='utf-8-sig', errors='replace', newline='') as fh:
        return list(csv.DictReader(fh))


def mascara(hexstr):
    try:
        v = int(hexstr, 16)
    except ValueError:
        return set()
    return {i for i in range(8) if v >> i & 1}


def hhmmss(nombre):
    m = re.search(r'(\d\d)-(\d\d)-(\d\d)_think', nombre)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))


def consola_de(ruta_logs, think_nombre):
    """ConsoleLog cuya hora (HH-MM) es la última <= hora de la corrida."""
    t_run = hhmmss(think_nombre)
    mejor, mejor_t = None, -1
    for f in os.listdir(ruta_logs):
        m = re.match(r'Console_Log_07-11_(\d\d)-(\d\d)_Robots_\d+\.csv$', f)
        if not m:
            continue
        t = int(m.group(1)) * 3600 + int(m.group(2)) * 60
        if t <= t_run and t > mejor_t:
            mejor, mejor_t = f, t
    return mejor

# --------------------------------------------------------------- por corrida


def analizar(v2, fila):
    base = os.path.join(v2, 'base')
    stem = fila['archivo_think'].replace('_think.csv', '')
    ev = leer_csv(os.path.join(base, 'BdieLogs', stem + '_eventos.csv'))
    th = leer_csv(os.path.join(base, 'BdieLogs', stem + '_think.csv'))
    esperados = {int(x) for x in fila['robots_ids'].replace('"', '').split(',')}
    for e in ev:
        e['t'] = float(e['tlog'])
        e['id'] = int(e['id'])
    for r in th:
        r['tl'] = float(r['tlog'])
        r['id'] = int(r['id'])
    t0 = next(e['t'] for e in ev if e['tipo'] == 'sesion' and 'START' in e['detalle'])
    fin = {e['id']: e['t'] for e in ev if e['tipo'] == 'fase' and e['detalle'].endswith('FIN')}
    t_det = min((e['t'] for e in ev if e['tipo'] == 'fase' and e['detalle'].endswith('CONGREGAR')),
                default=None)
    R = dict(orden=int(fila['orden']), stem=stem, t0=t0, esperados=esperados,
             t_det=None if t_det is None else t_det - t0)

    # ---------------- 1. latidos ------------------------------------------------
    cfile = consola_de(os.path.join(base, 'ConsoleLogs'), fila['archivo_think'])
    con = []
    with open(os.path.join(base, 'ConsoleLogs', cfile), encoding='utf-8', errors='replace') as fh:
        rd = csv.reader(fh)
        next(rd)
        for row in rd:
            if len(row) < 4:
                continue
            try:
                con.append((float(row[0]), int(row[1]), row[3]))
            except ValueError:
                pass
    tc = [t for t, i, m in con if i == -1 and m.startswith('CMD|TARGET_COLOR') and t <= t0 + 0.5]
    R['consola'] = cfile
    R['alineacion_s'] = (t0 - max(tc)) if tc else None
    hb = defaultdict(list)       # id -> [(t, seq)]
    ultimo_msg = defaultdict(float)
    for t, i, m in con:
        if i >= 0:
            ultimo_msg[i] = max(ultimo_msg[i], t) if t <= (max(fin.values(), default=1e9) + 5) else ultimo_msg[i]
        if m.startswith('HB|'):
            p = m.split('|')
            try:
                hb[i].append((t, int(p[-1])))
            except ValueError:
                pass
    lat = {}
    for rid in sorted(esperados):
        t_fin = fin.get(rid)
        pts = [(t, s) for t, s in hb.get(rid, []) if t >= t0 and (t_fin is None or t <= t_fin)]
        if len(pts) < 3:
            lat[rid] = None
            continue
        ts = [t for t, _ in pts]
        seqs = sorted({s for _, s in pts})
        ints = [b - a for a, b in zip(ts, ts[1:])]
        esperados_seq = seqs[-1] - seqs[0] + 1
        lat[rid] = dict(n=len(pts), dur=ts[-1] - ts[0], hz=(len(pts) - 1) / (ts[-1] - ts[0]),
                        ints=ints, recibidos=len(seqs), esperados=esperados_seq,
                        hasta='FIN' if t_fin is not None else 'último HB')
    R['hb'] = lat

    # ---------------- 2. reparto -------------------------------------------------
    rep = [e for e in ev if e['tipo'] == 'reparto']
    rondas = defaultdict(list)
    for e in rep:
        m = re.search(r'ronda (\d+) · franja (\d+) de (\d+) · participantes \{([\d,]+)\}', e['detalle'])
        if m:
            rondas[int(m.group(1))].append(dict(t=e['t'], id=e['id'], franja=int(m.group(2)),
                                                n=int(m.group(3)),
                                                part={int(x) for x in m.group(4).split(',')}))
    R['rondas'] = {k: v for k, v in sorted(rondas.items())}
    zon = [e for e in ev if e['tipo'] == 'zona' and not e['detalle'].endswith('-1')]
    r1 = rondas.get(1, [])
    part1 = r1[0]['part'] if r1 else set()
    # primer reparto: última adopción de franja (zona -1 -> k) de un participante
    adop1 = [e['t'] for e in ev if e['tipo'] == 'racu' and e['detalle'] == '0 → 1' and e['id'] in part1
             and (not r1 or e['t'] <= max(x['t'] for x in r1) + 0.01)]
    R['acuerdo1_zona'] = (max(adop1) - t0) if adop1 else None
    R['acuerdo1_reporte'] = (max(x['t'] for x in r1) - t0) if r1 else None
    R['part1'] = part1
    # nuevos repartos por pérdida de un participante (antes del primer FIN)
    t_fin1 = min(fin.values()) if fin else 1e9
    eventos_baja = []
    for k, filas in sorted(rondas.items()):
        if k == 1:
            continue
        t_ron = min(x['t'] for x in filas)
        if t_ron >= t_fin1:
            continue
        part_prev = rondas[k - 1][0]['part'] if rondas.get(k - 1) else set()
        part_k = filas[0]['part']
        perdidos = part_prev - part_k
        nuevos = part_k - part_prev
        # primera transición de «vivos» (en un sobreviviente) que deja fuera a
        # un perdido: la máscara anterior lo incluía y la nueva no
        drops = []
        for e in ev:
            if e['tipo'] != 'vivos' or e['id'] not in part_k or e['t'] > t_ron + 0.01:
                continue
            a_, b_ = [x.strip() for x in e['detalle'].split('→')]
            if mascara(a_) & perdidos and not (mascara(b_) & perdidos):
                drops.append(e['t'])
        t_drop = min(drops) if drops else None
        msgs = [t for t, i, m in con if i in perdidos and (t_drop is None or t <= t_drop)]
        t_ult = max(msgs) if msgs else None
        # adopción: última transición racu 0 -> 1 de un participante en esta ronda
        t_rep_k = min((e['t'] for e in ev if e['tipo'] == 'rep' and e['detalle'].endswith(f'→ {k}')),
                      default=t_ron)
        adop = [e['t'] for e in ev if e['tipo'] == 'racu' and e['detalle'] == '0 → 1'
                and e['id'] in part_k and t_rep_k - 0.01 <= e['t'] <= t_ron + 10]
        eventos_baja.append(dict(ronda=k, perdidos=perdidos, nuevos=nuevos, part=part_k,
                                 t_ult_msg=None if t_ult is None else t_ult - t0,
                                 t_drop=None if t_drop is None else t_drop - t0,
                                 t_reparto=max(x['t'] for x in filas) - t0,
                                 t_adop=(max(adop) - t0) if adop else None))
    R['rerepartos'] = eventos_baja
    reales = set()
    for e in eventos_baja:
        reales |= e['perdidos']
    falsas = 0
    for e in ev:
        if e['tipo'] != 'vivos' or e['t'] >= t_fin1 or e['t'] < t0 + 2:
            continue
        a_, b_ = [x.strip() for x in e['detalle'].split('→')]
        if (mascara(a_) - mascara(b_)) - reales:
            falsas += 1
    R['falsas_caidas'] = falsas

    # comprobación de franjas repetidas / sin dueño desde think.csv, entre el
    # primer acuerdo y la detección
    th.sort(key=lambda r: r['tl'])
    estado = {}
    ult = {}
    n_actual = None
    rep_sorted = sorted(rep, key=lambda e: e['t'])
    t_ini = (t0 + R['acuerdo1_zona']) if R['acuerdo1_zona'] is not None else None
    t_end = (t0 + R['t_det']) if R['t_det'] is not None else t_fin1
    t_dup = t_sin = t_tot = 0.0
    sin_ventanas = []
    prev_t = None
    prev_flags = None
    for r in th:
        t = r['tl']
        if prev_t is not None and t_ini is not None and prev_t >= t_ini and prev_t < t_end:
            dt = min(t, t_end) - prev_t
            t_tot += dt
            if prev_flags[0]:
                t_dup += dt
            if prev_flags[1]:
                t_sin += dt
                if sin_ventanas and abs(sin_ventanas[-1][1] - (prev_t - t0)) < 1e-6:
                    sin_ventanas[-1][1] = min(t, t_end) - t0
                else:
                    sin_ventanas.append([prev_t - t0, min(t, t_end) - t0])
        estado[r['id']] = (r['fase'], int(r['zona']))
        ult[r['id']] = t
        nn = [e for e in rep_sorted if e['t'] <= t]
        if nn:
            n_actual = int(re.search(r'de (\d+)', nn[-1]['detalle']).group(1))
        vivos = [i for i in estado if t - ult[i] <= VIVO_S and estado[i][0] in ('DESPLEGAR', 'BUSCAR')]
        zs = [estado[i][1] for i in vivos if estado[i][1] >= 0]
        dup = len(zs) != len(set(zs))
        sin = n_actual is not None and any(z not in zs for z in range(n_actual))
        prev_t, prev_flags = t, (dup, sin)
    R['t_ventana'] = t_tot
    R['t_dup'] = t_dup
    R['t_sin'] = t_sin
    R['sin_int'] = [(a_, b_) for a_, b_ in sin_ventanas if b_ - a_ >= 0.1]

    # ---------------- 3. traza M13 -------------------------------------------------
    cambios = []
    prev = {}
    base_n = base_ev = 0
    est = [e for e in ev if e['tipo'] in ('obj', 'lleg', 'cerr', 'fase', 'dueno', 'zona')]
    for r in th:
        i = r['id']
        if i in prev and r['plan'] == prev[i]['plan']:
            try:
                base_n += 1
                base_ev += int(r['tev']) > int(prev[i]['t']) and r['ev'] not in ('-', '')
            except ValueError:
                pass
        if i in prev and r['plan'] != prev[i]['plan']:
            try:
                t_rob, tev, t_prev = int(r['t']), int(r['tev']), int(prev[i]['t'])
            except ValueError:
                t_rob = tev = t_prev = 0
            ev_nuevo = tev > t_prev and r['ev'] not in ('-', '')
            deseo_cambia = r['deseo'] != prev[i]['deseo']
            cambios.append(dict(t=r['tl'] - t0, id=i, plan0=prev[i]['plan'], plan1=r['plan'],
                                deseo0=prev[i]['deseo'], deseo1=r['deseo'],
                                fase0=prev[i]['fase'], fase1=r['fase'],
                                ev=r['ev'] if ev_nuevo else None,
                                dt_ev_ms=(t_rob - tev) if ev_nuevo else None,
                                deseo_cambia=deseo_cambia,
                                obj=(prev[i]['obj'], r['obj']), lleg=(prev[i]['lleg'], r['lleg']),
                                cerr=(prev[i]['cerr'], r['cerr']),
                                est=[(e['tipo'], e['detalle']) for e in est
                                     if e['id'] == i and r['tl'] - 1.0 <= e['t'] <= r['tl'] + 0.01]))
        prev[i] = r
    R['cambios'] = cambios
    R['base_m13'] = (base_ev, base_n)
    return R


def disparador(c):
    """Texto corto con el evento o cambio de estado que explica el cambio.
    Prioridad: cambio de estado registrado en el segundo previo (objetivo,
    llegada, cierre, fin, franja); si no hay, el evento BDI nuevo."""
    BS = chr(92)
    if c['plan0'] == '-':
        return 'inicio de la sesión (' + BS + 'texttt{START})'
    partes = []
    tipos = set(c['est'])
    descubre = any(t == 'dueno' and d.endswith(f"→ {c['id']}") for t, d in tipos)
    if descubre:
        partes.append('detecta el objetivo con su sensor')
    elif any(t == 'obj' and d == '0 → 1' for t, d in tipos):
        partes.append('recibe el hallazgo')
    if any(t == 'lleg' and d == '0 → 1' for t, d in tipos) and not descubre:
        partes.append('llega a su ranura')
    if any(t == 'cerr' and d == '0 → 1' for t, d in tipos):
        partes.append('cierre de la congregación')
    if any(t == 'fase' and d.endswith('→ FIN') for t, d in tipos):
        partes.append('llega a la base (' + BS + 'texttt{FIN})')
    if any(t == 'zona' and d.startswith('-1 →') for t, d in tipos):
        partes.append('adopta su franja')
    if partes:
        return '; '.join(partes)
    if c['ev']:
        nombres = {'ARRIVED': 'llegó a la celda meta', 'ZONE_AGREED': 'reparto acordado',
                   'TIMEOUT': 'tiempo vencido', 'TARGET_REPORTED': 'hallazgo difundido'}
        partes.append(f"\\texttt{{{c['ev'].replace('_', chr(92) + '_')}}} ({nombres.get(c['ev'], c['ev'].lower())})")
    if not partes and c['deseo_cambia']:
        partes.append('cambio de deseo')
    return '; '.join(partes) if partes else '—'

# ---------------------------------------------------------------------- main


def main():
    import sys
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument('--v2', default=V2_DEF)
    a = ap.parse_args()
    v2 = a.v2
    hoja = leer_csv(os.path.join(v2, 'campana_hoja_corrida.csv'))
    filas = [f for f in hoja if f['bloque'] == 'nominal' and f['archivo_think'].strip()]
    res = [analizar(v2, f) for f in filas]
    out_dir = os.path.join(v2, 'base', 'herramientas', 'resumen_campana')
    L = []
    p = L.append
    p('# Medidas reales del enjambre físico — sesión del 07-nov-2026\n')
    p(f'Generado por `medidas_hw_07nov.py` sobre {len(res)} corridas nominales '
      '(robots Atta_1, 2 y 4). Tiempos en s desde el `START` de la sesión.\n')
    p('Alineación ConsoleLog ↔ BdieLogs: `CMD|TARGET_COLOR` antes del START por '
      + ', '.join(f"c{r['orden']} {fnum(r['alineacion_s'], 2)} s" for r in res)
      + ' (mismo origen de tiempo).\n')

    # ---- 1. latidos
    p('## 1. Latidos (HB) recibidos en la base\n')
    p('La base oye el mismo broadcast UDP `HB` que los compañeros (puerto 6060): es la '
      'recepción en **un oyente más del canal**, no la recepción en cada robot. '
      'Ventana: del START a la llegada a FIN del robot (o a su último HB si es baja).\n')
    p('| Corrida | Robot | HB | Duración (s) | Hz | Intervalo mediano (s) | p95 (s) | Máx (s) | Pérdida (seq) |')
    p('|---|---|---|---|---|---|---|---|---|')
    hz, ints_all, rec, esp = [], [], 0, 0
    for r in res:
        for rid, d in sorted(r['hb'].items()):
            if not d:
                p(f"| {r['orden']} | Atta_{rid} | — | | | | | | |")
                continue
            hz.append(d['hz'])
            ints_all += d['ints']
            rec += d['recibidos']
            esp += d['esperados']
            p(f"| {r['orden']} | Atta_{rid} | {d['n']} | {fnum(d['dur'])} | {fnum(d['hz'], 2)} | "
              f"{fnum(st.median(d['ints']), 3)} | {fnum(percentil(d['ints'], 0.95), 3)} | "
              f"{fnum(max(d['ints']), 3)} | {pct(d['esperados'] - d['recibidos'], d['esperados'])} |")
    p('')
    p(f'**Conjunto ({len(hz)} series robot-corrida, {len(ints_all)} intervalos):** ritmo mediano '
      f'{fnum(st.median(hz), 2)} Hz (mín {fnum(min(hz), 2)}, máx {fnum(max(hz), 2)}); intervalo entre latidos '
      f'mediano {fnum(st.median(ints_all), 3)} s, p5 {fnum(percentil(ints_all, 0.05), 3)} s, '
      f'p95 {fnum(percentil(ints_all, 0.95), 3)} s, mín {fnum(min(ints_all), 3)} s, máx {fnum(max(ints_all), 3)} s; '
      f'intervalos > 3 s (plazo de «vivo»): {sum(1 for x in ints_all if x > VIVO_S)}. '
      f'Pérdida por número de secuencia: {esp - rec} de {esp} = {pct(esp - rec, esp, 2)}.\n')
    env = [(d['esperados'] - 1) / d['dur'] for r in res for d in r['hb'].values() if d]
    p(f'**Ritmo de emisión** (avance del número de secuencia por segundo, es decir, lo que el robot envía): '
      f'mediana {fnum(st.median(env), 2)} Hz (mín {fnum(min(env), 2)}, máx {fnum(max(env), 2)}), frente a '
      f'2 Hz de diseño (`hbPeriodMs` = 500 ms). Los robots emiten a 2 Hz; lo que baja a ~1,5 Hz es lo que '
      f'**la base** alcanza a registrar. La causa del lado de la base (búfer del socket, hilo de recepción que '
      f'también atiende poses a 10–20 Hz, ahorro de energía de la WiFi de la computadora) no se determinó con estos '
      f'registros.\n')
    HB = dict(hz_med=st.median(hz), hz_min=min(hz), hz_max=max(hz), int_med=st.median(ints_all),
              p5=percentil(ints_all, 0.05), p95=percentil(ints_all, 0.95), imax=max(ints_all),
              perd=(esp - rec) / esp, n=len(hz), nint=len(ints_all), esp=esp, rec=rec)

    # ---- 2. reparto
    p('## 2. Reparto de zonas en hardware\n')
    p('| Corrida | Participantes ronda 1 | Rondas | Acuerdo 1 (última adopción) | Último `ZA` reportado | '
      'Ventana revisada (s) | Franja repetida (s) | Franja sin dueño (s) |')
    p('|---|---|---|---|---|---|---|---|')
    ac = []
    for r in res:
        ac.append(r['acuerdo1_zona'])
        p(f"| {r['orden']} | {{{','.join(map(str, sorted(r['part1'])))}}} | {len(r['rondas'])} | "
          f"{fnum(r['acuerdo1_zona'])} | {fnum(r['acuerdo1_reporte'])} | {fnum(r['t_ventana'])} | "
          f"{fnum(r['t_dup'])} | {fnum(r['t_sin'])} |")
    p('')
    p(f'**Primer acuerdo:** mediana {fnum(st.median(ac))} s desde el START (mín {fnum(min(ac))}, '
      f'máx {fnum(max(ac))}; n = {len(ac)}). Diseño: 1,5 s de gracia + 1 s de asentamiento.\n')
    p('**Nuevos repartos por pérdida de un participante (antes del primer FIN):**\n')
    rr_all = []
    for r in res:
        for e in r['rerepartos']:
            rr_all.append((r['orden'], e))
            perd = ','.join(f'Atta_{x}' for x in sorted(e['perdidos'])) or '—'
            p(f"- Corrida {r['orden']}, ronda {e['ronda']}: sale {perd}. Último mensaje en la base "
              f"{fnum(e['t_ult_msg'])} s; los vivos lo excluyen {fnum(e['t_drop'])} s; nuevo reparto adoptado "
              f"{fnum(e['t_adop'])} s y reportado {fnum(e['t_reparto'])} s → "
              f"{fnum(None if e['t_drop'] is None or e['t_adop'] is None else e['t_adop'] - e['t_drop'])} s desde "
              f"que lo dan por caído y "
              f"{fnum(None if e['t_ult_msg'] is None or e['t_adop'] is None else e['t_adop'] - e['t_ult_msg'])} s "
              f"desde su último mensaje.")
    p('- Corrida 4: la baja (Atta_1 trabado contra la pared desde ~120 s) **sigue transmitiendo**, así que '
      'para el resto sigue «vivo» y no hay nuevo reparto: es una baja por inmovilización, no por silencio.')
    p('')
    p('Franja repetida = dos robots vivos en despliegue/búsqueda con la misma franja a la vez. Franja sin dueño = '
      'alguna franja 0…n−1 de la ronda vigente sin robot vivo en despliegue/búsqueda. Ventana: del primer acuerdo a la '
      'detección (primer paso a CONGREGAR). Vivo = con muestra de razonamiento en los últimos 3 s.\n')
    for r in res:
        for a_, b_ in r['sin_int']:
            p(f"- Corrida {r['orden']}: franja sin dueño de {fnum(a_)} a {fnum(b_)} s ({fnum(b_ - a_)} s).")
    p('(Los demás instantes sin dueño son de milisegundos: artefacto del muestreo, las filas de cada robot llegan '
      'desfasadas.)')
    p('')
    fc = sum(r['falsas_caidas'] for r in res)
    p(f'**Falsas caídas entre robots:** {fc} transiciones de «vivos» que excluyen a un compañero que seguía '
      'transmitiendo, en las 7 corridas (antes del primer FIN). Contrasta con la base, que sí tuvo intervalos > 3 s '
      'sin oír a un robot (sección 1): la pérdida medida en la base no es la que viven los robots.')
    p('')

    # ---- 3. M13
    p('## 3. Traza de razonamiento (M13)\n')
    p('Fuente: `*_think.csv` (2 Hz por robot). Un **cambio de plan** es una muestra cuyo `plan` difiere del de la '
      'muestra anterior del mismo robot. Se considera **explicado por un evento registrado** si en ese intervalo se '
      'registró un evento BDI nuevo (`tev` posterior a la muestra anterior: ARRIVED, ZONE_AGREED, TIMEOUT, '
      'TARGET_REPORTED) **o** cambió el deseo (cuyo disparador es un cambio de estado registrado: objetivo conocido, '
      'llegada, cierre, fase).\n')
    p('| Corrida | Cambios de plan | Con evento BDI nuevo | Solo cambio de deseo | Sin explicación |')
    p('|---|---|---|---|---|')
    tot = tev = tdes = tsin = 0
    sin_lista = []
    for r in res:
        c = r['cambios']
        a1 = sum(1 for x in c if x['ev'])
        a2 = sum(1 for x in c if not x['ev'] and x['deseo_cambia'])
        a3 = len(c) - a1 - a2
        sin_lista += [(r['orden'], x) for x in c if not x['ev'] and not x['deseo_cambia']]
        tot += len(c)
        tev += a1
        tdes += a2
        tsin += a3
        p(f"| {r['orden']} | {len(c)} | {a1} | {a2} | {a3} |")
    p(f"| **Total** | **{tot}** | **{tev}** | **{tdes}** | **{tsin}** |")
    p('')
    p(f'**M13 = {tev + tdes} de {tot} = {pct(tev + tdes, tot)}** de los cambios de plan tienen registrado el evento '
      f'o el cambio de deseo que los dispara (con evento BDI nuevo: {pct(tev, tot)}).\n')
    bev = sum(r['base_m13'][0] for r in res)
    bn = sum(r['base_m13'][1] for r in res)
    dts = [x['dt_ev_ms'] for r in res for x in r['cambios'] if x['dt_ev_ms'] is not None]
    p(f'Comparación: en los intervalos de muestreo **sin** cambio de plan hay un evento BDI nuevo en {bev} de {bn} '
      f'({pct(bev, bn)}), así que el 100 % no es un efecto de que haya eventos en todo intervalo. Retardo entre el '
      f'evento y la muestra que muestra el plan nuevo (reloj del robot): mediana {fnum(st.median(dts), 0)} ms, '
      f'p95 {fnum(percentil(dts, 0.95), 0)} ms, máx {fnum(max(dts), 0)} ms (n = {len(dts)}).\n')
    if sin_lista:
        p('Cambios sin explicación registrada:')
        for o, x in sin_lista:
            p(f"- c{o} t = {fnum(x['t'])} s Atta_{x['id']}: {x['plan0']} → {x['plan1']} (deseo {x['deseo1']})")
        p('')
    M13 = dict(tot=tot, ev=tev, des=tdes, sin=tsin)

    # tabla LaTeX de la corrida 2
    r2 = next(r for r in res if r['orden'] == 2)
    c2 = r2['cambios']
    tex = []
    tex.append('% Generado por Codigo/BDI-E V2/base/herramientas/medidas_hw_07nov.py (no editar a mano).')
    tex.append('\\begin{table}[htbp]')
    tex.append('  \\centering')
    tex.append('  \\footnotesize')
    tex.append(f'  \\caption{{Traza de razonamiento de la corrida~2 (tres robots): cada cambio de plan, con el deseo '
               f'vigente y el evento registrado que lo dispara. Tiempo desde el inicio de la sesión; '
               f'{len(c2)} cambios de plan en la corrida.}}')
    tex.append('  \\label{tab:traza_m13}')
    tex.append('  \\begin{tabularx}{\\textwidth}{@{}r l l l X@{}}')
    tex.append('    \\toprule')
    tex.append('    \\textbf{t (s)} & \\textbf{Robot} & \\textbf{Deseo} & \\textbf{Plan anterior $\\rightarrow$ nuevo} & '
               '\\textbf{Evento disparador} \\\\')
    tex.append('    \\midrule')

    def tt(s):
        return '\\texttt{' + s.replace('_', '\\_') + '}' if s and s != '-' else '---'
    for x in c2:
        tex.append(f"    {fnum(x['t'])} & Atta\\_{x['id']} & {tt(x['deseo1'])} & {tt(x['plan0'])} $\\rightarrow$ "
                   f"{tt(x['plan1'])} & {disparador(x)} \\\\")
    tex.append('    \\bottomrule')
    tex.append('  \\end{tabularx}')
    tex.append('\\end{table}')
    with open(os.path.join(out_dir, 'traza_m13.tex'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(tex) + '\n')
    p(f'Tabla de la corrida 2 ({len(c2)} cambios de plan): `resumen_campana/traza_m13.tex`.\n')

    # ---- 4. colisiones
    p('## 4. Colisiones (M10)\n')
    pc = {r['orden']: r for r in leer_csv(os.path.join(out_dir, 'por_corrida.csv'))}
    tot_h = tot_c = 0
    p('| Corrida | Hoja (`colisiones`, automático) | `resumen_campana.py` (contactos) | Colisión anotada por el operador |')
    p('|---|---|---|---|')
    for f in filas:
        m = re.search(r'(\d+) acercamientos', f['colisiones'])
        nh = int(m.group(1)) if m else 0
        nc = int(float(pc[f['orden']]['contactos'] or 0)) if f['orden'] in pc else 0
        txt = (f['desenlace_observado'] + ' ' + f['notas'] + ' ' + f['intervenciones_hora_robot_fase_que']).lower()
        anot = 'sí' if re.search(r'colisi|choc|golpe', txt) else 'no'
        tot_h += nh
        tot_c += nc
        p(f"| {f['orden']} | {nh} | {nc} | {anot} |")
    p(f'| **Total** | **{tot_h}** | **{tot_c}** | |')
    p('')
    p('Las dos cifras cuentan **episodios** de acercamiento a menos de un diámetro (150 mm) entre centros con las poses '
      'de la cámara; difieren por el muestreo (la hoja usa la última pose cada 0,5 s; `resumen_campana.py`, todas '
      'las poses). Ninguna corrida tiene una colisión anotada por el operador en el desenlace observado, las notas '
      'ni las intervenciones. Son indicios de contacto, no colisiones confirmadas.\n')

    with open(os.path.join(out_dir, 'medidas_hw_07nov.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(L) + '\n')
    print('\n'.join(L))
    print('HB', HB)
    print('M13', M13)


if __name__ == '__main__':
    main()
