#!/usr/bin/env python3
"""
verificar_bdie.py — reporte de una sesión BDI-E a partir de sus registros

    python3 herramientas/verificar_bdie.py                 # la última sesión de BdieLogs/
    python3 herramientas/verificar_bdie.py BdieLogs/BDIE_<fecha>_
    python3 herramientas/verificar_bdie.py ... --caida 2   # se apagó el robot 2 a propósito

Reproduce la sesión con el mismo BdieMonitor que corrió en vivo (mismas
verificaciones, bdie_checks.py) y agrega lo que solo se puede decir al final:
qué fases alcanzó cada robot y cuándo, cuánto tardó en propagarse el hallazgo,
quién se congregó, quién volvió y si la misión fue exitosa.

Escribe <prefijo>reporte.md junto a los registros y devuelve 0 si pasa y 1 si no.
"""
import csv
import glob
import json
import math
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(AQUI))
import bdie_checks as CK                                    # noqa: E402
from bdie_monitor import BdieMonitor, NUMERICOS             # noqa: E402

GRAVES = {"V1", "V2", "V3", "V4", "V5", "V6", "V8"}   # V7 y V10 dependen de la prueba


def cargar(prefijo):
    eventos = []
    with open(prefijo + "think.csv", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            d = {}
            for k, v in row.items():
                if k in NUMERICOS:
                    d[k] = int(v)
                elif k in ("x", "y", "tlog"):
                    d[k] = float(v)
                else:
                    d[k] = v
            eventos.append((d["tlog"], 1, "think", d))
    meta = {}
    with open(prefijo + "mapas.jsonl", encoding="utf-8") as f:
        for linea in f:
            r = json.loads(linea)
            if r.get("tipo") == "meta":
                meta = r
            else:
                r["de"] = {int(k): v for k, v in r["de"].items()}
                r["zonas"] = [tuple(z) for z in r["zonas"]]
                eventos.append((r["tlog"], 2, "mapa", r))
    if os.path.exists(prefijo + "poses.csv"):
        with open(prefijo + "poses.csv", encoding="utf-8") as f:
            por_t = {}
            for row in csv.DictReader(f):
                por_t.setdefault(float(row["tlog"]), {})[int(row["id"])] = (
                    float(row["x"]), float(row["y"]), float(row["ang"]))
            for t, p in por_t.items():
                eventos.append((t, 0, "poses", p))
    eventos.sort(key=lambda e: (e[0], e[1]))
    return meta, eventos


def verificar(prefijo, caidos=(), salida=True):
    meta, eventos = cargar(prefijo)
    mon = BdieMonitor(escribir=False, log=lambda m: None)
    if meta.get("home"):
        mon.home = tuple(meta["home"])
    mon.sesion = True
    # Lo que solo se ve al final
    fases = {}          # id → {fase: primer tlog}
    obj_t = {}          # id → primer tlog con obj=1
    duenos = set()
    cerr_t = {}
    lleg_t = {}
    poses = {}          # id → [(t, x, y)]
    ultimo = {}
    tfin = 0.0
    difs = []           # diferencia media entre réplicas de la feromona repelente
    ultimo_mapa = {}
    for t, _, tipo, d in eventos:
        tfin = max(tfin, t)
        if tipo == "think":
            mon.on_think(d, t)
            f = fases.setdefault(d["id"], {})
            f.setdefault(d["fase"], t)
            if d["obj"] == 1:
                obj_t.setdefault(d["id"], t)
                duenos.add(d["dueno"])
            if d["cerr"] == 1:
                cerr_t.setdefault(d["id"], t)
            if d.get("lleg") == 1:
                lleg_t.setdefault(d["id"], t)
            ultimo[d["id"]] = d
        elif tipo == "mapa":
            mon.on_map(d, t)
            ultimo_mapa[d["id"]] = d
            frescos = [m for m in ultimo_mapa.values() if t - m["tlog"] <= 1.2]
            if len(frescos) >= 2 and d["id"] == max(m["id"] for m in frescos):
                ref = frescos[0]["R"]
                for m in frescos[1:]:
                    if len(m["R"]) == len(ref) and ref:
                        difs.append(sum(abs(a - b) for a, b in zip(m["R"], ref)) / (255.0 * len(ref)))
        else:
            mon.on_poses(d, t)
            for i, (x, y, _) in d.items():
                poses.setdefault(i, []).append((t, x, y))
        mon.tick(t)

    ids = sorted(fases)
    vivos = [i for i in ids if i not in caidos]
    L = []
    L.append(f"# Reporte BDI-E — {os.path.basename(prefijo)}\n")
    L.append(f"Robots: {', '.join(f'R{i}' for i in ids)}" +
             (f" · caída inducida: {', '.join(f'R{i}' for i in caidos)}" if caidos else "") +
             f" · duración registrada {tfin:.0f} s · base {mon.home}\n")

    # Fases
    L.append("## Fases alcanzadas (s desde el inicio)\n")
    orden = ["DESPLEGAR", "BUSCAR", "CONGREGAR", "VOLVER", "FIN", "SEGURO"]
    L.append("| robot | " + " | ".join(orden) + " |")
    L.append("|---|" + "---|" * len(orden))
    for i in ids:
        L.append(f"| R{i} | " + " | ".join(
            f"{fases[i][f]:.0f}" if f in fases[i] else "—" for f in orden) + " |")

    # Hallazgo y propagación
    L.append("\n## Hallazgo\n")
    t_hall = min(obj_t.values()) if obj_t else None
    if t_hall is None:
        L.append("- **No se encontró el objetivo.**")
    else:
        prop = max(obj_t.get(i, math.inf) for i in vivos) - t_hall
        L.append(f"- primer hallazgo a los **{t_hall:.1f} s**, dueño(s): {sorted(duenos)}")
        L.append(f"- propagación a todos los vivos: **{prop:.1f} s**" if math.isfinite(prop)
                 else "- ⚠ algún robot vivo nunca supo del objetivo")

    # Congregación y retorno contra la pose de cámara
    L.append("\n## Congregación y retorno (pose de cámara)\n")
    cong = ret = 0
    tgt = None
    # El objetivo: la verdad del banco si la hay; si no, la celda en la que
    # coincide la MAYORÍA de los robots (con V6 abierta pueden discrepar).
    ref = next(iter(mon.mapas.values()), None)
    celdas = [u["cobj"] for u in ultimo.values() if u["obj"] == 1 and u["cobj"] >= 0]
    if meta.get("objetivo_real"):
        tgt = tuple(meta["objetivo_real"])
    elif celdas and ref:
        r, c = divmod(max(set(celdas), key=celdas.count), ref["cols"])
        tgt = (ref["ox"] + (c + .5) * ref["celda"], ref["oy"] + (r + .5) * ref["celda"])
    t_cierre = min(cerr_t.values()) if cerr_t else None
    # «Congregó» = el MEET del laboratorio declaró la llegada a su ranura (THINK
    # lleg=1): el mismo criterio que el banco. La distancia mínima al objetivo
    # queda como dato, no como criterio (el anillo del lab crece contra la pared).
    L.append("| robot | dist. mín. al objetivo | congregó (lleg=1) | dist. mín. a la base tras el cierre | volvió |")
    L.append("|---|---|---|---|---|")
    for i in vivos:
        pp = poses.get(i, [])
        dmt = min((math.hypot(x - tgt[0], y - tgt[1]) for _, x, y in pp), default=math.inf) if tgt else math.inf
        dmb = min((math.hypot(x - mon.home[0], y - mon.home[1]) for t, x, y in pp
                   if t_cierre is not None and t >= t_cierre), default=math.inf) if mon.home else math.inf
        ok_c, ok_r = i in lleg_t, dmb <= CK.RADIO_BASE_MM
        cong += ok_c
        ret += ok_r
        L.append(f"| R{i} | {dmt:.0f} mm | {'sí' if ok_c else 'no'} | {dmb:.0f} mm | {'sí' if ok_r else 'no'} |")
    n = max(1, len(vivos))
    umbral = math.ceil(0.8 * n)
    completa = (t_hall is not None and t_cierre is not None and cong >= umbral
                and ret >= umbral and tfin <= 20 * 60)
    L.append(f"\nCongregaron {cong}/{n}, volvieron {ret}/{n} (umbral {umbral}). "
             f"Cierre de la congregación: {f'{t_cierre:.0f} s' if t_cierre else 'no ocurrió'}.")
    L.append(f"\n**Misión completa (criterio del cliente): {'SÍ' if completa else 'NO'}**")
    if not poses:
        L.append("\n⚠ Sin poses de cámara en el log: congregación y retorno no se pueden medir.")

    # Consistencia de la feromona entre réplicas: no tiene umbral; se reporta para
        # comparar sesiones.
    if difs:
        difs.sort()
        L.append(f"\nFeromona repelente, diferencia media entre réplicas: mediana "
                 f"{difs[len(difs) // 2]:.1%}, p95 {difs[int(0.95 * (len(difs) - 1))]:.1%} "
                 f"({len(difs)} comparaciones). Consistencia EVENTUAL: no tiene que ser 0.")

    # Alertas
    L.append("\n## Verificaciones\n")
    por = {}
    for (t, r, cod, txt) in mon.alertas:
        por.setdefault(cod, []).append((t, r, txt))
    esperadas = set()
    if caidos:
        esperadas.add("V10")
    for cod in ["V1", "V2", "V3", "V4", "V5", "V6", "V7", "V8", "V10"]:
        lst = por.get(cod, [])
        estado = "✅ pasa" if not lst else ("ℹ esperada" if cod in esperadas else
                                            ("❌ FALLA" if cod in GRAVES or cod == "V10" else "⚠ revisar"))
        L.append(f"- **{cod}** {estado}" + (f" — {len(lst)} alerta(s); primera a los {lst[0][0]:.0f} s: {lst[0][2]}" if lst else ""))
    # V10 (un robot que deja de hablar) es grave salvo que la caída se haya
    # declarado: apagar un robot a propósito es la prueba de fallo inducido.
    graves = [c for c in por if c in GRAVES or (c == "V10" and not caidos)]
    # Caída: todos los vivos la registran y se reasignan zonas
    if caidos:
        for k in caidos:
            sin = [i for i in vivos if ultimo.get(i) and int(str(ultimo[i]["vivos"]), 16) >> k & 1]
            L.append(f"- caída de R{k}: " + ("✅ todos los vivos la registraron" if not sin
                     else f"❌ siguen contándolo vivo: {sin}"))
            if sin:
                graves.append("caida")
    pasa = completa and not graves
    L.append(f"\n## Veredicto: {'PASA' if pasa else 'NO PASA'}\n")
    texto = "\n".join(L)
    with open(prefijo + "reporte.md", "w", encoding="utf-8") as f:
        f.write(texto + "\n")
    if salida:
        print(texto)
    return pasa, dict(completa=completa, graves=graves, alertas=por, t_hallazgo=t_hall)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    caidos = []
    if "--caida" in sys.argv:
        caidos = [int(x) for x in sys.argv[sys.argv.index("--caida") + 1].split(",")]
        args = [a for a in args if a != sys.argv[sys.argv.index("--caida") + 1]]
    if args:
        prefijo = args[0]
    else:
        cand = sorted(glob.glob(os.path.join(os.path.dirname(AQUI), "BdieLogs", "BDIE_*_think.csv")),
                      key=os.path.getmtime)
        if not cand:
            sys.exit("No hay sesiones en BdieLogs/.")
        prefijo = cand[-1][:-len("think.csv")]
    pasa, _ = verificar(prefijo, caidos)
    sys.exit(0 if pasa else 1)


if __name__ == "__main__":
    main()
