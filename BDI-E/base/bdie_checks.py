"""
bdie_checks.py — verificaciones de comportamiento de BDI-E, en un solo lugar

Las usan el monitor en vivo (bdie_monitor.py, alertas en la GUI durante la
prueba) y el verificador fuera de línea (herramientas/verificar_bdie.py, sobre
los registros de una sesión). Es la misma regla en los dos lados, para que una
alerta vista en la GUI aparezca también en el reporte y viceversa.

Cada verificación es una función pura que devuelve una lista de hallazgos
(tuplas (código, texto)); una lista vacía significa que pasa. Los códigos
V1–V10 identifican cada verificación.
"""

FASES = ["IDLE", "DESPLEGAR", "BUSCAR", "CONGREGAR", "VOLVER", "FIN", "SEGURO"]

# Transiciones legales del agente (bdi.h). SEGURO se alcanza desde cualquier
# fase (batería, timeout de misión, ABORT); una sesión nueva vuelve a DESPLEGAR
# desde cualquier fase. DESPLEGAR → CONGREGAR es legal: el objetivo puede
# aparecer antes de llegar a la zona.
LEGALES = {
    ("IDLE", "DESPLEGAR"), ("DESPLEGAR", "BUSCAR"), ("DESPLEGAR", "CONGREGAR"),
    ("BUSCAR", "CONGREGAR"), ("CONGREGAR", "VOLVER"), ("VOLVER", "FIN"),
}

NO_PROGRESO_MS = 12000      # noProgressMs de bdi.h
SILENCIO_S = 3.0            # sin THINK durante esto con misión activa = robot mudo
SATURACION_MAX = 0.50       # fracción de celdas en 255 que ya es parálisis
RADIO_BASE_MM = 350.0       # el criterio de retorno del banco


def transicion_legal(antes, despues):
    if antes == despues or despues == "SEGURO" or despues == "DESPLEGAR":
        return True
    return (antes, despues) in LEGALES


def particion(mapa):
    """V2 · las zonas de una réplica: disjuntas y cubriendo toda la cuadrícula."""
    out = []
    filas, cols = mapa["filas"], mapa["cols"]
    if not mapa["zonas"]:
        return []        # todavía no deliberó: la asignación nace en el primer ciclo
    dueno = {}
    for i, (r0, r1, c0, c1) in enumerate(mapa["zonas"]):
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                if (r, c) in dueno:
                    out.append(("V2", f"celda ({r},{c}) en las zonas {dueno[(r, c)]} y {i}"))
                dueno[(r, c)] = i
    faltan = filas * cols - len(dueno)
    if faltan > 0:
        out.append(("V2", f"{faltan} celdas sin zona"))
    usadas = sorted(mapa["de"].values())
    if len(usadas) != len(set(usadas)):
        out.append(("V2", f"dos robots con la misma zona: {mapa['de']}"))
    return out[:3]


def acuerdo_zonas(mapas):
    """V1 · todos los robots tienen el mismo reparto vigente.

        El reparto se acuerda por mensajes (ZC/ZA), y cada BMAP trae el reparto que
        ese robot dio por acordado. Mientras una ronda se cierra hay una ventana de
        desacuerdo esperada (el monitor la tolera exigiendo que el desacuerdo
        persista); un desacuerdo que dura es un defecto.

        mapas: {id: mapa decodificado}, todos recientes. Compara la asignación de
        cada robot contra la del de menor ID."""
    ids = sorted(mapas)
    if len(ids) < 2:
        return []
    ref = mapas[ids[0]]
    out = []
    for i in ids[1:]:
        m = mapas[i]
        if m["zonas"] != ref["zonas"] or m["de"] != ref["de"]:
            out.append(("V1", f"R{i} ve {m['de']} y R{ids[0]} ve {ref['de']}"))
    return out


def zona_propia(think, mapa):
    """V3 · la zona que el agente persigue es la que su asignación le da."""
    rid = think["id"]
    # La zona solo importa mientras se reparte y se busca. Al terminar (FIN),
        # abortar (SEGURO) o fuera de misión, el robot borra su memoria y su zona
        # queda en -1, aunque el BMAP siga mostrando el último reparto.
    if think.get("fase") not in ("DESPLEGAR", "BUSCAR"):
        return []
    esperada = mapa["de"].get(rid, -1)
    if think["zona"] != esperada:
        return [("V3", f"R{rid} dice zona {think['zona']} y su asignación dice {esperada}")]
    return []


def dueno_objetivo(thinks):
    """V6 · todos los que conocen el objetivo coinciden en de quién es y en
    DÓNDE está (la celda). Dos celdas distintas son dos congregaciones."""
    conocen = [t for t in thinks.values() if t["obj"] == 1]
    duenos = {t["dueno"] for t in conocen}
    celdas = {t["cobj"] for t in conocen}
    out = []
    if len(duenos) > 1:
        out.append(("V6", f"dueños distintos del objetivo: {sorted(duenos)}"))
    if len(celdas) > 1:
        quien = ", ".join(f"R{t['id']}→{t['cobj']}" for t in sorted(conocen, key=lambda t: t["id"]))
        out.append(("V6", f"el objetivo está en celdas distintas según quién: {quien}"))
    return out


def saturacion(mapa):
    """V8 · la feromona no se satura (con todo en 255 la ruleta es uniforme)."""
    n = len(mapa["R"])
    if n and sum(1 for v in mapa["R"] if v >= 255) / n > SATURACION_MAX:
        return [("V8", f"R{mapa['id']}: más del {SATURACION_MAX:.0%} de celdas saturadas")]
    return []


# V9 no existe: la congregación no usa feromona atractiva, la hace el MEET del
# laboratorio. La llegada se lee del THINK (lleg=1), que es lo que declara
# ReactiveNavStep al alcanzar la ranura.
