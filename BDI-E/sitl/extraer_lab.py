#!/usr/bin/env python3
# =============================================================================
#  extraer_lab.py — copia VERBATIM las funciones de congregación del laboratorio
#
#  El banco no reescribe la congregación: la COMPILA. Este script toma del
#  sketch que se flashea (robot/AttaBot/*.ino) las funciones que ejecutan un
#  MEET —el reparto de ranuras, el anillo contra las paredes, la aproximación
#  en dos etapas, el paso de navegación reactiva y la llegada— y las copia tal
#  cual a lab_cong_gen.inc, que lab_cong.cpp incluye DENTRO de una estructura.
#  Así las globales del .ino pasan a ser miembros y cada robot del banco tiene
#  las suyas: cinco robots, cinco copias del estado, un solo código.
#
#  Si el laboratorio cambia una de estas funciones, el banco la toma en el
#  próximo `make`. No hay una segunda versión que mantener.
# =============================================================================
import os, re, sys

AQUI   = os.path.dirname(os.path.abspath(__file__))
SKETCH = os.path.normpath(os.path.join(AQUI, "..", "robot", "AttaBot"))
SALIDA = os.path.join(AQUI, "lab_cong_gen.inc")

# (archivo, función). El orden no importa dentro de una estructura.
FUNCIONES = [
    ("AttaBot.ino",    "DegToRad"),
    ("AttaBot.ino",    "DegToArc"),
    ("comandos.ino",   "HandleMeet"),
    ("comandos.ino",   "HandleNeighborPositions"),
    ("comandos.ino",   "HandlePositionResponse"),
    ("navegacion.ino", "SafeRingSlotAngle"),
    ("navegacion.ino", "RequestPositionQueued"),
    ("navegacion.ino", "UpdateCongregationGoal"),
    ("navegacion.ino", "MeetSlotIndex"),
    ("navegacion.ino", "ReactiveNavStep"),
    # el pegamento de BDI-E con el MEET: también el mismo que se flashea
    ("bdie.ino",       "BdieStartMeet"),
    ("bdie.ino",       "BdieDropMeet"),
    ("bdie.ino",       "BdieOnNavArrived"),
]


def cierre(src, i):
    """Índice justo después de la llave que cierra la que abre en src[i]."""
    assert src[i] == "{"
    prof, n = 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            i = src.find("\n", i); i = n if i < 0 else i; continue
        if src.startswith("/*", i):
            i = src.find("*/", i) + 2; continue
        if c in "\"'":
            j = i + 1
            while src[j] != c:
                j += 2 if src[j] == "\\" else 1
            i = j + 1; continue
        if c == "{": prof += 1
        elif c == "}":
            prof -= 1
            if prof == 0: return i + 1
        i += 1
    raise ValueError("llave sin cerrar")


def extraer(archivo, nombre):
    src = open(os.path.join(SKETCH, archivo), encoding="utf-8").read()
    # definición a comienzo de línea (no un prototipo: termina en '{', no en ';')
    pat = re.compile(r"^[A-Za-z_][\w\s\*&:<>,]*?\b" + nombre + r"\s*\([^;{]*\)\s*\{", re.M)
    m = pat.search(src)
    if not m:
        sys.exit("no se encontró la definición de %s en %s" % (nombre, archivo))
    ini = m.start()
    fin = cierre(src, m.end() - 1)
    linea = src.count("\n", 0, ini) + 1
    return '#line %d "%s"\n%s\n' % (linea, archivo, src[ini:fin])


def main():
    partes = ["// GENERADO por extraer_lab.py desde robot/AttaBot/ — NO EDITAR.\n"
              "// Copia verbatim de las funciones de congregación del laboratorio.\n\n"]
    for archivo, nombre in FUNCIONES:
        partes.append(extraer(archivo, nombre))
        partes.append("\n")
    open(SALIDA, "w", encoding="utf-8").write("".join(partes))
    print("  %d funciones del laboratorio → %s" % (len(FUNCIONES), os.path.basename(SALIDA)))


if __name__ == "__main__":
    main()
