#!/usr/bin/env python3
"""Sortea las posiciones del objetivo y de la base para la campaña (07-nov-2026).

Regla (D6, Indicadores/PLAN_Analisis_Resultados.md):
  - objetivo uniforme en el área libre, a >= MARGEN_OBJ mm de las paredes y a
    >= DIST_MIN mm de la base;
  - base uniforme a >= MARGEN_BASE mm de las paredes (deja lugar a los robots
    que se estacionan alrededor, ~170 mm), o FIJA con --base x,y (decisión del
    autor 07-nov-2026: base siempre en (300, 300), robots arrancan junto a ella);
  - ni objetivo ni base a < SEP_INI mm de una posición inicial de robot
    (si se pasan con --iniciales);
  - semilla fija; la lista se genera y se guarda ANTES de la primera corrida.

Bloques: N nominales (en el orden sorteado) y luego la línea base, que repite
los escenarios 1..L de la lista nominal (mismas posiciones, otra estrategia).

Uso (desde `Codigo/BDI-E V2`):
  python base/herramientas/sortear_campana.py --base 300,300   # lo de la campaña (07-nov)
  python base/herramientas/sortear_campana.py                  # base también sorteada
  python herramientas/sortear_campana.py --iniciales "300,1200;600,1200;900,1200"
"""
import argparse
import csv
import math
import random

# Área libre de la arena con los robots estacionados (grid.h, 17-oct-2026):
# x en [10, 1910], y en [0, 1400] mm (origen esquina superior izquierda).
X0, X1, Y0, Y1 = 10.0, 1910.0, 0.0, 1400.0
MARGEN_OBJ = 200.0
MARGEN_BASE = 300.0
DIST_MIN = 700.0
SEP_INI = 300.0


def punto(rng, margen):
    return (round(rng.uniform(X0 + margen, X1 - margen)),
            round(rng.uniform(Y0 + margen, Y1 - margen)))


def lejos(p, otros, d):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= d for q in otros)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--semilla', type=int, default=20261107)
    ap.add_argument('--nominales', type=int, default=22)
    ap.add_argument('--linea-base', type=int, default=10)
    ap.add_argument('--base', default='', help='base fija "x,y" (si no, se sortea)')
    ap.add_argument('--iniciales', default='',
                    help='posiciones iniciales fijas de los robots: "x,y;x,y;..."')
    ap.add_argument('--salida', default='campana_posiciones.csv')
    a = ap.parse_args()

    ini = [tuple(float(v) for v in s.split(',')) for s in a.iniciales.split(';') if s.strip()]
    rng = random.Random(a.semilla)
    base_fija = tuple(float(v) for v in a.base.split(',')) if a.base else None
    esc = []
    while len(esc) < a.nominales:
        base = base_fija or punto(rng, MARGEN_BASE)
        if not lejos(base, ini, SEP_INI):
            continue
        for _ in range(1000):
            obj = punto(rng, MARGEN_OBJ)
            if lejos(obj, [base], DIST_MIN) and lejos(obj, ini, SEP_INI):
                break
        else:
            continue
        esc.append((obj, base))

    filas = []
    for i, (obj, base) in enumerate(esc, 1):
        filas.append(dict(orden=i, bloque='nominal', escenario=i,
                          obj_x=obj[0], obj_y=obj[1], base_x=round(base[0]), base_y=round(base[1]),
                          dist_obj_base=round(math.hypot(obj[0] - base[0], obj[1] - base[1]))))
    for j in range(1, a.linea_base + 1):
        obj, base = esc[j - 1]
        filas.append(dict(orden=a.nominales + j, bloque='linea_base', escenario=j,
                          obj_x=obj[0], obj_y=obj[1], base_x=round(base[0]), base_y=round(base[1]),
                          dist_obj_base=round(math.hypot(obj[0] - base[0], obj[1] - base[1]))))

    with open(a.salida, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0]))
        w.writeheader()
        w.writerows(filas)
    print(f'{len(filas)} corridas -> {a.salida} (semilla {a.semilla}, base: {base_fija or "sorteada"}, '
          f'iniciales: {ini or "no declaradas"})')
    for r in filas:
        print(f"  {r['orden']:>2} {r['bloque']:<10} esc {r['escenario']:>2}  "
              f"objeto ({r['obj_x']:>4}, {r['obj_y']:>4})  base ({r['base_x']:>4}, {r['base_y']:>4})  "
              f"d = {r['dist_obj_base']} mm")


if __name__ == '__main__':
    main()
