#!/usr/bin/env python3
"""Congela el firmware de una campaña: huella SHA-256 y copia de respaldo.

Se corre después de programar la versión que va a la campaña y antes de la
primera corrida nominal, desde la carpeta BDI-E:

    python base/herramientas/congelar_firmware.py

Deja `_congelado_<fecha>/` con una copia de robot/AttaBot/ y firmware/, y el
archivo FIRMWARE_CONGELADO.txt con la huella de cada archivo y una huella
global. Si después cambia un archivo, --verificar <carpeta> indica cuál.
"""
import argparse
import datetime
import hashlib
import os
import shutil
import sys

CARPETAS = ['robot/AttaBot', 'firmware']
EXT = ('.ino', '.h', '.cpp', '.csv')


def archivos():
    for c in CARPETAS:
        for n in sorted(os.listdir(c)):
            if n.endswith(EXT):
                yield os.path.join(c, n).replace('\\', '/')


def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--verificar', metavar='CARPETA_CONGELADA',
                    help='compara el código actual con un _congelado_<fecha>/')
    a = ap.parse_args()
    if not os.path.isdir('robot/AttaBot'):
        sys.exit('Correr desde la carpeta «Codigo/BDI-E V2».')

    if a.verificar:
        ref = {}
        for linea in open(os.path.join(a.verificar, 'FIRMWARE_CONGELADO.txt'), encoding='utf-8'):
            p = linea.split()
            if len(p) == 2 and len(p[0]) == 64:
                ref[p[1]] = p[0]
        cambios = [f for f in archivos() if ref.get(f) != sha(f)]
        cambios += [f for f in ref if not os.path.exists(f)]
        print('Sin cambios: el firmware es el congelado.' if not cambios
              else 'CAMBIARON: ' + ', '.join(cambios))
        return

    hoy = datetime.date.today().strftime('%d%b%Y').lower()
    dst = f'_congelado_{hoy}'
    if os.path.exists(dst):
        sys.exit(f'{dst} ya existe: no se pisa un congelado.')
    os.makedirs(dst)
    glob = hashlib.sha256()
    lineas = []
    for f in archivos():
        h = sha(f)
        glob.update((h + f).encode())
        lineas.append(f'{h}  {f}')
        os.makedirs(os.path.join(dst, os.path.dirname(f)), exist_ok=True)
        shutil.copy2(f, os.path.join(dst, f))
    g = glob.hexdigest()
    with open(os.path.join(dst, 'FIRMWARE_CONGELADO.txt'), 'w', encoding='utf-8') as o:
        o.write(f'Firmware congelado {datetime.datetime.now():%Y-%m-%d %H:%M}\n')
        o.write(f'Huella global: {g}\n\n')
        o.write('\n'.join(lineas) + '\n')
    print(f'Congelado en {dst}/  ·  huella global {g[:16]}…  ·  {len(lineas)} archivos')


if __name__ == '__main__':
    main()
