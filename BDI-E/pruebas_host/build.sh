#!/bin/bash
# Uso: build.sh <carpeta del sketch> [prueba.cpp]
# Imita el preprocesador de Arduino: prototipos automáticos, AttaBot.ino primero
# y los demás .ino en orden alfabético, en una sola unidad de traducción.
H="$(cd "$(dirname "$0")" && pwd)"; S="$1"; OUT="$H/sketch.cpp"
python3 "$H/protos.py" "$S" "$H/protos.txt" >/dev/null
{ echo '#include "Arduino.h"'
  echo "#line 1 \"AttaBot.ino\""
  awk -v P="$H/protos.txt" '{print} /^#include <Wire.h>/{while((getline l < P)>0) print l; print "#line " NR+1 " \"AttaBot.ino\""}' "$S/AttaBot.ino"
  for f in $(cd "$S"; ls *.ino | grep -v '^AttaBot.ino$' | LC_ALL=C sort); do echo "#line 1 \"$f\""; cat "$S/$f"; done
  if [ -n "$2" ]; then echo "#line 1 \"$(basename "$2")\""; cat "$2"; else echo 'int main(){setup(); for(int i=0;i<3;i++) loop(); return 0;}'; fi
} > "$OUT"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable \
    -Wno-unused-function -Wno-sign-compare -Wno-switch -Wno-reorder -Wno-missing-field-initializers \
    -DARDUINO=10819 -I"$H/stubs" -I"$S" "$OUT" "$H/stubs/globals.cpp" -o "$H/sketch.bin"
