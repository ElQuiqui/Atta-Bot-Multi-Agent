"""Genera los prototipos que el IDE de Arduino crearía automáticamente.

Recorre AttaBot.ino y después los demás .ino en orden alfabético, igual que el
preprocesador de Arduino, y escribe una declaración por cada función definida
en el nivel superior. Lo usa pruebas_host/build.sh para compilar el sketch en
la PC.

    python3 protos.py <carpeta del sketch> <archivo de salida>
"""
import re, sys, os
S = sys.argv[1]
files = ["AttaBot.ino"] + sorted(f for f in os.listdir(S) if f.endswith(".ino") and f != "AttaBot.ino")
protos = []
for f in files:
    lines = open(os.path.join(S, f), encoding="utf-8").read().split("\n")
    i = 0
    while i < len(lines):
        L = lines[i]
        if L and not L[0].isspace() and not L.startswith(("#","//","/*","*","}")) and "(" in L \
           and not re.match(r"(inline|static|struct|class|enum|typedef|const |return|else|if|for|while)\b", L) and "=" not in L.split("(")[0]:
            sig = L; j = i
            while "{" not in sig and ";" not in sig and j + 1 < len(lines):
                j += 1; sig += " " + lines[j].strip()
            if "{" in sig and ";" not in sig.split("{")[0]:
                s = sig.split("{")[0].strip()
                if s.endswith(")") and "::" not in s.split("(")[0]:
                    protos.append(s + ";")
            i = j
        i += 1
open(sys.argv[2], "w").write("\n".join(protos) + "\n")
print(len(protos), "prototipos")
