# BDI-E — Arquitectura multiagente descentralizada para el Atta-Bot

Código del proyecto final de graduación *«Arquitectura multiagente descentralizada para búsqueda
autónoma bajo entornos de emergencia en enjambres de robots»* (Ingeniería Mecatrónica, Instituto
Tecnológico de Costa Rica, 2026), desarrollado en el Grupo de Investigación en Robótica Múltiple
(GIRoM). Autor: Cristhofer Loría Calderón. Asesor: Ing. Felipe Meza Obando.

BDI-E (*BDI estigmérgico*) hace que cada Atta-Bot complete la misión **por su cuenta y a bordo
de su ESP32**, sin que la computadora le dé órdenes. La misión tiene cuatro fases:

1. Buscar un objetivo marcado por color.
2. Detectarlo.
3. Congregarse a su alrededor.
4. Volver en grupo a la base.

La cámara cenital sigue informando la posición de cada robot, pero **la decisión es de cada
robot**.

Dentro de cada robot conviven tres capas:

| Capa | Qué hace |
|---|---|
| **Social** | Acuerda por mensajes qué zona de la arena le toca a cada robot y difunde el hallazgo de forma epidémica. |
| **Deliberativa** | Ciclo de creencias, deseos e intenciones (BDI). Usa un mapa de feromona repelente para no repetir celdas y sigue un plan de retorno en formación laxa. |
| **Reactiva** | La evasión de obstáculos del firmware del laboratorio, sin cambios. |

Para congregarse se usa el `MEET` del laboratorio, sin líder.

## Base de esta carpeta

El firmware y la base parten del `main` de [`jcbrenes/Atta-Bot`](https://github.com/jcbrenes/Atta-Bot),
commit `82b9b8b` (13-ago-2026). Es la versión con siete `.ino` por tema y la gramática
`DESTINO.VERBO|args`.

**Esta carpeta es independiente: no modifica nada fuera de `BDI-E/`.**

## Estructura

| Carpeta | Contenido |
|---|---|
| `firmware/` | Núcleo de BDI-E en C++ **sin dependencias de Arduino**. Es la fuente de verdad. Ver el detalle abajo. |
| `robot/AttaBot/` | **El sketch completo para el robot.** Es el firmware del laboratorio con enganches de una línea marcados `BDI-E`, más `bdie.ino` (todo el acople), `latencia.ino` (medición de latencia entre robots) y copias de los seis encabezados de `firmware/`. |
| `base/` | La base del laboratorio con los enganches de BDI-E. Ver el detalle abajo. |
| `base/herramientas/` | Análisis de las corridas, en `analisis_corrida.py`, `resumen_campana.py` y `medidas_hw_07nov.py`. También el sorteo de la campaña, la verificación de una sesión (`verificar_bdie.py`) y el diagnóstico de ArUco. |
| `sitl/` | Banco de simulación *software-in-the-loop* (SITL). `bdie_bridge.cpp` compila **el mismo C++ que corre en el robot** y `bdie_sim.py` lo usa vía `ctypes`. `lab_cong.cpp` y `extraer_lab.py` compilan la congregación del laboratorio tal como está en el sketch. |
| `pruebas_host/` | El sketch compilado en la PC contra *stubs* de hardware, junto con sus pruebas. Ver el detalle abajo. |
| `Makefile.mk` | Recetas para el banco y las pruebas. |

**`firmware/`** tiene seis encabezados:

| Archivo | Qué contiene |
|---|---|
| `grid.h` | La cuadrícula. |
| `pheromone_map.h` | El mapa de feromona. |
| `assignment.h` | El reparto de zonas. |
| `bdi.h` | El ciclo BDI y sus planes. |
| `mission.h` | El protocolo y las fases. |
| `telemetry.h` | La telemetría a la base. |

**`base/`** agrega estos archivos a la base del laboratorio:

| Archivo | Qué hace |
|---|---|
| `swarm_broadcast.py` | Difunde las poses del enjambre. |
| `bdie_monitor.py` | Recibe la telemetría y guarda los registros. |
| `bdie_checks.py` | Corre diez verificaciones en vivo. |
| `bdie_panel.py`, `mision_panel.py` | Agregan pestañas a la GUI. |
| `cubo_vision.py` | Ubica el objetivo con la cámara solo como verdad de terreno, para el análisis. **No se lo comunica a los robots.** |
| `prueba_base_bdie.py` | Prueba de la base. |

**`pruebas_host/`** incluye tres pruebas en C++ y la prueba de observación:

| Archivo | Qué prueba |
|---|---|
| `test_v2.cpp` | Integración del sketch completo. |
| `prueba_reparto.cpp` | El reparto por mensajes contra un oráculo. |
| `prueba_propiedades_bdi.cpp` | Las propiedades del ciclo BDI. |
| `prueba_observacion.py` | La observación (telemetría, monitor y verificador). |

En `robot/AttaBot/`, `perifericos.ino` y `sensores.ino` están sin cambios. Para ver exactamente qué
se tocó del firmware del laboratorio, compare esa carpeta con `Controller/AttaBot/` del commit
indicado arriba, o busque `BDI-E` en los comentarios.

## Puntos de contacto con el firmware del laboratorio

| Archivo | Cambio |
|---|---|
| `utils.h` | Estado `DELIBERATE` al final del `enum` de la máquina de estados. |
| `estados.ino` | Agrega `case DELIBERATE` → `StateDeliberate()`. Intercepta `WAIT` y conecta la búsqueda por color del laboratorio con el agente (`BdieSyncSearch()` y `BdieTargetConfirmed()`). En cada `loop()`, `BdieLoopHook()` atiende el guardián de la navegación, el latido y la búsqueda por color. |
| `comandos.ino` | `ReadUdpPackets()`: una rama para los mensajes de BDI-E → `HandleBdieMessage()`, que es la misma `Feed()` que usa el banco. También `LAT_PING`. Agrega los campos de BDI-E a `GET_STATUS`, y `CONFIG` pasa el ID real al agente. |
| `AttaBot.ino` | `WiFi.setSleep(false)`. La ranura de los infrarrojos se deriva del ID: el firmware original sorteaba la ranura antes de `randomSeed()`, de modo que todos los robots emitían a la vez. |
| `motores.ino` | Durante la misión no se vuelve a armar la patrulla de 10 min tras un falso positivo de color: el control vuelve al agente. |
| `navegacion.ino` | Avisa al agente cuando el robot llega a su ranura del `MEET` (`BdieOnNavArrived()`). |

**Así queda la arquitectura:**
- La máquina de estados del laboratorio sigue ejecutando avanzar, girar y evitar obstáculos.
- El agente solo decide **cuál es la próxima celda** y se la entrega por `instructionList`.
- Los lazos PID, la evasión y la comunicación no cambian.

## Mensajes

Todo viaja por **difusión UDP en el puerto 6060**, sobre la WiFi de infraestructura. Ningún mensaje
pasa de **cinco campos**, que es el límite del parser del firmware.

**Entre robots y desde la base:**

| Mensaje | Qué lleva |
|---|---|
| `POSES` | Posiciones de todos los robots, una vez por cuadro. Las envía la base. |
| `SESSION\|START\|x\|y` | Inicio de la misión. |
| `SESSION\|ABORT` | Interrupción de la misión. |
| `SESSION\|JOIN\|id` | Reincorporación de un robot. |
| `HB` | Latido, a 2 Hz. Un compañero cuenta como «vivo» si se le oyó en los últimos 3 s. |
| `TGT`, `TGT_REQ` | Anuncio del objetivo y petición del objetivo. |
| `ZC`, `ZA` | Reparto de zonas: `ZC` es el reclamo y `ZA` el acuerdo. En un conflicto gana el robot más cercano; a igual distancia, el de menor ID. |

**Del robot a la base**, solo para observación:

| Mensaje | Frecuencia | Qué lleva |
|---|---|---|
| `THINK` | 2 Hz | Fase, deseo, plan y zona. |
| `BMAP` | 1 Hz | Feromona, cobertura y zonas. |

**Comandos nuevos:**

| Comando | Para qué |
|---|---|
| `TARGET_COLOR\|rojo\|verde\|azul` | Elegir el color del objetivo. |
| `BDIE_TELEM\|0\|1\|2` | Nivel de telemetría. |
| `BASE.SESSION\|START\|x\|y` | Iniciar la misión desde la consola o la GUI. |
| `BASE.SESSION\|ABORT` | Interrumpir la misión. |
| `BASE.SESSION\|STATE` | Consultar el estado de la misión. |
| `BASE.SESSION\|JOIN\|id` | Reincorporar un robot. |

## Compilar y cargar el robot

1. Abrir `robot/AttaBot/AttaBot.ino` con el IDE de Arduino, con el núcleo ESP32 de Espressif. Usar
   la tabla de particiones `partitions.csv` de la carpeta.
2. Instalar estas bibliotecas, las mismas del firmware del laboratorio:

   | Biblioteca | Versión |
   |---|---|
   | Adafruit APDS9960 | 1.3.1 |
   | FastLED | 3.10.5 |
   | SparkFun ICM-20948 | 1.3.2 |

   En la de SparkFun, activar `ICM_20948_USE_DMP` en `ICM_20948_C.h`.
3. Los seis `.h` de `robot/AttaBot/` son **copias** de `firmware/`. Si se edita el núcleo, se edita
   en `firmware/` y se copia. Para comprobar que las copias coinciden:
   `make -f Makefile.mk copias`.

## Correr la base

Requisitos: Python 3, `numpy`, `opencv-contrib-python` (ArUco), `PyQt5` y `matplotlib`.

```
cd base
python3 AttaBot_Base.py --robots 4      # --sim para correr sin cámara
```

En la consola de la base:

| Acción | Comando |
|---|---|
| Iniciar la misión hacia la base en (x, y) mm | `BASE.SESSION\|START\|x\|y` |
| Reincorporar un robot que se reinició | `CONFIG` y después `BASE.SESSION\|JOIN\|id` |

La GUI muestra dos pestañas nuevas: «Feromonas y zonas» y «Mente BDI-E». La telemetría se guarda en
`base/BdieLogs/`, y `base/herramientas/verificar_bdie.py` genera el reporte de una sesión.

## Banco de simulación y pruebas

Se corren en Linux, con `g++` y `python3`:

| Comando | Qué hace |
|---|---|
| `make -f Makefile.mk indicador` | Compila `sitl/libbdie.so` y corre los diez escenarios de verificación. |
| `make -f Makefile.mk sketch` | Compila el sketch completo contra *stubs* y corre la prueba de integración. |
| `make -f Makefile.mk reparto` | Prueba el reparto por mensajes contra el oráculo, con pérdida, caída, silencio y reincorporación. |
| `make -f Makefile.mk observacion` | Corre la telemetría, el monitor y el verificador sobre el banco. |
| `make -f Makefile.mk copias` | Comprueba que `firmware/*.h` sea igual a `robot/AttaBot/*.h`. |
| `cd sitl && python3 bdie_sim.py --fallo` | Simula caídas de un robot durante la búsqueda. |
| `cd sitl && python3 bdie_sim.py --reconexion` | Simula un silencio de 20 s y un reinicio con `JOIN`. |

La prueba de propiedades del ciclo BDI se compila y se corre así:

```
g++ -std=c++17 -Ifirmware pruebas_host/prueba_propiedades_bdi.cpp -o prueba && ./prueba
```

## Análisis de una campaña

`base/herramientas/resumen_campana.py --hoja <hoja de corridas.csv>` reduce los registros de la
cámara y de los robots. Produce:
- tiempos por fase;
- tasa de misiones exitosas, con intervalos de Clopper–Pearson y prueba binomial exacta;
- fracción de agentes que completan la misión;
- radio del enjambre en el retorno y cobertura.

`analisis_corrida.py` analiza una sola corrida.

## Limitaciones conocidas

El banco encontró dos defectos del agente. Su arreglo está identificado, pero no entró en el
firmware de la campaña:
- **Celda del objetivo.** El robot que detecta el objetivo puede ubicarlo en una celda vecina a la
  que usan los demás.
- **Desacuerdo de zonas tras una caída.** Después de la caída de un robot, las réplicas pueden
  quedar un tiempo en desacuerdo sobre las zonas.

Además:
- **Robot trabado.** «Vivo = lo oigo»: un robot trabado que sigue transmitiendo conserva su zona.

## Licencia

GPL-3.0, la misma del repositorio (ver `LICENSE` en la raíz).
