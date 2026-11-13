# BDI-E V2 — banco SITL, prueba de integración del sketch y control de copias
#
#   make -f Makefile.mk indicador   compila libbdie.so y CORRE el indicador de OE2
#   make -f Makefile.mk sketch      el sketch COMPLETO (.ino del lab + bdie.ino) en
#                                   el host, contra stubs, y su prueba de integración
#   make -f Makefile.mk observacion la telemetría, el monitor y el verificador sobre
#                                   el banco, con defectos inyectados (PRUEBAS_observacion.md)
#   make -f Makefile.mk gui         capturas de la GUI con los paneles BDI-E, sin robots
#   make -f Makefile.mk reparto     el reparto por mensajes contra el oráculo (R1–R6)
#   make -f Makefile.mk copias      firmware/*.h == robot/AttaBot/*.h
#   make -f Makefile.mk limpiar

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra
PY       ?= python3

FIRMWARE := $(wildcard firmware/*.h)
LIB      := sitl/libbdie.so
# La congregación del laboratorio, compilada verbatim desde el sketch (ver
# sitl/extraer_lab.py): cambia si cambia cualquier .ino o utils.h.
LAB_SRC  := $(wildcard robot/AttaBot/*.ino) robot/AttaBot/utils.h
LAB_GEN  := sitl/lab_cong_gen.inc
LAB_OBJ  := sitl/lab_cong.o

.PHONY: all sitl indicador sketch observacion gui copias limpiar reparto

all: $(LIB)

$(LAB_GEN): sitl/extraer_lab.py $(LAB_SRC)
	$(PY) sitl/extraer_lab.py

$(LAB_OBJ): sitl/lab_cong.cpp $(LAB_GEN)
	$(CXX) -std=gnu++17 -O2 -fPIC -Wall -Wextra -Wno-unused-parameter -Wno-reorder \
	    -Wno-sign-compare -Wno-unused-function -Wno-unused-variable \
	    -Ipruebas_host/stubs -Irobot/AttaBot -c -o $@ sitl/lab_cong.cpp

$(LIB): sitl/bdie_bridge.cpp $(FIRMWARE) $(LAB_OBJ)
	$(CXX) $(CXXFLAGS) -fPIC -shared -o $@ sitl/bdie_bridge.cpp $(LAB_OBJ)

sitl: $(LIB)

# El indicador de OE2, congelado: diez escenarios, sin argumentos.
indicador: $(LIB)
	cd sitl && $(PY) bdie_sim.py --indicador

# Imita el preprocesador de Arduino (AttaBot.ino primero, el resto en orden
# alfabetico, prototipos automaticos) y compila TODO el sketch contra stubs de
# hardware. No reemplaza compilar para el ESP32: prueba que los enganches
# encajan con el firmware nuevo del laboratorio y se encadenan.
sketch:
	bash pruebas_host/build.sh robot/AttaBot pruebas_host/test_v2.cpp
	./pruebas_host/sketch.bin | grep -E "^  (ok|FALLA)|OK|FALLA \("

# El reparto de franjas por mensajes (14-oct-2026) contra el oráculo
# ComputeAssignment, con pérdida, caída, silencio y reincorporación.
reparto:
	$(CXX) $(CXXFLAGS) -o pruebas_host/prueba_reparto.bin pruebas_host/prueba_reparto.cpp
	./pruebas_host/prueba_reparto.bin

observacion: $(LIB)
	$(PY) pruebas_host/prueba_observacion.py

gui: $(LIB)
	$(PY) pruebas_host/render_gui.py 1 95
	$(PY) pruebas_host/render_gui.py 12 90 --caida 0@40

# Las cabeceras viven en firmware/ (fuente de verdad) y estan COPIADAS en
# robot/AttaBot/ para que el IDE de Arduino las compile desde la carpeta del
# sketch. Una divergencia silenciosa = flashear codigo distinto del repositorio.
HDRS := grid.h pheromone_map.h assignment.h bdi.h mission.h telemetry.h

copias:
	@ok=1; for h in $(HDRS); do \
	  if ! cmp -s firmware/$$h robot/AttaBot/$$h; then \
	    echo "  DIFIEREN: firmware/$$h  <>  robot/AttaBot/$$h"; ok=0; fi; done; \
	if [ $$ok -eq 1 ]; then echo "  Las seis cabeceras coinciden."; else exit 1; fi

limpiar:
	rm -rf $(LIB) $(LAB_OBJ) $(LAB_GEN) sitl/__pycache__ pruebas_host/sketch.cpp pruebas_host/sketch.bin
