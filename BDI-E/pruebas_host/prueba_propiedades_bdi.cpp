// Prueba de las cuatro propiedades BDI (Cap. 5, sec. 5.1.6 del informe).
// Corre en el host contra firmware/bdi.h, el MISMO header que se flashea
// (sin dependencias de Arduino). No toca el ciclo: solo lo alimenta.
//   g++ -std=c++17 -O1 -I../firmware prueba_propiedades_bdi.cpp -o prueba_propiedades_bdi && ./prueba_propiedades_bdi
#include <cstdio>
#include <cstring>
#include "bdi.h"
using namespace bdie;

static int fails = 0, oks = 0;
#define CHECK(c, msg) do{ if(c){ printf("  ok    %s\n", msg); oks++; } else { printf("  FALLA %s\n", msg); fails++; } }while(0)

static uint32_t T = 1000;
static uint32_t EP = 1;
static const int ID = 1;

// Lleva al agente a la fase BUSCAR, dentro de su franja, con el cuerpo libre.
static void preparar(Agent& a, Commitment c) {
    GridConfig g;
    a = Agent();
    a.Init(ID, g);
    a.commitment = c;
    a.OnPose(ID, 500, 500, 0, EP++);
    a.StartMission(300, 300, EP, T);
    // Solo en la arena: el reparto se acuerda consigo mismo (franja 0).
    for (int k = 0; k < 60 && !a.HasZone(); ++k) { a.OnPose(ID, 500, 500, 0, EP++); a.Cycle(T += 100); a.bodyIdle = true; }
    // Ponerlo en el centro de su franja.
    int zc = ZoneCenterCell(a.grid, a.assign.zones[a.assign.MyZone(ID)]);
    float x, y; IndexToMm(a.grid, zc, x, y);
    a.OnPose(ID, x, y, 0, EP++);
    a.intention = Intention(); a.bodyIdle = true;
    a.events.Clear();                    // eventos de arranque ya procesados
}

static const Plan* g_lib; static int g_libCount;
static bool ctxObstaculo(const Agent& a) { return a.bel.obstacleFront; }
static bool trLlegada(const Event& e) { return e.type == EV_ARRIVED; }
static void bdNuevo(Agent& a, Intention& I, Action& act) {
    I.planName = "P_NUEVO"; I.active = true; I.goalCell = -1; act.kind = Action::A_HOLD; (void)a;
}
static void bdRodear(Agent& a, Intention& I, Action& act) {
    I.planName = "P_RODEAR"; I.active = true; I.goalCell = -1; act.kind = Action::A_HOLD; (void)a;
}

int main() {
    Agent a;

    printf("Propiedad 1: disparador y contexto se evaluan por separado\n");
    {
        // Biblioteca = la desplegada + un plan con GUARDA DE CONTEXTO propia
        // (obstáculo al frente) y el mismo disparador (llegada a la celda).
        static Plan lib[kPlanCount + 1];
        std::memcpy(lib, kPlanLibrary, sizeof(kPlanLibrary));
        lib[kPlanCount] = { "P_RODEAR", D_EXPLORAR, trLlegada, ctxObstaculo, bdRodear, 50 };

        preparar(a, COMMIT_SINGLE_MINDED);
        a.lib = lib; a.libCount = kPlanCount + 1;
        a.OnSensors(false, false, false, false);
        a.OnArrived(); a.Cycle(T += 100);
        const char* sin = a.intention.planName;
        preparar(a, COMMIT_SINGLE_MINDED);
        a.lib = lib; a.libCount = kPlanCount + 1;
        a.OnSensors(true, false, false, false);
        a.OnArrived(); a.Cycle(T += 100);
        const char* con = a.intention.planName;
        printf("        mismo evento EV_ARRIVED: sin obstaculo -> %s ; con obstaculo -> %s\n", sin, con);
        CHECK(std::strcmp(sin, "P_RODEAR") != 0 && std::strcmp(con, "P_RODEAR") == 0,
              "el mismo evento produce planes distintos segun la creencia (contexto)");
        // Mismo contexto, otro evento: el disparador filtra.
        preparar(a, COMMIT_SINGLE_MINDED);
        a.lib = lib; a.libCount = kPlanCount + 1;
        a.OnSensors(true, false, false, false);
        a.Cycle(T += 100);                       // sin EV_ARRIVED
        CHECK(std::strcmp(a.intention.planName, "P_RODEAR") != 0,
              "con el mismo contexto y sin el evento disparador, el plan no se elige");
    }

    printf("Propiedad 2: agregar un plan no toca el ciclo\n");
    {
        preparar(a, COMMIT_SINGLE_MINDED);
        a.Cycle(T += 100);
        const char* antes = a.intention.planName;
        static Plan lib[kPlanCount + 1];
        std::memcpy(lib, kPlanLibrary, sizeof(kPlanLibrary));
        lib[kPlanCount] = { "P_NUEVO", D_EXPLORAR, plans::trAny, plans::ctxAlways, bdNuevo, 99 };
        preparar(a, COMMIT_SINGLE_MINDED);
        a.lib = lib; a.libCount = kPlanCount + 1;
        a.Cycle(T += 100);
        printf("        misma situacion: tabla original -> %s ; con una fila mas -> %s\n", antes, a.intention.planName);
        CHECK(std::strcmp(antes, "P_NUEVO") != 0 && std::strcmp(a.intention.planName, "P_NUEVO") == 0,
              "una fila nueva en la tabla cambia la decision sin modificar Agent::Cycle");
    }

    printf("Propiedad 3: la intencion persiste y se suelta segun la politica\n");
    {
        for (int pol = 0; pol < 2; ++pol) {
            const Commitment c = pol == 0 ? COMMIT_SINGLE_MINDED : COMMIT_BLIND;
            preparar(a, c);
            a.Cycle(T += 100);
            const int meta = a.intention.goalCell;
            const uint32_t d0 = a.deliberations;
            for (int k = 0; k < 20; ++k) a.Cycle(T += 100);      // 2 s, cuerpo ocupado, sin eventos
            const uint32_t d1 = a.deliberations;
            // Otro robot pisa la celda meta: la feromona la marca visitada.
            float x, y; IndexToMm(a.grid, meta, x, y);
            a.OnPose(2, x, y, 0, EP++);
            a.Cycle(T += 100);
            const uint32_t d2 = a.deliberations;
            printf("        %s: meta=%d, deliberaciones en 20 ciclos sin eventos=%u, al pisar otro la meta=%u, nueva meta=%d\n",
                   pol == 0 ? "SINGLE_MINDED" : "BLIND", meta, d1 - d0, d2 - d1, a.intention.goalCell);
            if (pol == 0) {
                CHECK(meta >= 0 && d1 == d0, "single-minded: 20 ciclos sin eventos, cero deliberaciones (persiste)");
                CHECK(d2 == d1 + 1 && a.intention.goalCell != meta,
                      "single-minded: la suelta cuando la creencia que la motivo deja de valer");
            } else {
                CHECK(d1 == d0 && d2 == d1, "ciego: no la suelta aunque la meta ya este cubierta");
            }
        }
        // La intencion lograda se suelta.
        preparar(a, COMMIT_SINGLE_MINDED);
        a.Cycle(T += 100);
        const int meta = a.intention.goalCell;
        float x, y; IndexToMm(a.grid, meta, x, y);
        const uint32_t d0 = a.deliberations;
        a.OnPose(ID, x, y, 0, EP++); a.OnArrived();
        a.Cycle(T += 100);
        CHECK(a.deliberations == d0 + 1 && a.intention.goalCell != meta,
              "al alcanzar la meta la intencion se suelta y se delibera en el mismo ciclo");
    }

    printf("Propiedad 4: deseos que compiten por prioridad\n");
    {
        preparar(a, COMMIT_BLIND);
        a.Cycle(T += 100);
        const char* antes = a.intention.planName;
        Desire d0 = a.desire;
        a.OnSensors(false, false, false, true);  // bateria baja
        Action act = a.Cycle(T += 100);
        printf("        antes: %s (%s) ; con bateria baja: %s (%s), accion=%d\n",
               antes, kDesireName[d0], a.intention.planName, kDesireName[a.desire], (int)act.kind);
        CHECK(a.desire == D_SEGURO && std::strcmp(a.intention.planName, "P_ABORTAR") == 0 &&
              act.kind == Action::A_ABORT,
              "bateria baja: D_SEGURO interrumpe la intencion aun con compromiso ciego");
        // Objetivo conocido durante la exploracion: CONGREGAR (60) le gana a EXPLORAR (40).
        preparar(a, COMMIT_BLIND);
        a.Cycle(T += 100);
        a.OnTargetReport(a.roster.epoch, 2, 1200, 800, T);
        act = a.Cycle(T += 100);
        printf("        hallazgo ajeno: %s (%s), accion=%d\n", a.intention.planName, kDesireName[a.desire], (int)act.kind);
        CHECK(a.desire == D_CONGREGAR && std::strcmp(a.intention.planName, "P_MEET_LAB") == 0 &&
              act.kind == Action::A_MEET,
              "hallazgo difundido: D_CONGREGAR desplaza a D_EXPLORAR sin caso especial en el plan");
    }

    printf("\n%d verificaciones correctas, %d fallas\n", oks, fails);
    return fails ? 1 : 0;
}
