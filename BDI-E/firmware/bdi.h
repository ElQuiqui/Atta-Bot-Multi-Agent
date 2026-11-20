/*****************************************************************************
 * bdi.h — Ciclo de razonamiento BDI de cada robot
 *
 * Cada robot ejecuta un agente BDI ligero, sin un intérprete genérico como
 * Jason (desproporcionado para un ESP32), pero con la misma estructura:
 * creencias con revisión y envejecimiento, deseos con prioridad, intenciones
 * con compromiso y condición de fracaso, una biblioteca de planes como tabla
 * estática y una política de reconsideración explícita.
 *
 * Cuatro propiedades lo distinguen de una máquina de estados, y las verifica
 * pruebas_host/prueba_propiedades_bdi.cpp:
 *
 *   1. El mismo evento produce acciones distintas según el contexto; en una
 *      máquina de estados la transición estado × evento es fija.
 *   2. Agregar un plan cambia el comportamiento sin tocar el ciclo: la
 *      biblioteca es una tabla y no un switch.
 *   3. La intención persiste entre ciclos: el agente sigue comprometido con lo
 *      que decidió aunque reciba eventos, lo que evita el zigzag.
 *   4. Un deseo de mayor prioridad interrumpe la intención en curso sin casos
 *      especiales repartidos por el código.
 *
 * El agente no reemplaza la máquina de estados de movimiento del firmware:
 * la alimenta. Produce Action, que mission.h traduce a instrucciones de
 * instructionList; el PID, los encoders, la IMU y la evasión no se tocan.
 *
 * Sin dependencias de Arduino.
 *****************************************************************************/
#ifndef BDIE_BDI_H
#define BDIE_BDI_H

#include <cstring>
#include <cmath>
#include "grid.h"
#include "pheromone_map.h"
#include "assignment.h"

namespace bdie {

/*===========================================================================*
 *  Fases de la misión
 *===========================================================================*/
enum Phase : uint8_t {
    PH_IDLE = 0,
    PH_DESPLEGAR,
    PH_BUSCAR,
    PH_CONGREGAR,
    PH_VOLVER,
    PH_FIN,
    PH_SEGURO
};

/*===========================================================================*
 *  Deseos
 *
 *  Se selecciona el de mayor prioridad cuya condición de activación se cumple
 *  sobre las creencias actuales. La prioridad de D_SEGURO hace que una
 *  batería baja interrumpa cualquier intención en curso, sin comprobaciones
 *  repartidas por los cuerpos de plan.
 *===========================================================================*/
enum Desire : uint8_t {
    D_TRANQUILO = 0,
    D_EXPLORAR,
    D_DESPLEGAR,
    D_CONGREGAR,
    D_VOLVER,
    D_SEGURO,
    D_COUNT,
    D_ANY = 0xFF                 // comodín para planes que sirven a cualquier deseo
};

static const uint8_t kDesirePriority[D_COUNT] = { 0, 40, 50, 60, 80, 100 };
static const char* const kDesireName[D_COUNT] = {
    "TRANQUILO", "EXPLORAR", "DESPLEGAR", "CONGREGAR", "VOLVER", "SEGURO"
};

/*===========================================================================*
 *  Eventos y cola
 *===========================================================================*/
enum EventType : uint8_t {
    EV_NONE = 0,
    EV_ARRIVED,              // el cuerpo terminó la instrucción
    EV_TARGET_SEEN,          // el sensor propio confirmó el objetivo (voto 3 de 5)
    EV_TARGET_REPORTED,      // otro robot difundió el hallazgo
    EV_TIMEOUT,              // la intención no progresa
    EV_BATTERY_LOW,
    EV_MISSION_TIMEOUT,
    EV_CONGREGATION_CLOSED,
    EV_SESSION_START,
    EV_SESSION_ABORT,
    EV_ZONE_AGREED           // se acordó un reparto (a = franja propia, b = ronda, c = franja anterior)
};

struct Event {
    EventType type = EV_NONE;
    int   a = 0, b = 0, c = 0;
    float f = 0.0f;
};

struct EventQueue {
    static const int N = 16;
    Event q[N];
    int head = 0, tail = 0;
    void Push(const Event& e) { const int nx = (tail + 1) % N; if (nx == head) return; q[tail] = e; tail = nx; }
    bool Pop(Event& e) { if (head == tail) return false; e = q[head]; head = (head + 1) % N; return true; }
    void Clear() { head = tail = 0; }
    bool Empty() const { return head == tail; }
};

/*===========================================================================*
 *  Mensajes de salida
 *
 *  mission.h los convierte a texto, con cinco campos como máximo para caber
 *  en el analizador de comandos del firmware (prueba T10).
 *
 *  MSG_ZC / MSG_ZA: reclamo de franja y franja acordada; a = ronda, b = id,
 *  c = franja, d = distancia, e = participantes.
 *===========================================================================*/
enum MsgKind : uint8_t { MSG_NONE = 0, MSG_HB, MSG_TGT, MSG_TGT_REQ, MSG_LOG,
                         MSG_ZC, MSG_ZA };

struct Msg {
    MsgKind  kind = MSG_NONE;
    int32_t  a = 0, b = 0, c = 0, d = 0;
    uint32_t e = 0;
    float    f1 = 0.0f, f2 = 0.0f;
};

struct MsgQueue {
    static const int N = 8;
    Msg q[N];
    int head = 0, tail = 0;
    void Push(const Msg& m) { const int nx = (tail + 1) % N; if (nx == head) return; q[tail] = m; tail = nx; }
    bool Pop(Msg& m) { if (head == tail) return false; m = q[head]; head = (head + 1) % N; return true; }
};

/*===========================================================================*
 *  Compromiso: el eje audaz ↔ cauteloso de Kinny y Georgeff
 *
 *  Una compuerta de reconsideración muy abierta vuelve al agente errático y
 *  sin progreso; una muy cerrada lo deja persiguiendo intenciones que ya no
 *  tienen sentido. Se ajusta en el banco de simulación.
 *===========================================================================*/
enum Commitment : uint8_t {
    COMMIT_BLIND = 0,        // solo al terminar o fracasar
    COMMIT_SINGLE_MINDED,    // además, si la creencia que la motivó dejó de valer (por defecto)
    COMMIT_OPEN_MINDED       // además, si otra opción la supera por un margen δ
};

/*===========================================================================*
 *  Acciones: lo único que el agente le entrega al cuerpo
 *===========================================================================*/
struct Action {
    /* A_MEET: congregarse alrededor de (x, y) con el MEET del laboratorio
     * (HandleMeet en comandos.ino). El agente decide cuándo y dónde; el reparto de
     * ranuras, el anillo, la aproximación y la llegada son código del laboratorio
     * sin cambios. Ver la sección de congregación de bdie.ino. */
    enum Kind : uint8_t { A_NONE = 0, A_GOTO_CELL, A_GOTO_MM, A_HOLD, A_ABORT, A_MEET } kind = A_NONE;
    int   cell = -1;
    float x = 0.0f, y = 0.0f;
};

struct Intention {
    const char* planName = "";
    Desire      desire   = D_TRANQUILO;
    int         goalCell = -1;
    float       goalX = 0.0f, goalY = 0.0f;
    bool        goalIsMm = false;        // la meta es un punto en mm, no el centro de una celda
    bool        active   = false;
    uint32_t    tStart   = 0;

    /* Detección de falta de progreso: mejor distancia a la meta y momento en que
     * mejoró por última vez. El reloj de fracaso vive en la intención y no en el
     * plan: si se reiniciara al cambiar de plan, dos planes que se alternan lo
     * dejarían siempre en cero y el robot nunca se daría por atascado. */
    int      bestDist  = 1 << 20;
    uint32_t tProgress = 0;
    int      retries   = 0;
};

struct PeerState {
    uint8_t  phase       = PH_IDLE;
    bool     knowsTarget = false;
    bool     arrived     = false;
    bool     heard       = false;
    uint32_t tLast       = 0;
};

class Agent;

struct Plan {
    const char* name;
    Desire      desire;                       // a qué deseo sirve; D_ANY = a cualquiera
    bool (*trigger)(const Event&);            // evento disparador
    bool (*context)(const Agent&);            // guarda de contexto, independiente del disparador
    void (*body)(Agent&, Intention&, Action&);
    uint8_t     priority;                     // desempate entre planes del mismo deseo
};

/*===========================================================================*
 *  El agente
 *===========================================================================*/
class Agent {
public:
    // ── Identidad y mundo ─────────────────────────────────────────────────
    int          selfId = 0;
    GridConfig   grid;
    PheromoneMap map;
    Roster       roster;
    Assignment   assign;        // el reparto vigente (acordado, o el de respaldo)
    Reparto      rp;            // la ronda en curso y los reclamos oídos (assignment.h)

    // ── Parámetros ajustables ─────────────────────────────────────────────
    Commitment commitment        = COMMIT_SINGLE_MINDED;
    float      openMindedDelta   = 0.15f;
    uint32_t   noProgressMs      = 12000;     // ventana de falta de progreso
    uint32_t   congregationMs    = 300000;    // techo de la congregación: 5 min desde el primer arribo
    uint32_t   missionTimeoutMs  = 1200000;   // 20 min: tiempo límite de la misión
    uint32_t   bdiPeriodMs       = 100;       // 10 Hz
    uint32_t   hbPeriodMs        = 500;       // 2 Hz
    /* Un robot está vivo si se oyó de él un mensaje propio (HB, ZC, ZA o
     * TGT_REQ) en este lapso: seis latidos a 2 Hz. Con 5 % de pérdida, perder seis
     * latidos seguidos tiene probabilidad 1,6·10⁻⁸. */
    uint32_t   peerDeadMs        = 3000;
    /* Período de evaporación de la feromona. El ritmo de evaporación se
     * controla con el período y no con rho (ver la nota de cuantización en
     * pheromone_map.h). Con 16 s, un depósito de 120 tarda unos 16 min en
     * desaparecer, lo que cubre la fase de búsqueda en los robots físicos, que
     * es varias veces más lenta que en el banco. NAV_CONFIG|EVAP|ms lo cambia en
     * vivo. */
    uint32_t   evaporatePeriodMs = 16000;     // ver pheromone_map.h
    bool       frontierEnabled   = true;      // búsqueda de frontera dentro de P_EXPLORAR_ACO
    /* Bloqueo temporal de una meta inalcanzable. El objetivo ocupa una celda a
     * la que ningún robot puede entrar con su centro; esa celda nunca queda
     * visitada, y la búsqueda de frontera la elegiría una y otra vez. Cuando la
     * intención de buscar se queda sin progreso (EV_TIMEOUT), su celda meta se
     * bloquea durante tempBlockMs y la ruleta y la frontera la saltan. El bloqueo
     * es local a cada robot y se libera solo; no afecta a las celdas bloqueadas
     * por obstáculos. */
    uint32_t   tempBlockMs       = 60000;
    float      returnDoneMm      = 250.0f;   // FIN: distancia a la base para terminar
    float      returnPeerMm      = 500.0f;
    float      returnSlotMm      = 170.0f;   // radio del anillo de lugares en la base (0 = todos al mismo punto)
    uint32_t   tgtRepeatMs       = 1000;      // reenvío epidémico del hallazgo
    uint32_t   formationWaitMs   = 8000;      // tope de espera al rezagado


    float      formationMinMm    = 350.0f;    // separación mínima entre vecinos
    float      formationMaxMm    = 900.0f;    // banda de rezago tolerada en el retorno
    /* Criterio para esperar en el retorno:
     *   0 · vecino más cercano: no acota el radio del enjambre, porque con los
     *       robots repartidos cada uno tiene a alguien cerca y nadie espera.
     *   1 · cota de radio (por defecto): se espera mientras algún robot vivo esté
     *       más lejos de la base que el propio por más de formationMaxMm. Acota
     *       directamente el radio del enjambre y se calcula sobre la difusión de
     *       posiciones, sin mensajes. */
    int        formationMode     = 1;
    /* Gracia antes de emprender el retorno. El radio del enjambre en el retorno
     * lo fija sobre todo el desfase con que cada robot cierra la congregación: el
     * que cierra primero arranca mientras el último sigue quieto. Una espera fija
     * de unos segundos corrige ese desfase sin crear una barrera de
     * sincronización, porque no depende de que nadie avise. */
    uint32_t   returnGraceMs     = 3000;

    // ── Creencias ─────────────────────────────────────────────────────────
    struct Beliefs {
        float x = 0.0f, y = 0.0f, th = 0.0f;
        int   myCell = -1;
        Phase phase  = PH_IDLE;

        bool  session     = false;
        float baseX = 0.0f, baseY = 0.0f;
        int   baseCell    = -1;

        bool  targetKnown = false;
        int   targetOwner = -1;
        int   targetCell  = -1;
        float targetX = 0.0f, targetY = 0.0f;
        bool  iFoundIt    = false;

        int   myZone      = -1;
        bool  zoneReached = false;

        /* La congregación la ejecuta el MEET del laboratorio. El agente recuerda a
         * qué punto lo pidió: si el objetivo cambia (desempate por ID), el MEET en
         * curso queda viejo y hay que pedir otro. */
        bool  meetIssued         = false;
        float meetX = 0.0f, meetY = 0.0f;
        /* Lo escribe OnSlotReached(), es decir, el cuerpo: el laboratorio declara
         * la llegada cuando la navegación alcanza la ranura. */
        bool  arrivedAtPlace     = false;
        bool  congregationClosed = false;
        bool  batteryLow         = false;
        bool  obstacleFront      = false;

        PeerState peer[MAX_ROBOTS];
    } bel;

    // ── Estado interno del ciclo ──────────────────────────────────────────
    EventQueue events;
    MsgQueue   out;
    Intention  intention;
    Desire     desire   = D_TRANQUILO;
    bool       bodyIdle = true;               // ¿la cola de instrucciones está vacía?
    uint32_t   now = 0;

    const char* lastPlan   = "";
    /* Instrumentación (telemetry.h): último evento que percibió el ciclo. Solo
     * sirve para observar por qué el agente reconsideró; ninguna decisión lo lee. */
    uint8_t     lastEvent  = 0;          // EventType
    uint32_t    tLastEvent = 0;
    const char* lastDesire = "TRANQUILO";
    uint32_t    deliberations = 0;

    const Plan* lib      = 0;                 // biblioteca de planes; inyectable (propiedad 2)
    int         libCount = 0;

    uint32_t rng = 1;

    // ── Relojes ───────────────────────────────────────────────────────────
    uint32_t tMission       = 0;
    uint32_t tEvaporate     = 0;
    struct TempBlock { int cell; uint32_t until; };
    TempBlock tempBlock[8];
    void TempBlockClear() { for (int i = 0; i < 8; ++i) tempBlock[i].cell = -1; }
    void TempBlockCell(int k, uint32_t now) {
        if (k < 0 || map.Blocked(k)) return;          // ya bloqueada, temporal o por obstáculo
        int slot = 0;
        for (int i = 0; i < 8; ++i) {
            if (tempBlock[i].cell < 0) { slot = i; break; }
            if (tempBlock[i].until < tempBlock[slot].until) slot = i;   // la más vieja
        }
        if (tempBlock[slot].cell >= 0) map.Unblock(tempBlock[slot].cell);
        tempBlock[slot].cell = k; tempBlock[slot].until = now + tempBlockMs;
        map.Block(k);
    }
    void TempBlockExpire(uint32_t now) {
        for (int i = 0; i < 8; ++i)
            if (tempBlock[i].cell >= 0 && (int32_t)(now - tempBlock[i].until) >= 0) {
                map.Unblock(tempBlock[i].cell);
                tempBlock[i].cell = -1;
            }
    }
    uint32_t tHeartbeat     = 0;
    uint32_t tTargetRepeat  = 0;
    uint32_t tFirstArrival  = 0;
    uint32_t tFormationWait = 0;
    uint32_t tClosed        = 0;
    uint32_t hbSeq          = 0;
    uint32_t sessionId      = 0xFFFFFFFFu;   // número de sesión del último START aceptado

    int lastCellOf[MAX_ROBOTS];

    // ── Construcción ──────────────────────────────────────────────────────
    Agent() { for (int i = 0; i < MAX_ROBOTS; ++i) lastCellOf[i] = -1; }

    void Init(int id, const GridConfig& g);

    void StartMission(float baseXmm, float baseYmm, uint32_t epoch, uint32_t nowMs);
    void ResetMissionMemory();
    /* Celda del propio robot. Si el robot está apenas fuera de la cuadrícula
     * (hasta 300 mm), toma la celda del borde más cercana, para que la exploración
     * lo traiga de vuelta adentro en vez de quedarse sin vecinas que elegir. */
    int  CeldaPropia(float x, float y) const {
        const int k = MmToIndex(grid, x, y);
        if (k >= 0) return k;
        const float m = 300.0f;
        const float x0 = grid.originXmm - m, x1 = grid.originXmm + grid.cols * grid.cellMm + m;
        const float y0 = grid.originYmm - m, y1 = grid.originYmm + grid.rows * grid.cellMm + m;
        if (x < x0 || x > x1 || y < y0 || y > y1) return -1;
        return MmToIndexClamped(grid, x, y);
    }
    void AbortMission(uint32_t nowMs);

    // ── Entradas ──────────────────────────────────────────────────────────
    void OnPose(int id, float x, float y, float th, uint32_t epoch);
    void OnHeartbeat(int id, uint8_t phase, uint8_t flags, uint32_t seq, uint32_t nowMs);
    /* Un reclamo (ZC) o un reparto acordado (ZA) de un par: se tratan igual. */
    void OnZoneClaim(uint32_t rep, uint32_t mask, int id, int zone, int dist, uint32_t nowMs);
    /* Se oyó un mensaje propio del par `id`: está vivo. */
    void Heard(int id, uint32_t nowMs);
    /* Paso 6 del ciclo (latido, reenvío epidémico y reparto), con reloj propio.
     * Cycle() lo llama, y el sketch lo llama también desde loop(), para que el
     * latido no dependa de que el cuerpo esté libre. */
    void Comunicar(uint32_t nowMs);
    void OnTargetSeen(uint32_t nowMs);
    void OnTargetReport(uint32_t epoch, int ownerId, float tx, float ty, uint32_t nowMs);
    void OnTargetRequest(int fromId, uint32_t nowMs);
    void OnSensors(bool front, bool left, bool right, bool batteryLow);
    void OnArrived();
    /* El MEET del laboratorio llegó a la ranura (enganche en la navegación). */
    void OnSlotReached();

    // ── El ciclo ──────────────────────────────────────────────────────────
    Action Cycle(uint32_t nowMs);

    // ── Consultas que usan los planes y las pruebas ───────────────────────
    bool  HasZone()  const { return bel.myZone >= 0; }
    /* ¿El MEET en curso apunta a un objetivo que ya no es el vigente? */
    bool  MeetStale() const {
        return bel.meetIssued && (bel.meetX != bel.targetX || bel.meetY != bel.targetY);
    }
    /* Un robot está vivo si se le oye; el propio robot, siempre. */
    bool  Talking(int id, uint32_t t) const;
    uint32_t TalkingMask(uint32_t t) const;
    /* Participantes del reparto: los robots que se oyen y están buscando (el
     * propio incluido, si busca). Un robot del que aún no llegó ningún latido tiene
     * fase desconocida y cuenta como participante. */
    uint32_t MembersMask(uint32_t t) const;
    bool  Buscando() const {
        return bel.session && !bel.targetKnown &&
               (bel.phase == PH_DESPLEGAR || bel.phase == PH_BUSCAR);
    }
    float NearestPeerMm(int* whoOut = 0) const;
    float RearPeerToBaseMm() const;      // distancia a la base del robot vivo más lejano
    void  ZoneMaskInto(uint8_t* m) const;
    uint8_t HbFlags() const;
    bool  QuorumReached() const;

    int   AliveCount() const { return roster.AliveCount(); }

    bool DesireActive(Desire d) const;
    Desire SelectDesire() const;
    bool ShouldReconsider(const Event& ev, Desire want) const;

    // ── Reparto por mensajes (assignment.h) ───────────────────────────────
    /* Corre dentro de Comunicar(), es decir, en cada loop() del robot y no solo
     * al deliberar: una ronda no puede esperar a que el cuerpo quede libre. */
    void Repartir(uint32_t t);
    void AbrirRonda(uint32_t rep, uint32_t mask, uint32_t t);
    void ActualizarReclamo(uint32_t t);
    void Adoptar(uint32_t t);

    // ── Utilidades para los cuerpos de plan ───────────────────────────────
    void GotoCell(Action& a, int cell) {
        a.kind = Action::A_GOTO_CELL;
        a.cell = cell;
        IndexToMm(grid, cell, a.x, a.y);
    }
    void GotoMm(Action& a, float x, float y) {
        a.kind = Action::A_GOTO_MM;
        a.cell = MmToIndex(grid, x, y);
        a.x = x; a.y = y;
    }
    void Meet(Action& a, float x, float y) {
        a.kind = Action::A_MEET;
        a.cell = MmToIndexClamped(grid, x, y);
        a.x = x; a.y = y;
    }
};

/*===========================================================================*
 *  Disparadores y contextos
 *
 *  Son predicados independientes: el disparador mira el evento y el contexto
 *  mira las creencias, y la combinación hace que el mismo evento produzca
 *  acciones distintas (propiedad 1).
 *===========================================================================*/
namespace plans {

inline bool trAny     (const Event& e) { (void)e; return true; }
inline bool trTimeout (const Event& e) { return e.type == EV_TIMEOUT; }

inline bool ctxAlways (const Agent& a) { (void)a; return true; }

}  // namespace plans

/*===========================================================================*
 *  Los siete cuerpos de plan
 *===========================================================================*/
namespace plans {

/* P_ESPERAR — D_TRANQUILO.
 * Es también la espera de la congregación: el robot que ya llegó a su ranura
 * deja de tener D_CONGREGAR activo y cae a D_TRANQUILO. */
inline void bdEsperar(Agent& a, Intention& I, Action& act) {
    (void)a;
    I.planName = "P_ESPERAR";
    I.goalCell = -1;
    I.active   = true;
    act.kind   = Action::A_HOLD;
}

/* P_IR_A_ZONA — D_DESPLEGAR. */
inline void bdIrAZona(Agent& a, Intention& I, Action& act) {
    I.planName = "P_IR_A_ZONA";
    I.active   = true;
    int goal = -1;
    if (a.HasZone()) goal = ZoneCenterCell(a.grid, a.assign.zones[a.bel.myZone]);
    if (goal < 0) { act.kind = Action::A_HOLD; I.goalCell = -1; return; }
    I.goalCell = goal;
    I.goalIsMm = false;
    a.GotoCell(act, goal);
}

/* P_EXPLORAR_ACO — D_EXPLORAR.
 * Ruleta sobre las ocho vecinas, restringida a la zona propia. Si la zona se
 * agotó, se amplía a la arena completa: quedarse quieto en una zona barrida es
 * peor que ayudar en la del vecino. */
inline void bdExplorarAco(Agent& a, Intention& I, Action& act) {
    I.planName = "P_EXPLORAR_ACO";
    I.active   = true;
    uint8_t mask[MAX_CELLS];
    a.ZoneMaskInto(mask);
    const uint8_t* zm = a.HasZone() ? mask : 0;

    /* Frontera: si ninguna vecina elegible está sin visitar, la ruleta solo
     * puede elegir celdas ya miradas y el robot daría vueltas sobre lo barrido. En
     * ese caso va directo a la celda no visitada más cercana, primero de su zona y,
     * si la zona está completa, de toda la arena. La intención queda comprometida
     * con esa celda hasta llegar o hasta que otro robot la pise
     * (ShouldReconsider), así que no se vuelve a sortear en cada ciclo. */
    if (a.frontierEnabled && a.bel.myCell >= 0 &&
        !a.map.HasUnvisitedNeighbor(a.grid, a.bel.myCell, zm)) {
        int f = a.map.NearestUnvisited(a.grid, a.bel.myCell, zm, a.rng);
        if (f < 0 && zm) f = a.map.NearestUnvisited(a.grid, a.bel.myCell, 0, a.rng);
        if (f >= 0) {
            I.planName = "P_IR_A_FRONTERA";
            I.goalCell = f;
            I.goalIsMm = false;
            a.GotoCell(act, f);
            return;
        }
    }

    int k = a.map.NextCell(a.grid, a.bel.myCell, a.bel.th, a.HasZone() ? mask : 0, a.rng);
    if (k < 0) k = a.map.NextCell(a.grid, a.bel.myCell, a.bel.th, 0, a.rng);
    if (k < 0) { act.kind = Action::A_HOLD; I.goalCell = -1; return; }
    I.goalCell = k;
    I.goalIsMm = false;
    a.GotoCell(act, k);
}

/* P_MEET_LAB — D_CONGREGAR.
 * La congregación la resuelve el MEET del laboratorio (HandleMeet en
 * comandos.ino; MeetSlotIndex, SafeRingSlotAngle y UpdateCongregationGoal en
 * navegacion.ino). Cada robot obtiene su ranura del anillo a partir de la
 * lista de vecinos NEIGHBOR_POSITIONS, sin líder y sin que la base decida, y
 * el laboratorio declara la llegada. El agente aporta cuándo (al conocer el
 * objetivo) y dónde (el punto del hallazgo).
 *
 * Todos los robots, incluido el descubridor, ejecutan el mismo plan. No hay
 * meta de celda (goalCell = -1): mientras el laboratorio navega, el cuerpo
 * está ocupado y el agente no delibera. Si el cuerpo queda libre sin haber
 * llegado (por tiempo máximo de la navegación, o porque el objetivo cambió),
 * este mismo plan vuelve a pedir el MEET. */
inline void bdCongregarLab(Agent& a, Intention& I, Action& act) {
    I.planName = "P_MEET_LAB";
    I.goalCell = -1;
    I.active   = true;
    a.bel.meetIssued = true;
    a.bel.meetX = a.bel.targetX;
    a.bel.meetY = a.bel.targetY;
    a.Meet(act, a.bel.targetX, a.bel.targetY);
}

/* P_VOLVER_FORM — D_VOLVER.
 * Retorno en formación laxa: sin líder ni formación rígida, cada robot navega
 * a la base corrigiendo su meta según su distancia a los vecinos, calculada
 * sobre la difusión de posiciones. La espera al rezagado tiene tope, para que
 * un robot atascado no detenga a los demás. */
inline void bdVolverForm(Agent& a, Intention& I, Action& act) {
    I.planName = "P_VOLVER_FORM";
    I.active   = true;

    /* Gracia de arranque: deja que los rezagados cierren antes de salir. */
    if (a.returnGraceMs > 0 && a.tClosed != 0 && (a.now - a.tClosed) < a.returnGraceMs) {
        I.goalCell = -1;
        act.kind = Action::A_HOLD;
        return;
    }

    int who = -1;
    const float d = a.NearestPeerMm(&who);

    /* ¿Hay que esperar? Depende de formationMode: el modo 0 mira al vecino más
     * cercano y el modo 1, al robot que va más atrás. */
    bool esperar = false;
    if (a.formationMode == 0) {
        esperar = (who >= 0 && d > a.formationMaxMm);
    } else {
        const float miDist   = std::sqrt((a.bel.x - a.bel.baseX) * (a.bel.x - a.bel.baseX) +
                                         (a.bel.y - a.bel.baseY) * (a.bel.y - a.bel.baseY));
        const float rezagado = a.RearPeerToBaseMm();
        esperar = (rezagado >= 0.0f) && ((rezagado - miDist) > a.formationMaxMm);
    }

    if (esperar) {
        if (a.tFormationWait == 0) a.tFormationWait = a.now;
        /* La espera tiene tope: sin él, un robot atascado detendría a los demás. */
        if ((a.now - a.tFormationWait) < a.formationWaitMs) {
            I.goalCell = -1;
            act.kind = Action::A_HOLD;
            return;
        }
    } else {
        a.tFormationWait = 0;
    }

    float gx = a.bel.baseX, gy = a.bel.baseY;
    /* Lugar propio en la base: cada robot va a un lugar distinto en un anillo de
     * returnSlotMm alrededor de la base (ángulo según su ID, 72° entre lugares:
     * hasta cinco robots sin repetir), para no estorbarse al llegar. Como
     * returnSlotMm es menor que returnDoneMm, llegar al lugar cuenta para FIN. */
    const float dBase = std::sqrt((a.bel.x - a.bel.baseX) * (a.bel.x - a.bel.baseX) +
                                  (a.bel.y - a.bel.baseY) * (a.bel.y - a.bel.baseY));
    if (a.returnSlotMm > 0.0f) {
        const float ang = (float)(a.selfId % 5) * 72.0f * 3.14159265f / 180.0f;
        gx += a.returnSlotMm * std::cos(ang);
        gy += a.returnSlotMm * std::sin(ang);
        const float m = 60.0f;
        const float x0 = a.grid.originXmm + m, x1 = a.grid.originXmm + a.grid.cols * a.grid.cellMm - m;
        const float y0 = a.grid.originYmm + m, y1 = a.grid.originYmm + a.grid.rows * a.grid.cellMm - m;
        gx = gx < x0 ? x0 : (gx > x1 ? x1 : gx);
        gy = gy < y0 ? y0 : (gy > y1 ? y1 : gy);
    }
    /* Cerca de la base manda el anillo: la separación mínima de la formación
     * empujaría a los vecinos fuera de sus lugares. */
    const bool lejosDeBase = a.returnSlotMm <= 0.0f || dBase > 2.0f * a.returnDoneMm;
    if (lejosDeBase && who >= 0 && d < a.formationMinMm && d > 1.0f) {
        // demasiado cerca: desplazar la meta alejándose del vecino
        const float dx = a.bel.x - a.roster.p[who].x;
        const float dy = a.bel.y - a.roster.p[who].y;
        const float n  = std::sqrt(dx * dx + dy * dy);
        if (n > 1.0f) {
            gx += (dx / n) * (a.formationMinMm - d);
            gy += (dy / n) * (a.formationMinMm - d);
        }
    }
    I.goalX = gx; I.goalY = gy; I.goalIsMm = true;
    I.goalCell = MmToIndexClamped(a.grid, gx, gy);
    a.GotoMm(act, gx, gy);
}

/* P_REINTENTAR — cualquier deseo, disparado por falta de progreso.
 * Suelta la intención, desplaza la meta y vuelve a deliberar. Después de tres
 * intentos deja de restringirse a la zona propia. */
inline void bdReintentar(Agent& a, Intention& I, Action& act) {
    I.planName = "P_REINTENTAR";
    I.retries += 1;
    I.active   = false;                      // fuerza deliberación en el próximo ciclo
    I.bestDist = 1 << 20;
    I.tProgress = a.now;
    if (a.bel.myCell >= 0) {
        int nb[8];
        const int m = Neighbors8(a.grid, a.bel.myCell, nb);
        if (m > 0) {
            const int pick = (int)(XorShift32(a.rng) % (uint32_t)m);
            I.goalCell = nb[pick];
            a.GotoCell(act, nb[pick]);
            return;
        }
    }
    act.kind = Action::A_HOLD;
}

/* P_ABORTAR — D_SEGURO. Limpia la cola y para. */
inline void bdAbortar(Agent& a, Intention& I, Action& act) {
    I.planName = "P_ABORTAR";
    I.goalCell = -1;
    I.active   = true;
    a.bel.phase = PH_SEGURO;
    act.kind = Action::A_ABORT;
}

}  // namespace plans

/*===========================================================================*
 *  Biblioteca de planes
 *
 *  Es una tabla y no un switch (propiedad 2): agregar un comportamiento es
 *  agregar una fila, y el ciclo no cambia.
 *===========================================================================*/
static const Plan kPlanLibrary[] = {
    /* nombre            deseo         disparador          contexto             cuerpo                 prio */
    { "P_REINTENTAR",    D_ANY,        plans::trTimeout,   plans::ctxAlways,    plans::bdReintentar,    90 },
    { "P_ABORTAR",       D_SEGURO,     plans::trAny,       plans::ctxAlways,    plans::bdAbortar,       10 },
    { "P_VOLVER_FORM",   D_VOLVER,     plans::trAny,       plans::ctxAlways,    plans::bdVolverForm,    10 },
    { "P_MEET_LAB",      D_CONGREGAR,  plans::trAny,       plans::ctxAlways,    plans::bdCongregarLab,  10 },
    { "P_IR_A_ZONA",     D_DESPLEGAR,  plans::trAny,       plans::ctxAlways,    plans::bdIrAZona,       10 },
    { "P_EXPLORAR_ACO",  D_EXPLORAR,   plans::trAny,       plans::ctxAlways,    plans::bdExplorarAco,   10 },
    { "P_ESPERAR",       D_TRANQUILO,  plans::trAny,       plans::ctxAlways,    plans::bdEsperar,       10 },
};
static const int kPlanCount = (int)(sizeof(kPlanLibrary) / sizeof(kPlanLibrary[0]));

/*===========================================================================*
 *  Implementación
 *===========================================================================*/

inline void Agent::Init(int id, const GridConfig& g) {
    selfId = id;
    grid   = g;
    map.Reset(g.Cells());
    roster = Roster();
    assign = Assignment();
    rp     = Reparto();
    bel    = Beliefs();
    events.Clear();
    intention = Intention();
    lib      = kPlanLibrary;
    libCount = kPlanCount;
    /* Semilla derivada del ID: dos robots no sortean igual (si lo hicieran, la
     * ruleta dejaría de dispersarlos) y una corrida se puede repetir exacta. */
    TempBlockClear();
    rng = 0x9E3779B9u ^ (uint32_t)(id * 2654435761u);
    if (rng == 0) rng = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) lastCellOf[i] = -1;
}

/* Borra la memoria de una corrida: al iniciar una sesión, al abortarla y al
 * terminarla con éxito (FIN). Se conservan la pose, la batería, qué robots
 * están vivos y las celdas bloqueadas por obstáculos. */
inline void Agent::ResetMissionMemory() {
    /* Cada sesión empieza de cero: feromona, cobertura, objetivo, llegada,
     * cierre de la congregación y lo que se sabe de los demás robots. Sin esto, una
     * segunda misión arrancaría creyendo conocer el objetivo anterior y con el
     * quórum ya cumplido. */
    for (int i = 0; i < 8; ++i)                      // bloqueos temporales de la sesión anterior
        if (tempBlock[i].cell >= 0) { map.Unblock(tempBlock[i].cell); tempBlock[i].cell = -1; }
    {
        uint32_t bloqueadas[VISITED_WORDS];
        std::memcpy(bloqueadas, map.blocked, sizeof(bloqueadas));
        map.Reset(grid.Cells());                     // feromona y cobertura a cero
        std::memcpy(map.blocked, bloqueadas, sizeof(bloqueadas));

        Beliefs prev = bel;
        bel = Beliefs();
        bel.x = prev.x; bel.y = prev.y; bel.th = prev.th; bel.myCell = prev.myCell;
        bel.batteryLow = prev.batteryLow;
        for (int i = 0; i < MAX_ROBOTS; ++i) {
            bel.peer[i] = prev.peer[i];
            bel.peer[i].knowsTarget = false;
            bel.peer[i].arrived     = false;
        }
    }
    events.Clear();
    intention      = Intention();
    desire         = D_TRANQUILO;
    bodyIdle       = true;
    tFirstArrival  = 0;
    tClosed        = 0;
    tFormationWait = 0;
    tTargetRepeat  = 0;
    for (int i = 0; i < MAX_ROBOTS; ++i) lastCellOf[i] = -1;
}

inline void Agent::StartMission(float baseXmm, float baseYmm, uint32_t epoch, uint32_t nowMs) {
    /* La base envía el START tres veces seguidas (UDP), y BASE.SESSION|JOIN lo
     * reenvía a un robot que se reinició. Si es la misma sesión que ya está en
     * curso, no se toca nada. */
    if (bel.session && epoch == sessionId &&
        bel.phase != PH_SEGURO && bel.phase != PH_FIN && bel.phase != PH_IDLE) return;
    sessionId = epoch;

    ResetMissionMemory();

    bel.session  = true;
    bel.baseX    = baseXmm;
    bel.baseY    = baseYmm;
    bel.baseCell = MmToIndexClamped(grid, baseXmm, baseYmm);
    bel.phase    = PH_DESPLEGAR;
    tMission     = nowMs ? nowMs : 1;
    tEvaporate   = nowMs;
    roster.epoch = epoch;
    /* El reparto empieza de cero en cada sesión: el robot no tiene franja hasta
     * que se acuerde la primera ronda (P_IR_A_ZONA espera quieto mientras tanto). */
    rp.ResetState();
    assign = Assignment();
    bel.myZone      = -1;
    bel.zoneReached = false;
    Event e; e.type = EV_SESSION_START; events.Push(e);
}

inline void Agent::AbortMission(uint32_t nowMs) {
    (void)nowMs;
    ResetMissionMemory();                  // abortar también limpia la corrida
    Event e; e.type = EV_SESSION_ABORT; events.Push(e);
    bel.session = false;
    bel.phase   = PH_SEGURO;
}

/* Pose recibida de la difusión. Se deposita feromona por todos los robots
 * vistos, no solo por el propio, y solo al cambiar de celda: a 10 Hz, un
 * depósito por paquete saturaría la celda en menos de un segundo y el mapa
 * dejaría de discriminar. */
inline void Agent::OnPose(int id, float x, float y, float th, uint32_t epoch) {
    roster.OnPose(id, x, y, th, epoch);
    const int k = MmToIndex(grid, x, y);
    /* La celda propia se actualiza con cada pose, no solo al deliberar, para que
     * la telemetría la muestre bien mientras el robot navega. */
    if (id == selfId) { bel.x = x; bel.y = y; bel.th = th; bel.myCell = CeldaPropia(x, y); }
    if (k >= 0 && k != lastCellOf[id]) {
        /* Solo se deposita con una sesión abierta, para que una corrida no arranque
         * con rastro de la anterior. */
        if (bel.session) map.Deposit(k);
        lastCellOf[id] = k;
    }
}

inline void Agent::OnHeartbeat(int id, uint8_t phase, uint8_t flags, uint32_t seq, uint32_t nowMs) {
    (void)seq;
    if (id < 0 || id >= MAX_ROBOTS) return;
    PeerState& p = bel.peer[id];
    p.phase       = phase;
    p.knowsTarget = (flags & 0x01) != 0;
    p.arrived     = (flags & 0x02) != 0;
    Heard(id, nowMs);
    if (p.arrived && tFirstArrival == 0) tFirstArrival = nowMs;
}

inline void Agent::Heard(int id, uint32_t nowMs) {
    if (id < 0 || id >= MAX_ROBOTS || id == selfId) return;
    bel.peer[id].heard = true;
    bel.peer[id].tLast = nowMs ? nowMs : 1;
}

/* La resta va con signo: un mensaje que llegó después del `now` del último
 * ciclo tiene tLast > t, y en aritmética sin signo eso daría unos 49 días de
 * silencio. */
inline bool Agent::Talking(int id, uint32_t t) const {
    if (id == selfId) return true;
    if (id < 0 || id >= MAX_ROBOTS) return false;
    const PeerState& p = bel.peer[id];
    if (!p.heard) return false;
    const int32_t dt = (int32_t)(t - p.tLast);
    return dt < 0 || (uint32_t)dt <= peerDeadMs;
}

inline uint32_t Agent::TalkingMask(uint32_t t) const {
    uint32_t m = 0;
    for (int i = 0; i < MAX_ROBOTS; ++i) if (Talking(i, t)) m |= (1u << i);
    return m;
}

inline uint32_t Agent::MembersMask(uint32_t t) const {
    if (!Buscando()) return 0;
    uint32_t m = (selfId >= 0 && selfId < MAX_ROBOTS) ? (1u << selfId) : 0u;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
        if (i == selfId || !Talking(i, t)) continue;
        const uint8_t f = bel.peer[i].phase;
        if (f == PH_IDLE || f == PH_DESPLEGAR || f == PH_BUSCAR) m |= (1u << i);
    }
    return m;
}

/* ── Reparto por mensajes ─────────────────────────────────────────────── */

inline void Agent::OnZoneClaim(uint32_t rep, uint32_t mask, int id, int zone, int dist,
                               uint32_t t) {
    if (id < 0 || id >= MAX_ROBOTS || id == selfId) return;
    Heard(id, t);
    if (!bel.session) return;
    if (rep > rp.seenMax) rp.seenMax = rep;
    ZoneClaim& c = rp.peer[id];
    const bool cambio = !c.valid || c.rep != rep || c.mask != mask ||
                        c.zone != zone || c.dist != dist;
    c.rep = rep; c.mask = mask; c.zone = zone; c.dist = dist; c.valid = true;
    if (!Buscando()) return;
    if (rep > rp.rep) {               // ronda más nueva: el robot se pasa a ella
        AbrirRonda(rep, MembersMask(t), t);
        return;
    }
    if (rep < rp.rep) {               // el otro va atrasado: que oiga pronto la ronda propia
        rp.sendNow = true;
        return;
    }
    if (cambio) {
        rp.tChange = t ? t : 1;
        ActualizarReclamo(t);
    }
}

inline void Agent::AbrirRonda(uint32_t rep, uint32_t mask, uint32_t t) {
    rp.rep     = rep;
    if (rep > rp.seenMax) rp.seenMax = rep;
    rp.mask    = mask;
    rp.anchor  = MmToIndexClamped(grid, bel.x, bel.y);    // ancla congelada para toda la ronda
    rp.adopted = false;
    rp.zone    = -1;
    rp.dist    = 0;
    rp.tChange = t ? t : 1;
    rp.sendNow = true;
    ++rp.rounds;
    ActualizarReclamo(t);
}

inline void Agent::ActualizarReclamo(uint32_t t) {
    int z = -1, d = 0;
    if (!ElegirFranja(grid, rp, selfId, z, d)) return;
    if (z != rp.zone || d != rp.dist) {
        rp.zone    = z;
        rp.dist    = d;
        rp.tChange = t ? t : 1;
        rp.sendNow = true;
        rp.adopted = false;          // el reclamo propio cambió: hay que volver a acordar
    }
}

inline void Agent::Adoptar(uint32_t t) {
    const int antes = bel.myZone;
    assign     = AssignmentFromReparto(grid, rp, selfId);
    bel.myZone = assign.MyZone(selfId);
    rp.adopted = true;
    rp.tAdopt  = t ? t : 1;
    rp.sendNow = true;               // el reporte (ZA) sale de inmediato
    ++rp.agreements;
    Event e; e.type = EV_ZONE_AGREED; e.a = bel.myZone; e.b = (int)rp.rep; e.c = antes;
    events.Push(e);
}

inline void Agent::Repartir(uint32_t t) {
    if (!Buscando()) return;
    if (rp.tTick != 0 && (t - rp.tTick) < rp.tickMs) return;
    rp.tTick = t ? t : 1;

    const bool enGracia = (tMission != 0) && ((t - tMission) < rp.startGraceMs);
    const uint32_t m = MembersMask(t);
    if (rp.rep == 0) {
        /* La primera ronda espera a haber oído a todos (la gracia de arranque),
         * salvo que otro robot ya la haya abierto: OnZoneClaim se pasa a la suya. */
        if (enGracia) return;
        AbrirRonda(rp.seenMax + 1, m, t);
    } else if (m != rp.mask) {
        /* Cambió el conjunto de participantes (alguien dejó de oírse o apareció):
         * ronda nueva. */
        AbrirRonda((rp.seenMax > rp.rep ? rp.seenMax : rp.rep) + 1, m, t);
    } else {
        ActualizarReclamo(t);
    }

    if (!rp.adopted && (t - rp.tChange) >= rp.settleMs && RondaCompleta(rp, selfId))
        Adoptar(t);

    /* Respaldo: si pasada la gracia y fallbackMs no hubo ningún acuerdo, se toma
     * el reparto por conocimiento común (ComputeAssignment). Sin él, un enlace
     * asimétrico dejaría al robot quieto en DESPLEGAR. Se cuenta y se reporta en la
     * telemetría (zfb). */
    if (!HasZone() && rp.fallbacks == 0 && tMission != 0 &&
        (t - tMission) >= rp.startGraceMs + rp.fallbackMs) {
        const int antes = bel.myZone;
        assign     = ComputeAssignment(grid, roster);
        bel.myZone = assign.MyZone(selfId);
        ++rp.fallbacks;
        Event e; e.type = EV_ZONE_AGREED; e.a = bel.myZone; e.b = 0; e.c = antes;
        events.Push(e);
    }

    /* Difusión: el reclamo cada resendMs mientras no hay acuerdo; el reporte de
     * la franja acordada, de inmediato y después cada keepMs, para que un robot
     * que perdió el último paquete pueda cerrar su ronda. */
    const uint32_t periodo = rp.adopted ? rp.keepMs : rp.resendMs;
    if (rp.zone >= 0 && (rp.sendNow || rp.tSent == 0 || (t - rp.tSent) >= periodo)) {
        Msg msg;
        msg.kind = rp.adopted ? MSG_ZA : MSG_ZC;
        msg.a = (int32_t)rp.rep; msg.b = selfId; msg.c = rp.zone; msg.d = rp.dist;
        msg.e = rp.mask;
        out.Push(msg);
        rp.tSent   = t ? t : 1;
        rp.sendNow = false;
        ++rp.sent;
    }
}

inline void Agent::OnTargetSeen(uint32_t nowMs) {
    if (bel.myCell < 0) return;
    if (bel.targetKnown && !TargetClaimWins(selfId, bel.targetOwner)) return;
    bel.targetKnown = true;
    bel.targetOwner = selfId;
    bel.iFoundIt    = true;
    /* El objetivo está unos 100 mm delante del robot (el sensor de color lo lee
     * casi tocándolo), no en su centro, igual que en el OBJECT_FOUND del
     * laboratorio. La celda se calcula desde esa posición con la misma función
     * que usan los demás robots al recibir el TGT, para que todos ubiquen el
     * objetivo en la misma celda. */
    bel.targetX     = bel.x + 100.0f * std::cos(bel.th * 0.017453292519943295f);
    bel.targetY     = bel.y + 100.0f * std::sin(bel.th * 0.017453292519943295f);
    bel.targetCell  = MmToIndexClamped(grid, bel.targetX, bel.targetY);
    /* El temporizador de la congregación no arranca aquí sino en el primer
     * arribo. Si arrancara con el hallazgo, la congregación podría cerrarse con el
     * enjambre todavía en camino. */

    Msg m; m.kind = MSG_TGT; m.a = (int32_t)roster.epoch; m.b = selfId;
    m.f1 = bel.targetX; m.f2 = bel.targetY;
    out.Push(m);
    tTargetRepeat = nowMs;

    Event e; e.type = EV_TARGET_SEEN; e.a = bel.targetCell; events.Push(e);
}

inline void Agent::OnTargetReport(uint32_t epoch, int ownerId, float tx, float ty, uint32_t nowMs) {
    (void)epoch; (void)nowMs;
    /* Desempate de hallazgos simultáneos: gana el descubridor de menor ID, y
     * todos aplican el mismo criterio sobre el mismo dato. */
    if (!TargetClaimWins(ownerId, bel.targetOwner)) return;
    bel.targetKnown = true;
    bel.targetOwner = ownerId;
    bel.iFoundIt    = (ownerId == selfId);
    bel.targetX     = tx;
    bel.targetY     = ty;
    bel.targetCell  = MmToIndexClamped(grid, tx, ty);
    /* Objetivo nuevo: la llegada al punto anterior ya no cuenta. El MEET en
     * curso queda viejo (MeetStale) y el cuerpo lo suelta. */
    if (MeetStale()) bel.arrivedAtPlace = false;
    if (bel.phase == PH_BUSCAR || bel.phase == PH_DESPLEGAR) bel.phase = PH_CONGREGAR;

    Event e; e.type = EV_TARGET_REPORTED; e.a = bel.targetCell; e.b = ownerId; events.Push(e);
}

inline void Agent::OnTargetRequest(int fromId, uint32_t nowMs) {
    (void)fromId; (void)nowMs;
    if (!bel.targetKnown) return;
    Msg m; m.kind = MSG_TGT; m.a = (int32_t)roster.epoch; m.b = bel.targetOwner;
    m.f1 = bel.targetX; m.f2 = bel.targetY;
    out.Push(m);
}

inline void Agent::OnSensors(bool front, bool left, bool right, bool batteryLow) {
    (void)left; (void)right;
    bel.obstacleFront = front;
    if (batteryLow && !bel.batteryLow) {
        Event e; e.type = EV_BATTERY_LOW; events.Push(e);
    }
    bel.batteryLow = batteryLow;
}

inline void Agent::OnArrived() {
    bodyIdle = true;
    Event e; e.type = EV_ARRIVED; events.Push(e);
}

/* El cuerpo avisa que el MEET del laboratorio llegó a la ranura. Se acepta
 * solo si ese MEET era el del objetivo vigente. */
inline void Agent::OnSlotReached() {
    if (!bel.targetKnown || !bel.meetIssued || MeetStale()) return;
    bel.arrivedAtPlace = true;
}

// ── Consultas ─────────────────────────────────────────────────────────────

inline float Agent::NearestPeerMm(int* whoOut) const {
    float best = 1e9f;
    int   who  = -1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
        /* Visible (pose fresca) y vivo (se le oye): un robot apagado que la cámara
         * sigue viendo no es un vecino con el que haya que formar. */
        if (i == selfId || !roster.Alive(i) || !Talking(i, now)) continue;
        const float dx = bel.x - roster.p[i].x, dy = bel.y - roster.p[i].y;
        const float d  = std::sqrt(dx * dx + dy * dy);
        if (d < best) { best = d; who = i; }
    }
    if (whoOut) *whoOut = who;
    return best;
}

/* Distancia a la base del robot vivo que va más atrás. Permite acotar el
 * radio del enjambre en el retorno. Se calcula sobre la difusión de posiciones,
 * sin mensajes. */
inline float Agent::RearPeerToBaseMm() const {
    float peor = -1.0f;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
        if (i == selfId || !roster.Alive(i) || !Talking(i, now)) continue;
        const float dx = roster.p[i].x - bel.baseX, dy = roster.p[i].y - bel.baseY;
        const float d  = std::sqrt(dx * dx + dy * dy);
        if (d > peor) peor = d;
    }
    return peor;
}

inline void Agent::ZoneMaskInto(uint8_t* m) const {
    if (!HasZone()) { for (int k = 0; k < grid.Cells(); ++k) m[k] = 1; return; }
    ZoneMask(grid, assign.zones[bel.myZone], m);
}

inline uint8_t Agent::HbFlags() const {
    uint8_t f = 0;
    if (bel.targetKnown)     f |= 0x01;
    if (bel.arrivedAtPlace)  f |= 0x02;
    return f;
}

/* Quórum: todos los robots vivos reportan que conocen el objetivo y que
 * llegaron a su lugar, y el propio también llegó. Se cuenta sobre los vivos (los
 * que se oyen), no sobre los nominales, así que un robot caído no impide el
 * cierre. */
inline bool Agent::QuorumReached() const {
    if (!bel.targetKnown || !bel.arrivedAtPlace) return false;
    for (int id = 0; id < MAX_ROBOTS; ++id) {
        if (id == selfId || !Talking(id, now)) continue;
        const PeerState& p = bel.peer[id];
        if (!p.knowsTarget || !p.arrived) return false;
    }
    return true;
}

// ── Deseos ────────────────────────────────────────────────────────────────

inline bool Agent::DesireActive(Desire d) const {
    switch (d) {
    case D_SEGURO:
        return bel.batteryLow ||
               (tMission != 0 && (now - tMission) > missionTimeoutMs) ||
               bel.phase == PH_SEGURO;
    case D_VOLVER:
        return bel.congregationClosed || bel.phase == PH_VOLVER;
    case D_CONGREGAR:
        return bel.targetKnown && !bel.arrivedAtPlace && !bel.congregationClosed;
    case D_DESPLEGAR:
        /* !targetKnown: un robot que ya llegó a su ranura de congregación, casi
         * siempre fuera de su franja, volvería a tener activo D_DESPLEGAR (que le gana
         * a D_TRANQUILO) y se iría a su zona mientras espera el cierre. */
        return bel.session && !bel.zoneReached && !bel.targetKnown;
    case D_EXPLORAR:
        return bel.session && bel.zoneReached && !bel.targetKnown;
    case D_TRANQUILO:
        return true;
    default:
        return false;
    }
}

inline Desire Agent::SelectDesire() const {
    Desire best = D_TRANQUILO;
    uint8_t bp  = 0;
    for (int i = 0; i < (int)D_COUNT; ++i) {
        const Desire d = (Desire)i;
        if (!DesireActive(d)) continue;
        if (kDesirePriority[i] >= bp) { bp = kDesirePriority[i]; best = d; }
    }
    return best;
}

// ── Política de reconsideración ───────────────────────────────────────────

inline bool Agent::ShouldReconsider(const Event& ev, Desire want) const {
    if (!intention.active) return true;

    /* Un deseo de mayor prioridad interrumpe la intención en curso
     * (propiedad 4): «batería baja» no necesita un caso especial en ningún plan. */
    if (want != intention.desire) return true;

    /* Una intención sin meta concreta es una espera (el cierre de la
     * congregación, el rezagado), y lo que la desbloquea es externo al agente: se
     * reevalúa en cada ciclo. Tratarla como un compromiso normal dejaría al agente
     * esperando un evento que ya pasó. */
    if (intention.goalCell < 0) return true;

    if (ev.type == EV_BATTERY_LOW || ev.type == EV_MISSION_TIMEOUT ||
        ev.type == EV_TARGET_SEEN || ev.type == EV_TARGET_REPORTED ||
        ev.type == EV_TIMEOUT     || ev.type == EV_CONGREGATION_CLOSED) return true;

    /* Un reparto acordado que cambió la franja propia invalida la meta. */
    if (ev.type == EV_ZONE_AGREED && ev.a != ev.c) return true;

    switch (commitment) {
    case COMMIT_BLIND:
        return false;                      // solo al terminar o fracasar

    case COMMIT_SINGLE_MINDED:
        /* La creencia que motivó la intención dejó de valer: la celda que iba a
         * explorar ya la pisó otro robot, así que ir ya no aporta cobertura. */
        if (bel.phase == PH_BUSCAR && intention.goalCell >= 0 &&
            (map.Visited(intention.goalCell) || map.Blocked(intention.goalCell))) return true;
        return false;

    case COMMIT_OPEN_MINDED: {
        if (bel.phase != PH_BUSCAR || intention.goalCell < 0 || bel.myCell < 0) return false;
        /* ¿Hay una vecina inmediata cuya deseabilidad supere a la de la meta actual
         * por más de δ? Se compara sobre la feromona, que es lo que la exploración
         * optimiza. */
        const float cur = 1.0f / (1.0f + (float)map.repel[intention.goalCell] / 255.0f);
        int nb[8];
        const int m = Neighbors8(grid, bel.myCell, nb);
        for (int i = 0; i < m; ++i) {
            if (nb[i] == intention.goalCell) continue;
            const float alt = 1.0f / (1.0f + (float)map.repel[nb[i]] / 255.0f);
            if (alt > cur * (1.0f + openMindedDelta)) return true;
        }
        return false;
    }
    }
    return false;
}

// ── El ciclo ──────────────────────────────────────────────────────────────

inline Action Agent::Cycle(uint32_t nowMs) {
    Action act;
    now = nowMs;

    /*---------------------------------------------------------------------
     * 1. Revisar las creencias. Va primero y separado: el agente delibera sobre
     *    un mundo ya actualizado.
     *-------------------------------------------------------------------*/
    bel.myCell = CeldaPropia(bel.x, bel.y);

    if (tEvaporate == 0) tEvaporate = now;
    while ((now - tEvaporate) >= evaporatePeriodMs) {
        map.Evaporate();
        tEvaporate += evaporatePeriodMs;
    }
    TempBlockExpire(now);

    /* Franja vigente. El reparto lo acuerdan los agentes en Repartir(), que corre
     * dentro de Comunicar(); aquí solo se lee el último acordado (o el respaldo). */
    bel.myZone = assign.MyZone(selfId);
    if (HasZone() && bel.myCell >= 0)
        bel.zoneReached = assign.zones[bel.myZone].Has(grid, bel.myCell);

    if (bel.session && bel.phase == PH_DESPLEGAR && bel.zoneReached) bel.phase = PH_BUSCAR;
    if (bel.targetKnown && (bel.phase == PH_BUSCAR || bel.phase == PH_DESPLEGAR))
        bel.phase = PH_CONGREGAR;

    /* El temporizador de la congregación arranca con el primer arribo, propio o
     * de otro robot. */
    if (bel.arrivedAtPlace && tFirstArrival == 0) tFirstArrival = now;

    /*---------------------------------------------------------------------
     * 2. Vencimientos que son creencias temporales.
     *-------------------------------------------------------------------*/
    if (tMission != 0 && (now - tMission) > missionTimeoutMs &&
        bel.phase != PH_FIN && bel.phase != PH_SEGURO) {
        Event e; e.type = EV_MISSION_TIMEOUT; events.Push(e);
    }

    /* Cierre de la congregación: por quórum o por temporizador, lo que ocurra
     * primero. El quórum es la salida temprana y el temporizador, el techo. Como
     * el temporizador corre en paralelo, un robot atrapado no puede impedir el
     * cierre. */
    if (!bel.congregationClosed && bel.phase == PH_CONGREGAR && bel.targetKnown) {
        const bool porQuorum = QuorumReached();
        const bool porReloj  = (tFirstArrival != 0) && ((now - tFirstArrival) >= congregationMs);
        if (porQuorum || porReloj) {
            bel.congregationClosed = true;
            tClosed   = now;
            bel.phase = PH_VOLVER;
            intention.active = false;
            Event e; e.type = EV_CONGREGATION_CLOSED; e.a = porQuorum ? 1 : 0; events.Push(e);
        }
    }

    /* FIN: con la congregación cerrada, el robot que está a menos de
     * returnDoneMm de la base termina su misión, borra la memoria de la corrida y
     * queda libre. Los demás dejan de oírlo a los 3 s y la formación deja de
     * esperarlo. */
    if (bel.phase == PH_VOLVER && bel.session) {
        const float dxb = bel.x - bel.baseX, dyb = bel.y - bel.baseY;
        /* El retorno es conjunto: el robot termina cuando está en la base y el robot
         * que va más atrás también llegó (o ya no queda ninguno vivo). */
        const float rezagado = RearPeerToBaseMm();
        if (dxb * dxb + dyb * dyb <= returnDoneMm * returnDoneMm &&
            (rezagado < 0.0f || rezagado <= returnPeerMm)) {
            ResetMissionMemory();
            bel.session = false;
            bel.phase   = PH_FIN;
            Event e; e.type = EV_ARRIVED; events.Push(e);
        }
    }

    /* Éxito de la intención: una intención que alcanzó su meta se suelta y el
     * agente vuelve a deliberar en este mismo ciclo. Comprometerse es no cambiar
     * de idea sin motivo, no negarse a terminar. */
    if (intention.active && intention.goalCell >= 0 && bel.myCell == intention.goalCell)
        intention.active = false;

    /*---------------------------------------------------------------------
     * 3. Falta de progreso de la intención vigente.
     *-------------------------------------------------------------------*/
    if (intention.active && intention.goalCell >= 0 && bel.myCell >= 0) {
        const int d = CellDistance(grid, bel.myCell, intention.goalCell);
        if (intention.tProgress == 0) intention.tProgress = now;
        if (d < intention.bestDist) { intention.bestDist = d; intention.tProgress = now; }
        if ((now - intention.tProgress) > noProgressMs) {
            if (bel.phase == PH_BUSCAR) TempBlockCell(intention.goalCell, now);
            Event e; e.type = EV_TIMEOUT; e.a = intention.goalCell; events.Push(e);
            intention.tProgress = now;
        }
    }

    /*---------------------------------------------------------------------
     * 4. Percibir un evento.
     *-------------------------------------------------------------------*/
    Event ev;
    if (!events.Pop(ev)) ev = Event();
    if (ev.type != EV_NONE) { lastEvent = (uint8_t)ev.type; tLastEvent = now; }

    if (ev.type == EV_MISSION_TIMEOUT) {             // el tiempo límite también limpia la corrida
        ResetMissionMemory();
        bel.session = false;
    }
    if (ev.type == EV_MISSION_TIMEOUT || ev.type == EV_BATTERY_LOW) bel.phase = PH_SEGURO;
    if (ev.type == EV_ARRIVED) bodyIdle = true;

    /*---------------------------------------------------------------------
     * 5. Deliberar: primero el deseo, después el plan.
     *-------------------------------------------------------------------*/
    desire     = SelectDesire();
    lastDesire = kDesireName[(int)desire];

    if (ShouldReconsider(ev, desire)) {
        ++deliberations;
        const Plan* chosen = 0;
        for (int i = 0; i < libCount; ++i) {
            const Plan& p = lib[i];
            if (p.desire != D_ANY && p.desire != desire) continue;
            if (!p.trigger(ev)) continue;         // disparador y contexto son
            if (!p.context(*this)) continue;      // predicados independientes
            if (!chosen || p.priority > chosen->priority) chosen = &p;
        }
        if (chosen) {
            if (intention.planName != chosen->name) {
                /* El reloj de fracaso se ancla al deseo y no al plan: mientras el deseo
                 * no cambie, sigue corriendo aunque cambie el plan; cuando cambia, empieza de
                 * nuevo, porque la meta es otra. */
                const int      keepRetries = intention.retries;
                const uint32_t keepProg    = intention.tProgress;
                const int      keepBest    = intention.bestDist;
                const Desire   deseoPrevio = intention.desire;
                intention = Intention();
                intention.tStart = now;
                if (deseoPrevio == desire && keepProg != 0) {
                    intention.retries   = keepRetries;
                    intention.tProgress = keepProg;
                    intention.bestDist  = keepBest;
                } else {
                    intention.tProgress = now;
                }
            }
            intention.desire = desire;
            chosen->body(*this, intention, act);
            lastPlan = intention.planName;
            if (act.kind != Action::A_NONE && act.kind != Action::A_HOLD) bodyIdle = false;
        }
    } else if (bodyIdle && intention.active && intention.goalCell >= 0) {
        /* Sigue comprometido: reemite el destino si el cuerpo quedó quieto
         * (propiedad 3, la intención persiste). */
        if (intention.goalIsMm) GotoMm(act, intention.goalX, intention.goalY);
        else                    GotoCell(act, intention.goalCell);
        bodyIdle = false;
    }

    /*---------------------------------------------------------------------
     * 6. Comunicar. Está en Comunicar() para que pueda correr sin deliberar.
     *-------------------------------------------------------------------*/
    Comunicar(now);

    return act;
}

/* Paso 6 del ciclo, con reloj propio. En el robot, Cycle() solo corre
 * cuando la máquina de estados deja el cuerpo libre, así que si el latido
 * dependiera de Cycle() saldría mucho menos que a 2 Hz mientras el robot
 * navega, y el quórum, que se cuenta con latidos, se retrasaría. Por eso el
 * sketch llama también a Comunicar() desde loop(). Los relojes internos
 * (tHeartbeat, tTargetRepeat) evitan duplicados: se llame desde donde se
 * llame, sale un latido cada hbPeriodMs.
 *
 * Por radio va solo lo que no está en la difusión de posiciones: el hallazgo,
 * la fase de cada agente y el reparto de franjas. */
inline void Agent::Comunicar(uint32_t now) {
    if (!bel.session) return;

    if (tHeartbeat == 0 || (now - tHeartbeat) >= hbPeriodMs) {
        tHeartbeat = now;
        Msg m; m.kind = MSG_HB; m.a = selfId; m.b = (int32_t)bel.phase;
        m.c = (int32_t)HbFlags(); m.d = (int32_t)(++hbSeq);
        out.Push(m);
    }

    /* Reenvío epidémico: se vuelve a difundir el hallazgo mientras algún robot
     * vivo no lo conozca. UDP no garantiza la entrega y no hay acuse. */
    if (bel.targetKnown && (now - tTargetRepeat) >= tgtRepeatMs) {
        /* Sobre los que se oyen: un robot que se reinició y se reincorpora empieza a
         * latir sin conocer el objetivo, y eso dispara el reenvío. */
        bool falta = false;
        for (int id = 0; id < MAX_ROBOTS; ++id) {
            if (id == selfId || !Talking(id, now)) continue;
            if (!bel.peer[id].knowsTarget) { falta = true; break; }
        }
        if (falta) {
            tTargetRepeat = now;
            Msg m; m.kind = MSG_TGT; m.a = (int32_t)roster.epoch; m.b = bel.targetOwner;
            m.f1 = bel.targetX; m.f2 = bel.targetY;
            out.Push(m);
        }
    }

    // Reparto de franjas, con reloj propio (assignment.h).
    Repartir(now);
}

}  // namespace bdie
#endif  // BDIE_BDI_H
