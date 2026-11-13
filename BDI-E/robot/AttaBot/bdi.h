/*****************************************************************************
 * bdi.h — El ciclo de razonamiento (P1)
 *
 * Cada robot corre un agente BDI completo. No es un motor genérico tipo Jason
 * —eso sería desproporcionado sobre un ESP32— sino una versión ligera con la
 * misma estructura: creencias con revisión y envejecimiento, deseos con
 * prioridad, intenciones con compromiso y condición de fracaso, biblioteca de
 * planes como tabla estática y política de reconsideración explícita.
 *
 * LO QUE HAY QUE PODER DEFENDER ANTE EL JURADO, y por eso está separado en el
 * código y no fundido en una función: esto NO es una máquina de estados con
 * otro nombre. Cuatro propiedades lo separan de una FSM, y las cuatro son
 * verificables (prueba T07):
 *
 *   1. El mismo evento produce distinta acción según el CONTEXTO. En una FSM
 *      la transición estado×evento es fija.
 *   2. Agregar un plan cambia el comportamiento SIN TOCAR el ciclo. Por eso la
 *      biblioteca es un puntero a tabla y no un switch.
 *   3. La intención PERSISTE entre ciclos. El agente sigue comprometido con lo
 *      que decidió aunque reciba eventos, y eso es lo que evita el zigzag.
 *   4. Un deseo de mayor prioridad INTERRUMPE la intención en curso, sin casos
 *      especiales repartidos por el código.
 *
 * LA OTRA MITAD DE R5: el agente no reemplaza la máquina de estados de
 * movimiento, LA ALIMENTA. Produce Action; mission.h las traduce a
 * instructionList y a nav.Start(). El PID, los encoders, la IMU y la evasión
 * no se tocan.
 *
 * SIN DEPENDENCIAS DE ARDUINO.
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
 *  Se selecciona el de MAYOR PRIORIDAD cuya condición de activación se cumple
 *  sobre las creencias actuales. La prioridad de D_SEGURO no es decorativa: es
 *  lo que garantiza que una batería baja interrumpa cualquier intención en
 *  curso, sin un `if (batteryLow)` repartido por doce cuerpos de plan.
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
    EV_TARGET_SEEN,          // mi sensor confirmó el objetivo (voto 3-de-5)
    EV_TARGET_REPORTED,      // otro robot difundió el hallazgo
    EV_TIMEOUT,              // la intención no progresa
    EV_BATTERY_LOW,
    EV_MISSION_TIMEOUT,
    EV_CONGREGATION_CLOSED,
    EV_SESSION_START,
    EV_SESSION_ABORT,
    EV_ZONE_AGREED           // se acordó un reparto (a = mi franja, b = ronda, c = la anterior)
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
 *  Los campos se mapean a texto en mission.h. La restricción dura es que
 *  SeparateCommand() del firmware parte en MÁXIMO 5 campos (comando + 4
 *  argumentos), así que el parser existente no se toca: solo se le agregan
 *  ramas. La prueba T10 verifica que ningún mensaje se pase de cinco.
 *
 *  MSG_ZC / MSG_ZA (14-oct-2026): el reclamo de franja y el reparto acordado.
 *  a = ronda, b = id, c = franja, d = distancia, e = participantes.
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
 *  Compromiso — el eje audaz ↔ cauteloso de Kinny y Georgeff
 *
 *  Es un parámetro real del proyecto, no relleno teórico: abrir la compuerta
 *  demasiado vuelve al agente errático y no progresa; abrirla poco lo deja
 *  persiguiendo intenciones ya imposibles. Se barre en el banco de pruebas y
 *  es una figura del Capítulo 5.
 *===========================================================================*/
enum Commitment : uint8_t {
    COMMIT_BLIND = 0,        // solo al terminar o fracasar     (control degenerado)
    COMMIT_SINGLE_MINDED,    // + si la creencia que lo motivó dejó de valer  (por defecto)
    COMMIT_OPEN_MINDED       // + si otra opción supera por un margen δ
};

/*===========================================================================*
 *  Acciones — lo único que el agente le entrega al cuerpo
 *===========================================================================*/
struct Action {
    /* A_MEET: congregarse alrededor de (x, y) con el MEET DEL LABORATORIO
     * (HandleMeet, comandos.ino). El agente decide CUÁNDO y DÓNDE; el reparto
     * de ranuras, el anillo, la aproximación en dos etapas y la llegada son
     * código del laboratorio, sin tocar. Ver bdie.ino §CONGREGACIÓN. */
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

    /* Detección de falta de progreso. Sustituye a IsInLoop/IsMakingProgress,
     * que ReactiveNav tenía y perdió: hoy el único escape ante un obstáculo
     * cóncavo es el timeout de 180 s, y en una corrida de 12 minutos un robot
     * atascado tres minutos arruina la corrida.
     *
     * EL RELOJ DE FRACASO VIVE EN LA INTENCIÓN, NO EN EL PLAN, y ése fue el
     * defecto que más costó encontrar en el prototipo anterior: si se reinicia
     * al cambiar de plan, dos planes que se alternan lo dejan siempre en cero y
     * el timeout no se cumple nunca. El robot se ve «ocupado», no roto. */
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
    bool (*context)(const Agent&);            // guarda de contexto — INDEPENDIENTE del trigger
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
    Assignment   assign;        // el reparto VIGENTE (acordado, o el respaldo)
    Reparto      rp;            // la ronda en curso y lo que se oyó (assignment.h §REPARTO)

    // ── Parámetros ajustables ─────────────────────────────────────────────
    Commitment commitment        = COMMIT_SINGLE_MINDED;
    float      openMindedDelta   = 0.15f;
    uint32_t   noProgressMs      = 12000;     // ventana de falta de progreso
    uint32_t   congregationMs    = 300000;    // techo: ~5 min desde el primer arribo
    uint32_t   missionTimeoutMs  = 1200000;   // 20 min — restricción dura del cliente
    uint32_t   bdiPeriodMs       = 100;       // 10 Hz
    uint32_t   hbPeriodMs        = 500;       // 2 Hz
    /* Un par está VIVO si se oyó de él un mensaje propio (HB, ZC, ZA, TGT_REQ)
     * en este lapso: 6 latidos a 2 Hz. Es la definición de vivo desde el
     * 14-oct-2026 —antes era «lo ve la cámara», y un robot apagado que sigue en
     * la arena con su ArUco visible no se daba por caído nunca—. A 5 % de
     * pérdida, seis latidos perdidos seguidos ocurren con probabilidad 1,6e-8. */
    uint32_t   peerDeadMs        = 3000;
    /* Período de evaporación de la feromona repelente.
     *
     * 4 s, no 1 s, y el número sale de una medición, no de una intuición: con
     * `deposit = 40` un depósito desaparecía del mapa en ~34 aplicaciones, o sea
     * ~34 s a 1 Hz. Eso es menos de lo que tarda un robot en dar una vuelta por
     * su franja, así que la huella se borraba antes de que sirviera para repeler
     * a nadie. A 4 s el mismo depósito dura ~136 s (~2,3 min), que cubre buena
     * parte de la fase de búsqueda. (14-oct: con `deposit = 120`, del 12-oct,
     * llega a cero en 59 aplicaciones, ~236 s; el período no se volvió a barrer.)
     *
     * Por qué el período y no `rho`: ver la nota de cuantización en
     * pheromone_map.h. En el rango en que operamos, `rho` NO es una palanca —el
     * decaimiento entero baja de a 1 por aplicación—, así que lo único que mueve
     * de verdad el ritmo es cada cuánto se aplica. */
    /* 07-nov-2026: 4 s → 16 s (huella ~16 min en vez de ~4). El banco fijó 4 s con
     * misiones de ~2 min (t_det mediana ~70 s); en el robot la búsqueda tarda unas
     * 4× más (24-oct: 155–983 s), así que con 4 s la huella se borraba a mitad de la
     * búsqueda: en la corrida de 16 min del 24-oct (15:06) 18 de 208 metas fueron a
     * celdas ya exploradas cuya huella ya se había evaporado. 16 s escala el período
     * del banco a la velocidad del robot. En el banco no cambia nada (60 semillas,
     * 4 robots: 59/60 completas con 4, 8, 16, 30 y 60 s). El indicador de OE2 sigue
     * corriendo con 4 s, como se congeló su protocolo. NAV_CONFIG|EVAP|ms lo cambia
     * en vivo. */
    uint32_t   evaporatePeriodMs = 16000;     // ver pheromone_map.h
    bool       frontierEnabled   = true;      // plan de frontera en P_EXPLORAR_ACO (12-oct)
    /* Bloqueo temporal de la meta inalcanzable (17-oct-2026). En el laboratorio
     * el cubo objetivo ocupa una celda en la que ningún centro de robot puede
     * entrar: esa celda no queda nunca visitada, y la frontera la elegía una y
     * otra vez como «la no visitada más cercana» → los robots orbitaban el cubo.
     * Cuando la intención de BUSCAR se queda sin progreso (EV_TIMEOUT) su celda
     * meta se bloquea tempBlockMs: la ruleta y la frontera la saltean. Es local
     * a cada robot y se libera sola; las bloqueadas por la campaña con
     * obstáculos no se tocan. */
    uint32_t   tempBlockMs       = 60000;
    float      returnDoneMm      = 250.0f;   // FIN: distancia a la base (17-oct; el banco cuenta «volvió» a 350)
    float      returnPeerMm      = 500.0f;
    float      returnSlotMm      = 170.0f;   // 24-oct: radio del anillo de lugares en la base (0 = todos al mismo punto)   // FIN: el par más atrasado también llegó
    uint32_t   tgtRepeatMs       = 1000;      // reenvío epidémico del hallazgo
    uint32_t   formationWaitMs   = 8000;      // tope de espera al rezagado


    float      formationMinMm    = 350.0f;    // separación mínima entre vecinos
    float      formationMaxMm    = 900.0f;    // banda de rezago tolerada en el retorno
    /* Cómo se decide que hay que esperar en el retorno.
     *   0 · VECINO MÁS CERCANO — lo primero que se implementó, y el banco
     *       demostró que NO acota el radio del enjambre: con los robots
     *       repartidos por la arena, cada uno tiene a alguien cerca y nadie
     *       espera nunca. El radio medido fue de ~1 240 mm sobre una arena de
     *       2 500 mm, que no es «volver juntos» bajo ningún criterio.
     *   1 · COTA DE RADIO — se espera mientras haya un par vivo que esté más
     *       lejos de la base que yo por más de formationMaxMm. Acota
     *       directamente la magnitud que el Capítulo 5 reporta, se calcula
     *       sobre el mismo broadcast y no cuesta un solo mensaje. */
    int        formationMode     = 1;
    /* Gracia antes de emprender el retorno.
     *
     * El banco mostró que el radio del enjambre durante el retorno NO lo fija
     * la banda de formación: ya viene fijado por el DESFASE con que cada agente
     * cierra la congregación. El que cierra primero arranca mientras el último
     * sigue estacionado, y ~3 s de ventaja a 122 mm/s son ~370 mm que la banda
     * después no puede recuperar.
     *
     * La gracia es la corrección barata y no reintroduce la barrera de
     * sincronización que el cliente objetó: es un tiempo FIJO, no una espera a
     * que alguien avise. Nadie puede bloquear a nadie. */
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

        /* La congregación la ejecuta el MEET del laboratorio. El agente solo
         * recuerda a qué punto lo pidió: si el objetivo cambia (desempate por
         * ID), el MEET en curso quedó VIEJO y hay que pedir otro. */
        bool  meetIssued         = false;
        float meetX = 0.0f, meetY = 0.0f;
        /* Lo escribe OnSlotReached(), es decir el cuerpo: el laboratorio
         * declara la llegada cuando ReactiveNavStep alcanza la ranura. */
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
    /* Instrumentación (V2, telemetry.h): el último evento que el ciclo
     * percibió. Es solo un registro para OBSERVAR por qué el agente
     * reconsideró; ninguna decisión lo lee. */
    uint8_t     lastEvent  = 0;          // EventType
    uint32_t    tLastEvent = 0;
    const char* lastDesire = "TRANQUILO";
    uint32_t    deliberations = 0;

    const Plan* lib      = 0;                 // biblioteca; inyectable (propiedad 2)
    int         libCount = 0;

    uint32_t rng = 1;

    // ── Relojes ───────────────────────────────────────────────────────────
    uint32_t tMission       = 0;
    uint32_t tEvaporate     = 0;
    struct TempBlock { int cell; uint32_t until; };
    TempBlock tempBlock[8];
    void TempBlockClear() { for (int i = 0; i < 8; ++i) tempBlock[i].cell = -1; }
    void TempBlockCell(int k, uint32_t now) {
        if (k < 0 || map.Blocked(k)) return;          // ya bloqueada (o de la campaña)
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
    /* La celda del propio robot (24-oct-2026). Con MmToIndex, un robot apenas
     * FUERA de la cuadrícula —el Atta_1 en (−12, 90), a 22 mm del borde
     * izquierdo— quedaba en la celda −1: P_EXPLORAR_ACO no tiene vecinos de −1,
     * no elige meta y el robot se quedó 3 min quieto deliberando en vacío
     * (corrida de las 15:06). Hasta 300 mm fuera se toma la celda del borde más
     * cercana, así la exploración lo trae de vuelta adentro. */
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
    /* El paso 6 del ciclo (latido y reenvío epidémico), por reloj propio.
     * Cycle() lo llama; el `.ino` lo llama TAMBIÉN desde loop(), para que
     * el latido no dependa de que el cuerpo esté libre. */
    void Comunicar(uint32_t nowMs);
    void OnTargetSeen(uint32_t nowMs);
    void OnTargetReport(uint32_t epoch, int ownerId, float tx, float ty, uint32_t nowMs);
    void OnTargetRequest(int fromId, uint32_t nowMs);
    void OnSensors(bool front, bool left, bool right, bool batteryLow);
    void OnArrived();
    /* El MEET del laboratorio llegó a la ranura (enganche en ReactiveNavStep). */
    void OnSlotReached();

    // ── El ciclo ──────────────────────────────────────────────────────────
    Action Cycle(uint32_t nowMs);

    // ── Consultas que usan los planes y las pruebas ───────────────────────
    bool  HasZone()  const { return bel.myZone >= 0; }
    /* ¿El MEET en curso apunta a un objetivo que ya no es el vigente? */
    bool  MeetStale() const {
        return bel.meetIssued && (bel.meetX != bel.targetX || bel.meetY != bel.targetY);
    }
    /* VIVO = LO OIGO. El propio robot siempre. */
    bool  Talking(int id, uint32_t t) const;
    uint32_t TalkingMask(uint32_t t) const;
    /* Participantes del reparto: los que se oyen y están buscando (yo incluido
     * si estoy buscando). Un par del que todavía no llegó ningún latido tiene
     * fase IDLE = desconocida, y cuenta. */
    uint32_t MembersMask(uint32_t t) const;
    bool  Buscando() const {
        return bel.session && !bel.targetKnown &&
               (bel.phase == PH_DESPLEGAR || bel.phase == PH_BUSCAR);
    }
    float NearestPeerMm(int* whoOut = 0) const;
    float RearPeerToBaseMm() const;      // el par vivo MÁS lejano de la base
    void  ZoneMaskInto(uint8_t* m) const;
    uint8_t HbFlags() const;
    bool  QuorumReached() const;

    int   AliveCount() const { return roster.AliveCount(); }

    bool DesireActive(Desire d) const;
    Desire SelectDesire() const;
    bool ShouldReconsider(const Event& ev, Desire want) const;

    // ── El reparto por mensajes (assignment.h §REPARTO) ───────────────────
    /* Corre dentro de Comunicar(), es decir en cada loop() del robot y no solo
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
 *  Son predicados INDEPENDIENTES a propósito. En una FSM la transición es una
 *  sola función de (estado, evento); acá el disparador mira el evento y el
 *  contexto mira las creencias, y la combinación es lo que hace que el mismo
 *  evento produzca acciones distintas. Es la propiedad 1 de la prueba T07.
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
 * Es también el plan de espera de la congregación: el robot que ya llegó a su
 * ranura deja de tener D_CONGREGAR activo y cae a D_TRANQUILO. No hace falta
 * un plan «esperar_congregacion» aparte, y no tenerlo es una pieza menos. */
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
 * Ruleta sobre las ocho vecinas, restringida a mi zona. Si la zona se agotó
 * —todas las vecinas elegibles están saturadas o fuera— se amplía a la arena
 * completa antes de rendirse: quedarse quieto en una zona barrida es peor que
 * ayudar en la del vecino. */
inline void bdExplorarAco(Agent& a, Intention& I, Action& act) {
    I.planName = "P_EXPLORAR_ACO";
    I.active   = true;
    uint8_t mask[MAX_CELLS];
    a.ZoneMaskInto(mask);
    const uint8_t* zm = a.HasZone() ? mask : 0;

    /* FRONTERA (12-oct-2026). Si ninguna vecina elegible está sin visitar, la
     * ruleta solo puede elegir entre celdas ya miradas y el robot da vueltas
     * sobre lo barrido (7 de 46 decisiones en la corrida de las 16:01). En ese
     * caso se va directo a la no visitada más cercana: primero de su zona; si
     * la zona está completa, de toda la arena. La intención queda comprometida
     * a esa celda hasta llegar o hasta que otro la pise (ShouldReconsider,
     * SINGLE_MINDED), así que no se re-sortea a cada ciclo. */
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
 * La congregación NO la resuelve el agente: la pide al MEET del laboratorio
 * (comandos.ino HandleMeet → navegacion.ino MeetSlotIndex / SafeRingSlotAngle /
 * UpdateCongregationGoal). Cada robot saca su ranura del anillo de la lista de
 * vecinos NEIGHBOR_POSITIONS, sin líder y sin que la base decida nada, y el
 * laboratorio declara la llegada. Lo único que aporta el agente es CUÁNDO
 * (al conocer el objetivo) y DÓNDE (el punto del hallazgo).
 *
 * Todos los robots corren el mismo plan, incluido el descubridor: en el MEET
 * del laboratorio el punto es fijo y nadie hace de líder.
 *
 * Sin meta de celda (goalCell = -1): mientras el laboratorio navega, el cuerpo
 * está ocupado y el agente no delibera. Si el cuerpo queda libre SIN haber
 * llegado —el laboratorio soltó la navegación por timeout, o el MEET quedó
 * viejo porque cambió el objetivo—, este mismo plan vuelve a pedirlo. */
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
 * Formación LAXA, como la pidió el cliente: «que no anden chocando, ni que
 * anden tan lejos». Sin líder y sin formación rígida: cada robot navega a la
 * base corrigiendo su meta por una banda de distancia al vecino más cercano,
 * calculada sobre las poses del broadcast.
 *
 * La espera al rezagado está ACOTADA a propósito. Esperar sin tope reintroduce
 * por la puerta de atrás la barrera de sincronización que el cliente objetó en
 * el quórum: un robot atascado congelaría a los otros cuatro. */
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

    /* ¿Hay que esperar? Ver Agent::formationMode: el modo 0 mira al vecino más
     * cercano y NO acota el radio del enjambre; el modo 1 mira al par que va
     * más atrás, que es exactamente la magnitud que el informe reporta. */
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
        /* La espera está ACOTADA a propósito: sin tope reintroduciría por la
         * puerta de atrás la barrera de sincronización que el cliente objetó en
         * el quórum — un robot atascado congelaría a los otros cuatro. */
        if ((a.now - a.tFormationWait) < a.formationWaitMs) {
            I.goalCell = -1;
            act.kind = Action::A_HOLD;
            return;
        }
    } else {
        a.tFormationWait = 0;
    }

    float gx = a.bel.baseX, gy = a.bel.baseY;
    /* LUGAR PROPIO EN LA BASE (24-oct-2026). Antes todos apuntaban al mismo
     * punto: en la corrida del 17-oct a las 15:02 el Atta_3 se estacionó en él y
     * el Atta_2 pasó 30 s esquivándolo a 120 mm sin poder llegar. Cada robot va a
     * su lugar en un anillo de returnSlotMm alrededor de la base (ángulo por ID,
     * 72° entre lugares: hasta 5 robots sin repetir), dentro de la cuadrícula.
     * returnSlotMm < returnDoneMm − llegada, así el lugar cuenta para FIN. */
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
    /* Cerca de la base manda el anillo: la repulsión de 350 mm empujaría a los
     * vecinos fuera de sus lugares (están a ~210 mm entre sí). */
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
 * intentos deja de restringirse a la zona: si el robot no puede progresar
 * dentro de su franja, insistir es la definición de atascarse. */
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
 *  La biblioteca de planes
 *
 *  Es una TABLA, no un switch, y ésa es la propiedad 2 de la prueba anti-FSM:
 *  agregar comportamiento es agregar una fila. El ciclo no se entera.
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
    /* Semilla derivada del ID: dos robots no sortean igual —si lo hicieran, la
     * ruleta dejaría de dispersar— y una corrida se repite exacta. */
    TempBlockClear();
    rng = 0x9E3779B9u ^ (uint32_t)(id * 2654435761u);
    if (rng == 0) rng = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) lastCellOf[i] = -1;
}

/* LA MEMORIA DE UNA CORRIDA SE BORRA AL TERMINARLA (17-oct-2026): al iniciar
 * una sesión, al abortarla y al terminarla con éxito (FIN). Se conservan la
 * pose, la batería, quién está vivo y las celdas bloqueadas por la campaña con
 * obstáculos. */
inline void Agent::ResetMissionMemory() {
    /* CADA SESIÓN EMPIEZA DE CERO (17-oct-2026). Antes StartMission no limpiaba
     * nada de la corrida anterior: la feromona y la cobertura seguían, y —lo
     * grave— también targetKnown, arrivedAtPlace, congregationClosed y los
     * «llegó / conoce el objetivo» de los pares. Una segunda misión sin
     * reiniciar la GUI arrancaba creyendo conocer el objetivo viejo, con el
     * quórum ya cumplido. Solo el CONFIG (reinicio de la GUI) lo limpiaba, vía
     * Init(). Se conservan: la pose, la batería, quién está vivo (roster y
     * latidos) y las celdas bloqueadas por la campaña con obstáculos. */
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
    /* El START se manda 3 veces seguidas (UDP) y BASE.SESSION|JOIN lo reenvía a
     * un robot que se reinició: si es la MISMA sesión que ya está en curso, no
     * se toca nada (antes, una repetición devolvía la fase a DESPLEGAR). */
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
    /* El reparto empieza de cero en cada sesión: sin franja hasta que la
     * primera ronda se acuerde (P_IR_A_ZONA espera quieto mientras tanto). */
    rp.ResetState();
    assign = Assignment();
    bel.myZone      = -1;
    bel.zoneReached = false;
    Event e; e.type = EV_SESSION_START; events.Push(e);
}

inline void Agent::AbortMission(uint32_t nowMs) {
    (void)nowMs;
    ResetMissionMemory();                  // 17-oct: abortar también limpia la corrida
    Event e; e.type = EV_SESSION_ABORT; events.Push(e);
    bel.session = false;
    bel.phase   = PH_SEGURO;
}

/* Depósito de feromona: se hace por TODOS los robots vistos en el broadcast,
 * no solo por el propio. Ahí está la estigmergia, y ésa es la razón por la que
 * las réplicas convergen sin hablarse.
 *
 * Se deposita al CAMBIAR de celda, no en cada paquete. A 10 Hz un depósito por
 * paquete satura la celda en 0.64 s y el mapa deja de discriminar; un depósito
 * por entrada mide lo que la métrica quiere medir, que es dónde estuvo el
 * enjambre. */
inline void Agent::OnPose(int id, float x, float y, float th, uint32_t epoch) {
    roster.OnPose(id, x, y, th, epoch);
    const int k = MmToIndex(grid, x, y);
    /* 24-oct: la celda propia se actualiza con cada pose, no solo al deliberar.
     * Mientras navegaba, el THINK mostraba la celda donde había deliberado por
     * última vez (corrida 17-oct 15:02: «celda=23» fija con el robot cruzando la
     * arena). */
    if (id == selfId) { bel.x = x; bel.y = y; bel.th = th; bel.myCell = CeldaPropia(x, y); }
    if (k >= 0 && k != lastCellOf[id]) {
        /* 24-oct: solo se deposita con sesión abierta. Tras el ABORT del 17-oct
         * la cobertura volvió a subir a 7 celdas en SEGURO con la memoria recién
         * borrada; la corrida siguiente arrancaba con rastro de la anterior. */
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

/* La resta va con signo: un mensaje que llegó DESPUÉS del `now` del último
 * ciclo tiene tLast > t, y en aritmética sin signo eso daría ~49 días de
 * silencio — el par se declararía muerto justo cuando acaba de hablar. */
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

/* ── El reparto por mensajes ─────────────────────────────────────────────── */

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
    if (rep > rp.rep) {               // ronda más nueva: me paso a ella
        AbrirRonda(rep, MembersMask(t), t);
        return;
    }
    if (rep < rp.rep) {               // va atrasado: que oiga mi ronda pronto
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
    rp.anchor  = MmToIndexClamped(grid, bel.x, bel.y);    // ancla CONGELADA
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
        rp.adopted = false;          // mi reclamo cambió: hay que volver a acordar
    }
}

inline void Agent::Adoptar(uint32_t t) {
    const int antes = bel.myZone;
    assign     = AssignmentFromReparto(grid, rp, selfId);
    bel.myZone = assign.MyZone(selfId);
    rp.adopted = true;
    rp.tAdopt  = t ? t : 1;
    rp.sendNow = true;               // el reporte (ZA) sale ya
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
        /* La primera ronda espera a haber oído a todos (la gracia), salvo que
         * un par ya la haya abierto: OnZoneClaim se pasa a la suya. */
        if (enGracia) return;
        AbrirRonda(rp.seenMax + 1, m, t);
    } else if (m != rp.mask) {
        /* Cambió el conjunto —alguien dejó de oírse o apareció—: ronda nueva. */
        AbrirRonda((rp.seenMax > rp.rep ? rp.seenMax : rp.rep) + 1, m, t);
    } else {
        ActualizarReclamo(t);
    }

    if (!rp.adopted && (t - rp.tChange) >= rp.settleMs && RondaCompleta(rp, selfId))
        Adoptar(t);

    /* RESPALDO. Si a los grace + fallback no hubo NINGÚN acuerdo, se toma el
     * reparto por conocimiento común de la versión anterior. Es la red de
     * seguridad: sin ella un enlace asimétrico dejaría al robot quieto en
     * DESPLEGAR. Se cuenta, y se ve en la telemetría (zfb). */
    if (!HasZone() && rp.fallbacks == 0 && tMission != 0 &&
        (t - tMission) >= rp.startGraceMs + rp.fallbackMs) {
        const int antes = bel.myZone;
        assign     = ComputeAssignment(grid, roster);
        bel.myZone = assign.MyZone(selfId);
        ++rp.fallbacks;
        Event e; e.type = EV_ZONE_AGREED; e.a = bel.myZone; e.b = 0; e.c = antes;
        events.Push(e);
    }

    /* Difusión: el reclamo cada resendMs mientras no hay acuerdo; el reporte
     * acordado, de inmediato y luego cada keepMs, para que un par que perdió el
     * último paquete pueda cerrar su ronda. */
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
    /* 17-oct-2026: el objetivo está ~100 mm DELANTE del robot (el sensor de
     * color lo lee tocándolo), no en su centro: es lo mismo que calcula el
     * OBJECT_FOUND del laboratorio. Y la celda se saca de esa posición con la
     * MISMA función que usan los pares al recibir el TGT; antes el descubridor
     * usaba su propia celda y los demás otra (alerta V6: R1→113, R2/R3→91). */
    bel.targetX     = bel.x + 100.0f * std::cos(bel.th * 0.017453292519943295f);
    bel.targetY     = bel.y + 100.0f * std::sin(bel.th * 0.017453292519943295f);
    bel.targetCell  = MmToIndexClamped(grid, bel.targetX, bel.targetY);
    /* OJO: el temporizador NO arranca acá. Arranca en el primer ARRIBO, que es
     * lo que dice el documento de arquitectura. Anclarlo al hallazgo —que fue
     * el primer intento— hace que la congregación pueda cerrarse con el
     * enjambre todavía en camino: el banco lo encontró viendo corridas que
     * cerraban exactamente en t_detect + 300 s con robots a 900 mm del
     * objetivo. */

    Msg m; m.kind = MSG_TGT; m.a = (int32_t)roster.epoch; m.b = selfId;
    m.f1 = bel.targetX; m.f2 = bel.targetY;
    out.Push(m);
    tTargetRepeat = nowMs;

    Event e; e.type = EV_TARGET_SEEN; e.a = bel.targetCell; events.Push(e);
}

inline void Agent::OnTargetReport(uint32_t epoch, int ownerId, float tx, float ty, uint32_t nowMs) {
    (void)epoch; (void)nowMs;
    /* Desempate de hallazgos simultáneos: gana el descubridor de MENOR ID, y
     * todos aplican el mismo criterio sobre el mismo dato. Sin esto el enjambre
     * se parte en dos grupos congregándose en puntos distintos. */
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

/* El cuerpo avisa que el MEET del laboratorio llegó a la ranura. Se acepta solo
 * si ese MEET era el del objetivo VIGENTE: una llegada al punto viejo no es
 * estar congregado. */
inline void Agent::OnSlotReached() {
    if (!bel.targetKnown || !bel.meetIssued || MeetStale()) return;
    bel.arrivedAtPlace = true;
}

// ── Consultas ─────────────────────────────────────────────────────────────

inline float Agent::NearestPeerMm(int* whoOut) const {
    float best = 1e9f;
    int   who  = -1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
        /* Visible (pose fresca) Y vivo (se oye): un robot apagado que la cámara
         * sigue viendo no es un vecino con el que haya que formar. */
        if (i == selfId || !roster.Alive(i) || !Talking(i, now)) continue;
        const float dx = bel.x - roster.p[i].x, dy = bel.y - roster.p[i].y;
        const float d  = std::sqrt(dx * dx + dy * dy);
        if (d < best) { best = d; who = i; }
    }
    if (whoOut) *whoOut = who;
    return best;
}

/* Distancia a la base del par vivo que va más atrás. Es lo que permite acotar
 * el RADIO DEL ENJAMBRE —la distancia máxima entre dos robots— que es el
 * criterio objetivo de «volver juntos» declarado en el documento de
 * arquitectura. Se calcula sobre las poses del broadcast: cero mensajes. */
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

/* Quórum: TODOS los pares VIVOS reportan que conocen el objetivo y que ya
 * llegaron a su lugar, y yo también llegué.
 *
 * Se cuenta sobre los pares VIVOS, no sobre los cinco nominales, y ésa es la
 * respuesta exacta a la objeción del cliente: un robot caído no lo impide.
 *
 * VIVO = LO OIGO (14-oct-2026). Antes se contaba sobre los que VE la cámara, y
 * un robot apagado que sigue en la arena se esperaba hasta el temporizador. */
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
        /* !targetKnown (17-oct-2026): sin esto, un robot que ya llegó a su ranura
         * de congregación —casi siempre FUERA de su franja— volvía a tener activo
         * «desplegar» (zoneReached = false), que le gana a TRANQUILO, y se iba
         * caminando a su zona mientras esperaba el cierre. Visto en el
         * laboratorio: R2 y R3 dejaron la ranura a los 2 s y deambularon 5 min. */
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

    /* Un deseo de mayor prioridad interrumpe la intención en curso. Es la
     * propiedad 4 de la prueba anti-FSM, y es lo que hace que «batería baja»
     * no necesite un caso especial en ningún cuerpo de plan. */
    if (want != intention.desire) return true;

    /* Una intención SIN meta concreta es una ESPERA —esperar el cierre de la
     * congregación, esperar al rezagado— y lo que la desbloquea es externo al
     * agente. Hay que reevaluarla en cada ciclo: tratarla como un compromiso
     * normal deja al agente esperando para siempre un evento que ya pasó. Es el
     * modo de fallo más silencioso de toda la arquitectura, porque el robot se
     * ve «tranquilo», no roto. */
    if (intention.goalCell < 0) return true;

    if (ev.type == EV_BATTERY_LOW || ev.type == EV_MISSION_TIMEOUT ||
        ev.type == EV_TARGET_SEEN || ev.type == EV_TARGET_REPORTED ||
        ev.type == EV_TIMEOUT     || ev.type == EV_CONGREGATION_CLOSED) return true;

    /* Un reparto acordado que me cambió de franja invalida la meta: iba a
     * desplegarme o a explorar en una franja que ya no es la mía. */
    if (ev.type == EV_ZONE_AGREED && ev.a != ev.c) return true;

    switch (commitment) {
    case COMMIT_BLIND:
        return false;                      // solo al terminar o fracasar

    case COMMIT_SINGLE_MINDED:
        /* La creencia que motivó la intención dejó de valer: la celda que iba a
         * explorar ya la pisó alguien —propio o ajeno, da igual: la feromona no
         * distingue— así que ir ya no aporta cobertura. */
        if (bel.phase == PH_BUSCAR && intention.goalCell >= 0 &&
            (map.Visited(intention.goalCell) || map.Blocked(intention.goalCell))) return true;
        return false;

    case COMMIT_OPEN_MINDED: {
        if (bel.phase != PH_BUSCAR || intention.goalCell < 0 || bel.myCell < 0) return false;
        /* ¿Hay una vecina inmediata cuya deseabilidad supere a la meta actual
         * por más de δ? Se compara sobre la feromona, que es lo que la
         * exploración optimiza. */
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
     * 1. ENVEJECER CREENCIAS.
     *    La revisión de creencias va primero y separada del resto: lo que el
     *    agente delibera tiene que ser sobre un mundo ya actualizado.
     *-------------------------------------------------------------------*/
    bel.myCell = CeldaPropia(bel.x, bel.y);

    if (tEvaporate == 0) tEvaporate = now;
    while ((now - tEvaporate) >= evaporatePeriodMs) {
        map.Evaporate();
        tEvaporate += evaporatePeriodMs;
    }
    TempBlockExpire(now);

    /* La franja vigente. El reparto ya no se calcula acá: lo ACUERDAN los
     * agentes por mensajes en Repartir(), que corre dentro de Comunicar() en
     * cada loop() —una ronda no puede esperar a que el cuerpo quede libre—.
     * Acá solo se lee el último acordado (o el respaldo). */
    bel.myZone = assign.MyZone(selfId);
    if (HasZone() && bel.myCell >= 0)
        bel.zoneReached = assign.zones[bel.myZone].Has(grid, bel.myCell);

    if (bel.session && bel.phase == PH_DESPLEGAR && bel.zoneReached) bel.phase = PH_BUSCAR;
    if (bel.targetKnown && (bel.phase == PH_BUSCAR || bel.phase == PH_DESPLEGAR))
        bel.phase = PH_CONGREGAR;

    /* El temporizador de la congregación arranca en el PRIMER ARRIBO —propio o
     * de un par, lo que ocurra antes— y no en el hallazgo. */
    if (bel.arrivedAtPlace && tFirstArrival == 0) tFirstArrival = now;

    /*---------------------------------------------------------------------
     * 2. VENCIMIENTOS que son creencias temporales.
     *-------------------------------------------------------------------*/
    if (tMission != 0 && (now - tMission) > missionTimeoutMs &&
        bel.phase != PH_FIN && bel.phase != PH_SEGURO) {
        Event e; e.type = EV_MISSION_TIMEOUT; events.Push(e);
    }

    /* CIERRE DE LA CONGREGACIÓN: quórum O temporizador, el primero que ocurra.
     *
     * El quórum es la salida temprana; el temporizador es el techo. La objeción
     * del cliente al quórum —que contar robots crea una barrera de
     * sincronización y un robot atrapado congela al enjambre— queda cubierta
     * porque el temporizador corre EN PARALELO y nadie puede bloquear el
     * cierre. Los dos criterios juntos son estrictamente mejores que cualquiera
     * por separado, y ésta es la única pieza donde BDI-E amplía el acuerdo de
     * la S4 en vez de sustituirlo. */
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

    /* ÉXITO DE LA INTENCIÓN. Una intención que alcanzó su meta está cumplida
     * y se SUELTA: el agente vuelve a deliberar en este mismo ciclo. Sin esto
     * el compromiso se vuelve terquedad —el agente sigue «yendo» a la celda en
     * la que ya está— y el agente deja de ser distinguible de una FSM atascada.
     * Es la otra mitad del compromiso: comprometerse es no cambiar de idea sin
     * motivo, no negarse a terminar. */
    /* FIN (17-oct-2026). Antes ningún robot llegaba nunca a PH_FIN: al volver a
     * la base seguía en VOLVER hasta el timeout de 20 min (→ SEGURO). Ahora, ya
     * cerrada la congregación, el robot que está a ≤ returnDoneMm de la base
     * termina su misión, borra la memoria de la corrida y queda libre. Los
     * demás dejan de oírlo a los 3 s y la formación deja de esperarlo. */
    if (bel.phase == PH_VOLVER && bel.session) {
        const float dxb = bel.x - bel.baseX, dyb = bel.y - bel.baseY;
        /* El retorno es CONJUNTO: se termina cuando yo estoy en la base Y el par
         * que va más atrás también llegó (o no queda ninguno vivo). Terminar solo
         * dejaba al primero parado en la base, y la separación mínima de la
         * formación les impedía llegar a los demás (el banco lo mostró: 3 de 80
         * misiones sin completar y colas de 700+ s). */
        const float rezagado = RearPeerToBaseMm();
        if (dxb * dxb + dyb * dyb <= returnDoneMm * returnDoneMm &&
            (rezagado < 0.0f || rezagado <= returnPeerMm)) {
            ResetMissionMemory();
            bel.session = false;
            bel.phase   = PH_FIN;
            Event e; e.type = EV_ARRIVED; events.Push(e);
        }
    }

    if (intention.active && intention.goalCell >= 0 && bel.myCell == intention.goalCell)
        intention.active = false;

    /*---------------------------------------------------------------------
     * 3. FALTA DE PROGRESO de la intención vigente.
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
     * 4. PERCIBIR un evento.
     *-------------------------------------------------------------------*/
    Event ev;
    if (!events.Pop(ev)) ev = Event();
    if (ev.type != EV_NONE) { lastEvent = (uint8_t)ev.type; tLastEvent = now; }

    if (ev.type == EV_MISSION_TIMEOUT) {             // 17-oct: el timeout también limpia la corrida
        ResetMissionMemory();
        bel.session = false;
    }
    if (ev.type == EV_MISSION_TIMEOUT || ev.type == EV_BATTERY_LOW) bel.phase = PH_SEGURO;
    if (ev.type == EV_ARRIVED) bodyIdle = true;

    /*---------------------------------------------------------------------
     * 5. DELIBERAR: primero el deseo, después el plan.
     *-------------------------------------------------------------------*/
    desire     = SelectDesire();
    lastDesire = kDesireName[(int)desire];

    if (ShouldReconsider(ev, desire)) {
        ++deliberations;
        const Plan* chosen = 0;
        for (int i = 0; i < libCount; ++i) {
            const Plan& p = lib[i];
            if (p.desire != D_ANY && p.desire != desire) continue;
            if (!p.trigger(ev)) continue;         // trigger y context son
            if (!p.context(*this)) continue;      // predicados INDEPENDIENTES
            if (!chosen || p.priority > chosen->priority) chosen = &p;
        }
        if (chosen) {
            if (intention.planName != chosen->name) {
                /* EL RELOJ DE FRACASO SE ANCLA AL DESEO, NO AL PLAN.
                 *
                 * Si se reiniciara en cada cambio de plan, dos planes que se
                 * alternan al servicio del MISMO deseo lo dejarían siempre en
                 * cero y el timeout de falta de progreso no se cumpliría nunca:
                 * el robot se ve «ocupado», no roto. Mientras el deseo no
                 * cambie, el reloj sigue corriendo; cuando cambia, empieza de
                 * nuevo, que es lo correcto porque la meta es otra. */
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
        /* Sigue comprometido: reemite el destino si el cuerpo quedó quieto.
         * Esto es la propiedad 3 —la intención persiste— hecha visible. */
        if (intention.goalIsMm) GotoMm(act, intention.goalX, intention.goalY);
        else                    GotoCell(act, intention.goalCell);
        bodyIdle = false;
    }

    /*---------------------------------------------------------------------
     * 6. COMUNICAR. Extraído a Comunicar() para que pueda correr SIN deliberar.
     *-------------------------------------------------------------------*/
    Comunicar(now);

    return act;
}

/* Comunicar — el paso 6 del ciclo, por reloj propio.
 *
 * Estaba dentro de Cycle(), y eso ataba el ritmo del latido al ritmo de
 * deliberación. En el robot, Cycle() solo corre cuando la máquina de estados
 * deja el cuerpo libre: mientras navega, no delibera, y por lo tanto NO EMITÍA.
 * Medido el 28-sep-2026 sobre hardware: 0,37 Hz contra los 2 Hz del diseño, el
 * 18 %. El quórum de la congregación se cierra contando latidos, así que a ese
 * ritmo el temporizador de respaldo dejaba de ser respaldo.
 *
 * Sacarlo acá NO cambia nada para quien solo llame a Cycle() —el banco, por
 * ejemplo—, porque Cycle() lo sigue invocando en el mismo punto. Lo que habilita
 * es que el `.ino` lo llame también desde loop(), fuera del switch de estados.
 * Los dos relojes internos (tHeartbeat, tTargetRepeat) hacen que no se duplique
 * nada: corra desde donde corra, sale uno cada hbPeriodMs.
 *
 * Lo que necesita radio es lo que NO se ve en el broadcast de poses: el
 * hallazgo, la fase de cada agente y, desde el 14-oct-2026, el reparto de
 * franjas (Repartir, al final). */
inline void Agent::Comunicar(uint32_t now) {
    if (!bel.session) return;

    if (tHeartbeat == 0 || (now - tHeartbeat) >= hbPeriodMs) {
        tHeartbeat = now;
        Msg m; m.kind = MSG_HB; m.a = selfId; m.b = (int32_t)bel.phase;
        m.c = (int32_t)HbFlags(); m.d = (int32_t)(++hbSeq);
        out.Push(m);
    }

    /* Reenvío epidémico: se re-difunde el hallazgo MIENTRAS algún par vivo no
     * lo conozca. UDP no garantiza entrega y no hay acuse; la misión no puede
     * depender de que un mensaje llegue a la primera. */
    if (bel.targetKnown && (now - tTargetRepeat) >= tgtRepeatMs) {
        /* Sobre los que SE OYEN: el robot que se reinició y se reincorpora
         * empieza a latir sin conocer el objetivo, y eso dispara el reenvío. */
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

    // El reparto de franjas, por reloj propio (assignment.h §REPARTO).
    Repartir(now);
}

}  // namespace bdie
#endif  // BDIE_BDI_H
