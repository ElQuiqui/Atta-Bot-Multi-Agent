/*****************************************************************************
 * bdie_bridge.cpp — Puente C para el banco de pruebas
 *
 * 🔴 CRITERIO NO NEGOCIABLE DEL PROYECTO, y este archivo es lo que lo
 * materializa: el simulador NO reimplementa la lógica de decisión. Carga esta
 * biblioteca y ejecuta el MISMO C++ que se compila para el ESP32.
 *
 * Es la respuesta al defecto conocido del repositorio —que sim/attabot_sim.py
 * sea una reimplementación del firmware, con lo que las dos versiones divergen
 * y cada defecto hay que corregirlo dos veces—. Y es la razón por la que
 * grid.h, pheromone_map.h, assignment.h y bdi.h se escribieron sin
 * dependencias de Arduino.
 *
 * El banco es MEDIO DE INGENIERÍA, NUNCA ENTREGABLE (R1 del cliente: «ya lo
 * tuvimos simulado; si no, sería una repetición de lo anterior»). No aparece en
 * ningún objetivo. Sirve para barrer los parámetros de la feromona sin ocupar
 * el laboratorio, que con menos de 4 h por semana es lo que lo justifica.
 *
 * La CONGREGACIÓN no está acá: es el MEET del laboratorio, y el banco lo
 * compila aparte, verbatim, en lab_cong.cpp (ver extraer_lab.py). Esta
 * biblioteca expone solo el lado del agente: pedir el MEET (acción 5),
 * enterarse de la llegada (bdie_slot_reached) y saber si quedó viejo.
 *
 * Compilar:  make sitl
 *****************************************************************************/
#include "../firmware/mission.h"
#include "../firmware/telemetry.h"
#include <cstring>

using namespace bdie;

extern "C" {

struct COut {
    int   kind;        // 0 none, 1 goto_cell, 2 goto_mm, 3 hold, 4 abort, 5 meet
    int   cell;
    float x, y;
    int   phase;
    int   desire;
    int   zone;
    int   myCell;
    int   coverage;    // celdas acumuladas — la MÉTRICA, no la feromona
    int   targetOwner;
    int   congregationClosed;
    int   arrivedAtPlace;
    int   iFoundIt;
};

/* El nodo es el agente MÁS el modelo de su sensor.
 *
 * V2 (28-sep-2026): target_sensor.h ya no existe. En el robot, la detección la
 * hace el firmware del LABORATORIO —SEARCH_APPROACH + MatchColor() con balance
 * de blancos por robot— y entrega UN hallazgo por acercamiento, sin voto K-de-M.
 * El modelo de acá reproduce ese contrato: el banco dice si el objeto está al
 * alcance del sensor, y el flanco de subida es un hallazgo. Lo que el banco NO
 * modela es el tiempo del acercamiento lento (SEARCH_CREEP_PWM, ≤ 6 s): queda
 * como diferencia declarada entre banco y robot. Las firmas de las funciones
 * no cambian, así que bdie_sim.py y el protocolo del indicador tampoco. */
struct LabColorSearchModel {
    bool active = true, prev = false;
    bool Edge(bool raw) { const bool e = active && raw && !prev; prev = raw; return e; }
};
struct Node {
    Agent               agent;
    LabColorSearchModel sensor;
};
static inline Agent* A(void* h) { return &((Node*)h)->agent; }

struct CMsg { int kind, a, b, c, d; float f1, f2; };

void* bdie_new(int id, float ox, float oy, float cellMm, int rows, int cols) {
    Node* n = new Node();
    GridConfig g;
    g.originXmm = ox; g.originYmm = oy; g.cellMm = cellMm; g.rows = rows; g.cols = cols;
    n->agent.Init(id, g);
    return (void*)n;
}
void bdie_free(void* h) { delete (Node*)h; }

void bdie_seed(void* h, unsigned s) { A(h)->rng = s ? s : 1u; }

/* ¿El objeto está al alcance del sensor? El flanco de subida con la búsqueda
 * armada es un hallazgo, como MatchColor() positivo en SEARCH_APPROACH. */
void bdie_sensor_raw(void* h, int raw, unsigned nowMs) {
    Node* n = (Node*)h;
    if (n->sensor.Edge(raw != 0)) n->agent.OnTargetSeen(nowMs);
}
void bdie_sensor_active(void* h, int on) {
    Node* n = (Node*)h;
    n->sensor.active = (on != 0);
    if (!n->sensor.active) n->sensor.prev = false;
}

void bdie_params(void* h, float alpha, float beta, int rho, int deposit,
                 int commitment, unsigned noProgressMs, unsigned congregationMs,
                 unsigned missionMs) {
    Agent* a = A(h);
    a->map.alpha      = alpha;
    a->map.beta       = beta;
    a->map.rho        = (uint8_t)rho;
    a->map.deposit    = (uint8_t)deposit;
    a->commitment     = (Commitment)commitment;
    a->noProgressMs   = noProgressMs;
    a->congregationMs = congregationMs;
    a->missionTimeoutMs = missionMs;
}

/* Período de evaporación de la feromona, para poder ablacionarlo con las mismas
 * semillas. Es LA palanca del ritmo: rho no lo es en este rango (ver la nota de
 * cuantización en pheromone_map.h). */
void bdie_evaporacion(void* h, unsigned periodoMs) {
    if (periodoMs > 0) A(h)->evaporatePeriodMs = periodoMs;
}

/* Bono de novedad y plan de frontera (12-oct-2026), para ablacionarlos con las
 * mismas semillas: novedad = 1 y frontera = 0 reproducen la exploración anterior. */
void bdie_exploracion(void* h, float novedad, int frontera) {
    A(h)->map.novelty      = novedad;
    A(h)->frontierEnabled  = (frontera != 0);
}

/* Modo de formación en el retorno y ancho de la banda. Es el parámetro que el
 * banco barrió para encontrar que el modo 0 no acota el radio del enjambre. */
void bdie_formation(void* h, int mode, float bandMm, unsigned waitMs, unsigned graceMs) {
    Agent* a = A(h);
    a->formationMode   = mode;
    a->formationMaxMm  = bandMm;
    a->formationWaitMs = waitMs;
    a->returnGraceMs   = graceMs;
}

/* Una sola puerta de entrada: el banco alimenta al agente con EXACTAMENTE las
 * mismas cadenas que viajan por UDP en el robot. Un defecto de interpretación
 * del protocolo no puede existir en uno y no en el otro. */
void bdie_feed(void* h, const char* line, unsigned nowMs) { Feed(*A(h), line, nowMs); }

void bdie_target_seen(void* h, unsigned nowMs) { A(h)->OnTargetSeen(nowMs); }
void bdie_sensors(void* h, int f, int l, int r, int batt) {
    A(h)->OnSensors(f != 0, l != 0, r != 0, batt != 0);
}
void bdie_arrived(void* h) { A(h)->OnArrived(); }
/* El MEET del laboratorio (lab_cong.cpp) llegó a la ranura. */
void bdie_slot_reached(void* h) { A(h)->OnSlotReached(); }
/* ¿El MEET en curso apunta a un objetivo que ya no es el vigente? */
int  bdie_meet_stale(void* h) { return A(h)->MeetStale() ? 1 : 0; }
void bdie_block(void* h, int cell) { A(h)->map.Block(cell); }

void bdie_cycle(void* h, unsigned nowMs, COut* o) {
    Agent* a = A(h);
    const Action act = a->Cycle(nowMs);
    o->kind    = (int)act.kind;
    o->cell    = act.cell;
    o->x       = act.x;
    o->y       = act.y;
    o->phase   = (int)a->bel.phase;
    o->desire  = (int)a->desire;
    o->zone    = a->bel.myZone;
    o->myCell  = a->bel.myCell;
    o->coverage = a->map.CoverageCells();
    o->targetOwner = a->bel.targetOwner;
    o->congregationClosed = a->bel.congregationClosed ? 1 : 0;
    o->arrivedAtPlace     = a->bel.arrivedAtPlace ? 1 : 0;
    o->iFoundIt           = a->bel.iFoundIt ? 1 : 0;
}

/* Los mensajes salen ya codificados como texto, igual que por la radio. */
int bdie_pop_wire(void* h, char* dst, int cap) {
    Wire w;
    if (!Emit(*A(h), w)) return 0;
    const int n = (w.len < cap - 1) ? w.len : cap - 1;
    std::memcpy(dst, w.buf, (size_t)n);
    dst[n] = '\0';
    return n;
}

/* Telemetría: el MISMO codificador que usa el robot (telemetry.h). El banco
 * puede así alimentar a la base y a su GUI con los mensajes exactos que
 * mandaría el enjambre físico. Devuelven la longitud, o -1 si no cabe. */
int bdie_telem_think(void* h, unsigned nowMs, char* dst, int cap) {
    return EncodeThink(*A(h), nowMs, dst, cap);
}
int bdie_telem_map(void* h, unsigned nowMs, char* dst, int cap) {
    return EncodeMap(*A(h), nowMs, dst, cap);
}

int   bdie_cells(void* h)     { return A(h)->map.n; }
const char* bdie_plan(void* h){ return A(h)->lastPlan; }
void  bdie_repel(void* h, unsigned char* dst, int n) {
    Agent* a = A(h);
    const int k = (n < a->map.n) ? n : a->map.n;
    std::memcpy(dst, a->map.repel, (size_t)k);
}
void  bdie_visited(void* h, unsigned char* dst, int n) {
    Agent* a = A(h);
    const int k = (n < a->map.n) ? n : a->map.n;
    for (int i = 0; i < k; ++i) dst[i] = a->map.Visited(i) ? 1 : 0;
}
void  bdie_zone_mask(void* h, unsigned char* dst, int n) {
    Agent* a = A(h);
    uint8_t m[MAX_CELLS];
    a->ZoneMaskInto(m);
    const int k = (n < a->map.n) ? n : a->map.n;
    std::memcpy(dst, m, (size_t)k);
}

/* ── Accesores del roster ────────────────────────────────────────────────────
 *
 * Existen para la verificación de integración base ↔ firmware: el banco genera
 * los paquetes con `base/swarm_broadcast.py` —el MISMO código que va a correr
 * en la PC del laboratorio— y comprueba acá que el decodificador del firmware
 * los interpreta como corresponde. Sin estos accesores habría que creerle al
 * `Feed()` sin mirar, que es justamente la clase de cosa que este proyecto no
 * hace. */
int bdie_roster_n(void* h)      { return A(h)->roster.n; }
unsigned bdie_epoch(void* h)    { return (unsigned)A(h)->roster.epoch; }
int bdie_alive_count(void* h)   { return A(h)->roster.AliveCount(); }
unsigned bdie_alive_mask(void* h){ return (unsigned)A(h)->roster.AliveMask(); }
int bdie_peer(void* h, int id, float* x, float* y, float* th, unsigned* ep) {
    Agent* a = A(h);
    if (id < 0 || id >= MAX_ROBOTS || !a->roster.p[id].seen) return 0;
    if (x)  *x  = a->roster.p[id].x;
    if (y)  *y  = a->roster.p[id].y;
    if (th) *th = a->roster.p[id].th;
    if (ep) *ep = (unsigned)a->roster.p[id].epoch;
    return a->roster.Alive(id) ? 1 : 0;
}
int bdie_base(void* h, float* x, float* y) {
    Agent* a = A(h);
    if (x) *x = a->bel.baseX;
    if (y) *y = a->bel.baseY;
    return a->bel.session ? 1 : 0;
}

/* ── El reparto por mensajes (14-oct-2026) ──────────────────────────────────
 *
 * bdie_comunicar es BdieLoopHook §2: el latido y el reparto corren por reloj
 * propio en CADA loop() del robot, no solo cuando el agente delibera. El banco
 * lo llama en cada paso para cada robot vivo; sin esto, mientras el cuerpo
 * navega el agente no latiría ni reclamaría, y el banco sería más pesimista que
 * el robot. */
void bdie_comunicar(void* h, unsigned nowMs) { A(h)->Comunicar(nowMs); }

/* VIVO = LO OIGO: el conjunto que este agente oye (bit i = robot i). */
unsigned bdie_talking_mask(void* h, unsigned nowMs) {
    return (unsigned)A(h)->TalkingMask(nowMs);
}

/* Estado de la ronda y contadores, para las métricas del banco. Devuelve la
 * franja VIGENTE del agente (-1 si todavía no tiene). */
int bdie_rep_info(void* h, unsigned* rep, unsigned* mask, int* adopted,
                  unsigned* rounds, unsigned* sent, unsigned* agreements,
                  unsigned* fallbacks, unsigned* assignEpoch, unsigned* assignMask) {
    Agent* a = A(h);
    if (rep)         *rep         = a->rp.rep;
    if (mask)        *mask        = a->rp.mask;
    if (adopted)     *adopted     = a->rp.adopted ? 1 : 0;
    if (rounds)      *rounds      = a->rp.rounds;
    if (sent)        *sent        = a->rp.sent;
    if (agreements)  *agreements  = a->rp.agreements;
    if (fallbacks)   *fallbacks   = a->rp.fallbacks;
    if (assignEpoch) *assignEpoch = a->assign.valid ? a->assign.epoch : 0;
    if (assignMask)  *assignMask  = a->assign.valid ? a->assign.mask : 0;
    return a->bel.myZone;
}

/* La franja que ESTE agente cree que tiene el robot `id` en el reparto vigente. */
int bdie_zone_of(void* h, int id) {
    Agent* a = A(h);
    if (!a->assign.valid || id < 0 || id >= MAX_ROBOTS) return -1;
    return a->assign.zoneOf[id];
}

/* Parámetros del reparto y de la vivacidad, para barrerlos con las mismas
 * semillas. 0 = no cambiar. */
void bdie_reparto_params(void* h, unsigned settleMs, unsigned resendMs, unsigned keepMs,
                         unsigned graceMs, unsigned fallbackMs, unsigned peerDeadMs) {
    Agent* a = A(h);
    if (settleMs)   a->rp.settleMs     = settleMs;
    if (resendMs)   a->rp.resendMs     = resendMs;
    if (keepMs)     a->rp.keepMs       = keepMs;
    if (graceMs)    a->rp.startGraceMs = graceMs;
    if (fallbackMs) a->rp.fallbackMs   = fallbackMs;
    if (peerDeadMs) a->peerDeadMs      = peerDeadMs;
}

}  // extern "C"
