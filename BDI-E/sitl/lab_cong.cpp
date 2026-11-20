/*****************************************************************************
 * lab_cong.cpp — Congregación del laboratorio, compilada para el banco
 *
 * BDI-E no tiene congregación propia: al conocer el objetivo le pide un MEET
 * al firmware del laboratorio (sección de congregación de bdie.ino). Para que
 * el banco mida esa congregación y no una imitación, este archivo compila las
 * funciones del laboratorio tal cual: extraer_lab.py las copia de los .ino de
 * robot/AttaBot a lab_cong_gen.inc, y aquí se incluyen dentro de la estructura
 * LabBody.
 *
 * Dentro de la estructura, las globales del sketch (congregation, nav,
 * disperse, robotPose, instructionList…) son miembros, así que cada robot del
 * banco tiene su propio estado y las copias no se pisan. El código de las
 * funciones no cambia.
 *
 * Lo que no viene del laboratorio:
 *   · SendMessage, MessageDebugf  radio y registro: el banco no los necesita.
 *   · SmoothLeaderPose            solo lo usa un líder, y en el MEET no hay.
 *   · MaybeDisperseHop            la dispersión está apagada en la misión
 *                                 (disperse.target = 0).
 *   · agent.OnSlotReached()       el enganche hacia el agente, que en el banco
 *                                 está en otra biblioteca; aquí levanta una
 *                                 bandera que bdie_sim.py le pasa al agente.
 *
 * Compila en C++17 contra los stubs de pruebas_host/ (Arduino.h y compañía),
 * en su propia unidad de traducción, para que las macros de Arduino (PI, min,
 * max) no se mezclen con los encabezados de BDI-E.
 *****************************************************************************/
#include "Arduino.h"
#include "utils.h"
#include <array>
#include <deque>
#include <map>

unsigned long g_ms = 0;   // el reloj de millis() del stub; lo fija el banco

struct LabBody {
    // ── Las globales del sketch que tocan estas funciones (AttaBot.ino) ─────
    CongregationState congregation;
    DisperseState     disperse;
    EKFState          ekf;
    ObstacleState     obstacles;
    ReactiveNav       nav;
    pose              robotPose{0, 0, 0};
    std::array<float, 2>             fsmInstruction{};
    std::deque<std::array<float, 2>> instructionList;
    std::map<String, IPAddress>      robots;
    String robotID = "-1";
    bool   isEvading     = false;
    bool   ekfNavEnabled = false;
    int    blindNavSteps = 0;
    float  arenaWidthMm  = 2400.0f;
    float  arenaHeightMm = 1750.0f;
    float  centerToWheelDistance = 41.5f;   // como en AttaBot.ino
    bool   missionActive = true;            // el banco solo corre misiones
    bool   bdieOwnsMeet  = false;
    unsigned long navWaitAfterTurnMs = 300;  // pausas de ReactiveNavStep, como en AttaBot.ino
    unsigned long navWaitAfterMoveMs = 300;

    struct AgentProxy {                     // agent.OnSlotReached() → bandera
        bool llego = false;
        void OnSlotReached() { llego = true; }
    } agent;

    // ── Lo que el banco no necesita (ver cabecera) ───────────────────────────
    void SendMessage(IPAddress, const char*) {}
    void MessageDebugf(const char*, ...) {}
    void SmoothLeaderPose(float x, float y, float a, float* ox, float* oy, float* oa) {
        *ox = x; *oy = y; *oa = a;
    }
    void MaybeDisperseHop() {}
    void BdieMarcarBarrido() {}             // el barrido de celda no existe en el banco

    // ── El código del laboratorio, tal cual ─────────────────────────────────
#include "lab_cong_gen.inc"
};

extern "C" {

struct LabOut {
    int   navActive, navPending, meetActive, stagingDone, slotIndex, slotTotal;
    float goalX, goalY, slotX, slotY, ring, slotRadius;
    int   queue;
};

void* lab_new(int id, float arenaW, float arenaH, float arrivalMm) {
    LabBody* b = new LabBody();
    char buf[12]; snprintf(buf, sizeof(buf), "%d", id);
    b->robotID = String(buf);
    b->arenaWidthMm  = arenaW;
    b->arenaHeightMm = arenaH;
    b->nav.arrivalThreshold = arrivalMm;    // BdieInitGrid: RecommendedArrivalMm
    return b;
}
void lab_free(void* h) { delete (LabBody*)h; }
void lab_clock(unsigned long ms) { g_ms = ms; }

static std::array<String, 6> Args(const char* line) {
    std::array<String, 6> a;
    int k = 0; std::string cur;
    for (const char* p = line; ; ++p) {
        if (*p == '|' || *p == '\0') {
            if (k < 6) a[k++] = String(cur.c_str());
            cur.clear();
            if (*p == '\0') break;
        } else cur += *p;
    }
    return a;
}

// Los mismos puntos de entrada que en el robot:
void lab_start_meet(void* h, float x, float y) { ((LabBody*)h)->BdieStartMeet(x, y); }
void lab_drop_meet(void* h) {
    LabBody* b = (LabBody*)h;
    b->BdieDropMeet();
    b->nav.Reset();                 // BdieReleaseBody
    b->instructionList.clear();
}
void lab_neighbors(void* h, const char* line) {           // NEIGHBOR_POSITIONS|…
    ((LabBody*)h)->HandleNeighborPositions(Args(line));
}
/* StateRequestPosition marca la pregunta; POSITION_RESPONSE la contesta. */
void lab_position(void* h, float x, float y, float ang) {
    LabBody* b = (LabBody*)h;
    b->congregation.StartRequest();
    char line[96];
    snprintf(line, sizeof(line), "POSITION_RESPONSE|%.1f|%.1f|%.1f", x, y, ang);
    b->HandlePositionResponse(Args(line));
    b->congregation.CompleteRequest();
}
void lab_obstacles(void* h, int left, int center, int right) {
    LabBody* b = (LabBody*)h;
    b->obstacles.leftObstacle    = left   != 0;
    b->obstacles.centralObstacle = center != 0;
    b->obstacles.rightObstacle   = right  != 0;
}
/* La siguiente instrucción de la cola (TURN arco, MOVE mm, WAIT ms,
 * REQUEST_POSITION). Devuelve 0 si la cola está vacía. */
int lab_pop(void* h, int* kind, float* value) {
    LabBody* b = (LabBody*)h;
    if (b->instructionList.empty()) return 0;
    *kind  = (int)b->instructionList.front()[0];
    *value = b->instructionList.front()[1];
    b->instructionList.pop_front();
    return 1;
}
/* El guardián de BdieLoopHook (bdie.ino): nav.HasTimedOut() solo se evalúa
 * dentro de ReactiveNavStep, que no corre mientras el robot espera a otro; el
 * robot lo evalúa en cada loop() y suelta el cuerpo. Devuelve 1 si lo soltó. */
int lab_guardian(void* h) {
    LabBody* b = (LabBody*)h;
    if (b->nav.isActive && b->nav.HasTimedOut()) {
        b->nav.Reset();
        b->instructionList.clear();
        b->BdieDropMeet();
        return 1;
    }
    return 0;
}
int lab_take_arrival(void* h) {
    LabBody* b = (LabBody*)h;
    const int r = b->agent.llego ? 1 : 0;
    b->agent.llego = false;
    return r;
}
void lab_state(void* h, LabOut* o) {
    LabBody* b = (LabBody*)h;
    o->navActive   = b->nav.isActive;
    o->navPending  = b->nav.pendingInit;
    o->meetActive  = b->bdieOwnsMeet && b->congregation.IsActive();
    o->stagingDone = b->congregation.stagingDone;
    o->slotIndex   = b->congregation.followerIndex;
    o->slotTotal   = b->congregation.totalFollowers;
    o->goalX = b->nav.goalX;  o->goalY = b->nav.goalY;
    o->slotX = b->congregation.slotX;  o->slotY = b->congregation.slotY;
    o->ring  = b->congregation.parkingDist;
    o->slotRadius = b->congregation.slotRadius;
    o->queue = (int)b->instructionList.size();
}
// Códigos del enum RobotState de utils.h, para que Python no los copie a mano.
int lab_code(const char* nombre) {
    const std::string s(nombre);
    if (s == "WAIT") return WAIT;
    if (s == "MOVE") return MOVE;
    if (s == "TURN") return TURN;
    if (s == "REQUEST_POSITION") return REQUEST_POSITION;
    return -1;
}

}  // extern "C"
