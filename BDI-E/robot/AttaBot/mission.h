/*****************************************************************************
 * mission.h — Protocolo de mensajes y acople con el firmware
 *
 * Dos partes separadas:
 *
 *   §1 Protocolo (sin Arduino). Codifica y decodifica los mensajes de BDI-E:
 *      texto plano con '|' como separador, igual que el resto del firmware, y
 *      a lo sumo cinco campos por mensaje, que es lo que admite el analizador
 *      de comandos del firmware (SeparateCommand en comandos.ino). La prueba
 *      T10 verifica esa cota en la PC.
 *
 *   §2 Acople con Arduino (#ifdef ARDUINO). Traduce los mensajes entrantes a
 *      eventos del agente y las Action del agente a instrucciones de la cola
 *      instructionList y de la navegación reactiva.
 *
 * Mensajes:
 *   POSES|epoch|blob            base → todos   blob = "id:x:y:a;id:x:y:a;…"
 *   SESSION|START|epoch|hx|hy   base → todos   inicio y posición de la base
 *   SESSION|ABORT               base → todos   aborto de la corrida
 *   HB|id|fase|flags|seq        robot → todos  latido a 2 Hz
 *   TGT|epoch|id|x|y            robot → todos  hallazgo, difusión epidémica
 *   TGT_REQ|id                  robot → todos  petición explícita del hallazgo
 *   ZC|ronda:part|id|zona|dist  robot → todos  reclamo de franja
 *   ZA|ronda:part|id|zona|dist  robot → todos  franja acordada
 *
 * En ZC y ZA, `part` son los participantes de la ronda en hexadecimal (bit i =
 * robot i), unidos a la ronda con ':' para no pasar de cinco campos, y `dist`
 * son las celdas desde el ancla del robot hasta el centro de la franja (ver
 * «Reparto por mensajes» en assignment.h).
 *
 * Ningún mensaje va dirigido a la base. La base escucha para registrar
 * métricas, pero lo que envía a los robots son datos, nunca órdenes.
 *****************************************************************************/
#ifndef BDIE_MISSION_H
#define BDIE_MISSION_H

#include "bdi.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace bdie {

/*===========================================================================*
 *  §1  PROTOCOLO — sin dependencias de Arduino
 *===========================================================================*/

/* Cota del protocolo: comando + 4 argumentos. */
static const int MAX_FIELDS  = 5;
static const int MAX_WIRE    = 128;   // muy por debajo del búfer de 255 B del firmware

struct Wire {
    char buf[MAX_WIRE];
    int  len    = 0;
    int  fields = 0;
    Wire() { buf[0] = '\0'; }
};

/* Cuenta campos separados por '|'. Hace verificable la cota de cinco campos
 * en la PC (prueba T10). */
inline int CountFields(const char* s) {
    if (!s || !*s) return 0;
    int n = 1;
    for (const char* p = s; *p; ++p) if (*p == '|') ++n;
    return n;
}

inline void Finish(Wire& w) {
    w.len    = (int)std::strlen(w.buf);
    w.fields = CountFields(w.buf);
}

inline void EncodeHb(Wire& w, int id, uint8_t phase, uint8_t flags, uint32_t seq) {
    std::snprintf(w.buf, MAX_WIRE, "HB|%d|%u|%u|%lu",
                  id, (unsigned)phase, (unsigned)flags, (unsigned long)seq);
    Finish(w);
}

inline void EncodeTgt(Wire& w, uint32_t epoch, int ownerId, float x, float y) {
    std::snprintf(w.buf, MAX_WIRE, "TGT|%lu|%d|%.0f|%.0f",
                  (unsigned long)epoch, ownerId, (double)x, (double)y);
    Finish(w);
}

inline void EncodeTgtReq(Wire& w, int id) {
    std::snprintf(w.buf, MAX_WIRE, "TGT_REQ|%d", id);
    Finish(w);
}

/* Reclamo (ZC) o reparto acordado (ZA). */
inline void EncodeZone(Wire& w, bool acordado, uint32_t rep, uint32_t mask, int id,
                       int zone, int dist) {
    std::snprintf(w.buf, MAX_WIRE, "%s|%lu:%lx|%d|%d|%d", acordado ? "ZA" : "ZC",
                  (unsigned long)rep, (unsigned long)mask, id, zone, dist);
    Finish(w);
}

inline void EncodeSessionStart(Wire& w, uint32_t epoch, float hx, float hy) {
    std::snprintf(w.buf, MAX_WIRE, "SESSION|START|%lu|%.0f|%.0f",
                  (unsigned long)epoch, (double)hx, (double)hy);
    Finish(w);
}

inline void EncodeSessionAbort(Wire& w) {
    std::snprintf(w.buf, MAX_WIRE, "SESSION|ABORT");
    Finish(w);
}

/* POSES lleva todas las poses en un solo campo, separadas con ';' y ':', que
 * SeparateCommand no parte. Con cinco robots el paquete mide unos 79 B. */
inline void EncodePoses(Wire& w, uint32_t epoch, const Roster& r) {
    int n = std::snprintf(w.buf, MAX_WIRE, "POSES|%lu|", (unsigned long)epoch);
    for (int i = 0; i < r.n && i < MAX_ROBOTS; ++i) {
        if (!r.p[i].seen) continue;
        n += std::snprintf(w.buf + n, (size_t)(MAX_WIRE - n), "%d:%.0f:%.0f:%.0f;",
                           i, (double)r.p[i].x, (double)r.p[i].y, (double)r.p[i].th);
        if (n >= MAX_WIRE - 1) break;
    }
    Finish(w);
}

/*---------------------------------------------------------------------------*
 *  Decodificación
 *---------------------------------------------------------------------------*/
enum WireKind : uint8_t {
    W_UNKNOWN = 0, W_POSES, W_SESSION_START, W_SESSION_ABORT, W_HB, W_TGT, W_TGT_REQ,
    W_ZC, W_ZA
};

struct PoseItem { int id; float x, y, th; };

struct Decoded {
    WireKind kind = W_UNKNOWN;
    uint32_t epoch = 0;
    int      id = 0;
    uint8_t  phase = 0, flags = 0;
    uint32_t seq = 0;
    float    x = 0.0f, y = 0.0f;
    uint32_t mask = 0;            // ZC/ZA: participantes de la ronda
    int      zone = -1, dist = 0; // ZC/ZA
    PoseItem pose[MAX_ROBOTS];
    int      poseCount = 0;
    int      fields = 0;
};

inline int SplitFields(const char* s, char out[MAX_FIELDS][32], int maxCopy) {
    int nf = 0, j = 0;
    for (const char* p = s; ; ++p) {
        if (*p == '|' || *p == '\0') {
            if (nf < MAX_FIELDS) { out[nf][j < maxCopy ? j : maxCopy - 1] = '\0'; }
            ++nf; j = 0;
            if (*p == '\0') break;
            continue;
        }
        if (nf < MAX_FIELDS && j < maxCopy - 1) out[nf][j++] = *p;
    }
    return nf;
}

inline bool Decode(const char* s, Decoded& d) {
    if (!s || !*s) return false;
    d = Decoded();
    d.fields = CountFields(s);

    char f[MAX_FIELDS][32];
    for (int i = 0; i < MAX_FIELDS; ++i) f[i][0] = '\0';
    SplitFields(s, f, 32);

    if (!std::strcmp(f[0], "HB")) {
        d.kind  = W_HB;
        d.id    = std::atoi(f[1]);
        d.phase = (uint8_t)std::atoi(f[2]);
        d.flags = (uint8_t)std::atoi(f[3]);
        d.seq   = (uint32_t)std::strtoul(f[4], 0, 10);
        return true;
    }
    if (!std::strcmp(f[0], "TGT")) {
        d.kind  = W_TGT;
        d.epoch = (uint32_t)std::strtoul(f[1], 0, 10);
        d.id    = std::atoi(f[2]);
        d.x     = (float)std::atof(f[3]);
        d.y     = (float)std::atof(f[4]);
        return true;
    }
    if (!std::strcmp(f[0], "TGT_REQ")) {
        d.kind = W_TGT_REQ;
        d.id   = std::atoi(f[1]);
        return true;
    }
    if (!std::strcmp(f[0], "ZC") || !std::strcmp(f[0], "ZA")) {
        d.kind = (f[0][1] == 'A') ? W_ZA : W_ZC;
        char* fin = 0;
        d.epoch = (uint32_t)std::strtoul(f[1], &fin, 10);       // la ronda
        d.mask  = (fin && *fin == ':') ? (uint32_t)std::strtoul(fin + 1, 0, 16) : 0u;
        d.id    = std::atoi(f[2]);
        d.zone  = std::atoi(f[3]);
        d.dist  = std::atoi(f[4]);
        return d.fields == 5;
    }
    if (!std::strcmp(f[0], "SESSION")) {
        if (!std::strcmp(f[1], "START")) {
            d.kind  = W_SESSION_START;
            d.epoch = (uint32_t)std::strtoul(f[2], 0, 10);
            d.x     = (float)std::atof(f[3]);
            d.y     = (float)std::atof(f[4]);
        } else {
            d.kind = W_SESSION_ABORT;
        }
        return true;
    }
    if (!std::strcmp(f[0], "POSES")) {
        d.kind  = W_POSES;
        d.epoch = (uint32_t)std::strtoul(f[1], 0, 10);
        /* El blob se recorre sobre la cadena original: puede ser más largo que los
         * 32 caracteres del campo recortado, y truncarlo perdería robots. */
        const char* p = std::strchr(s, '|');
        if (p) p = std::strchr(p + 1, '|');
        if (p) {
            ++p;
            while (*p && d.poseCount < MAX_ROBOTS) {
                int   id = 0;
                float x = 0, y = 0, th = 0;
                if (std::sscanf(p, "%d:%f:%f:%f", &id, &x, &y, &th) == 4) {
                    d.pose[d.poseCount].id = id;
                    d.pose[d.poseCount].x  = x;
                    d.pose[d.poseCount].y  = y;
                    d.pose[d.poseCount].th = th;
                    ++d.poseCount;
                }
                const char* nx = std::strchr(p, ';');
                if (!nx) break;
                p = nx + 1;
            }
        }
        return true;
    }
    return false;
}

/*---------------------------------------------------------------------------*
 *  Mensaje entrante → agente. Es la única puerta de entrada, y el banco SITL
 *  y el firmware usan esta misma función, de modo que la interpretación del
 *  protocolo es idéntica en ambos.
 *---------------------------------------------------------------------------*/
inline void Feed(Agent& a, const char* line, uint32_t nowMs) {
    Decoded d;
    if (!Decode(line, d)) return;
    switch (d.kind) {
    case W_POSES:
        for (int i = 0; i < d.poseCount; ++i)
            a.OnPose(d.pose[i].id, d.pose[i].x, d.pose[i].y, d.pose[i].th, d.epoch);
        break;
    case W_SESSION_START:  a.StartMission(d.x, d.y, d.epoch, nowMs);          break;
    case W_SESSION_ABORT:  a.AbortMission(nowMs);                             break;
    case W_HB:
        /* El latido propio vuelve por la difusión y se ignora. */
        if (d.id != a.selfId) a.OnHeartbeat(d.id, d.phase, d.flags, d.seq, nowMs);
        break;
    case W_TGT:            a.OnTargetReport(d.epoch, d.id, d.x, d.y, nowMs);  break;
    case W_TGT_REQ:        a.Heard(d.id, nowMs); a.OnTargetRequest(d.id, nowMs); break;
    case W_ZC:
    case W_ZA:
        /* El reclamo propio también vuelve por la difusión; OnZoneClaim lo ignora. */
        a.OnZoneClaim(d.epoch, d.mask, d.id, d.zone, d.dist, nowMs);
        break;
    default: break;
    }
}

/* Mensaje saliente del agente → texto. */
inline bool Emit(Agent& a, Wire& w) {
    Msg m;
    if (!a.out.Pop(m)) return false;
    switch (m.kind) {
    case MSG_HB:      EncodeHb(w, m.a, (uint8_t)m.b, (uint8_t)m.c, (uint32_t)m.d); return true;
    case MSG_TGT:     EncodeTgt(w, (uint32_t)m.a, m.b, m.f1, m.f2);                return true;
    case MSG_TGT_REQ: EncodeTgtReq(w, m.a);                                        return true;
    case MSG_ZC:
    case MSG_ZA:      EncodeZone(w, m.kind == MSG_ZA, (uint32_t)m.a, m.e, m.b, m.c, m.d);
                      return true;
    default:          w.buf[0] = '\0'; w.len = 0; w.fields = 0;                    return true;
    }
}

/*===========================================================================*
 *  §2  Acople con Arduino
 *
 *  Solo esta parte conoce tipos del sketch. El acople es simple porque
 *  instructionList ya es la cola de instrucciones que consume la máquina de
 *  estados del laboratorio: el agente no mueve las ruedas, encola
 *  instrucciones en esa misma cola. Lo que está del otro lado (PID, encoders,
 *  IMU, evasión) no se modifica.
 *===========================================================================*/
#ifdef ARDUINO

/* Traduce una Action del agente a instrucciones del cuerpo del robot.
 *
 * A_GOTO_CELL y A_GOTO_MM se resuelven igual, con la navegación reactiva del
 * laboratorio (ReactiveNav en utils.h), que ya sabe ir a un punto en mm
 * esquivando obstáculos; solo cambia de dónde sale el punto.
 *
 * Devuelve true si el robot debe frenar. Así esta función no necesita ver la
 * variable global `state`, y la transición la decide el sketch.
 *
 * El umbral de llegada se fija una vez desde la base con
 * NAV_CONFIG|ARRIVAL_THRESHOLD|<mm>, con el valor de RecommendedArrivalMm()
 * (grid.h). */
inline bool ApplyAction(const Action& act) {
    switch (act.kind) {
    case Action::A_GOTO_CELL:
    case Action::A_GOTO_MM:
        /* La navegación del firmware es un lazo de pregunta y respuesta con la
         * base: cada paso lo da ReactiveNavStep() (navegacion.ino), que encola giro,
         * espera, avance, espera y REQUEST_POSITION, y el manejador de
         * POSITION_RESPONSE (comandos.ino) es el que llama al paso siguiente. Por eso
         * no basta con fijar la meta: hay que encolar el primer REQUEST_POSITION,
         * igual que hace el comando GOTO del firmware. */
        nav.goalX       = act.x;
        nav.goalY       = act.y;
        nav.pendingInit = true;
        nav.isActive    = false;
        instructionList.clear();
        fsmInstruction[0] = REQUEST_POSITION;
        fsmInstruction[1] = 0;
        instructionList.push_back(fsmInstruction);
        return false;
    case Action::A_HOLD:
        instructionList.clear();
        return false;
    case Action::A_ABORT:
        instructionList.clear();
        nav.Reset();
        return true;                 // el sketch pone state = STOP
    case Action::A_MEET:
        /* La congregación es el MEET del laboratorio (comandos.ino); la dispara
         * BdieStartMeet() en bdie.ino. */
        return false;
    case Action::A_NONE:
    default:
        return false;
    }
}

/* Difunde los mensajes que el agente encoló, por el mismo canal de difusión
 * UDP que usan los mensajes robot a robot del firmware. El destino de
 * difusión lo fija la base con el comando CONFIG. */
inline void PumpOutgoing(Agent& agent) {
    Wire w;
    while (Emit(agent, w)) {
        if (w.len == 0) continue;
        SendMessage(robots["Broadcast"], w.buf);
    }
}

#endif  // ARDUINO

}  // namespace bdie
#endif  // BDIE_MISSION_H
