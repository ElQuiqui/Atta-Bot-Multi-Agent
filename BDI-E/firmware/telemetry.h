/*****************************************************************************
 * telemetry.h — Telemetría de razonamiento hacia la base
 *
 * Instrumentación, no protocolo de misión: ningún robot lee estos mensajes y
 * la misión no depende de que lleguen. Van por unicast a la base, para que los
 * demás robots no gasten su analizador en paquetes que no les sirven.
 *
 * Sin dependencias de Arduino: el robot y el banco (sitl/bdie_bridge.cpp)
 * codifican con estas mismas funciones, así que la GUI muestra en el banco
 * exactamente lo que mostraría con robots.
 *
 * Dos mensajes, en formato clave=valor (la base los parte por '|' y '='):
 *
 *   THINK|id=2|t=..|fase=BUSCAR|deseo=EXPLORAR|plan=P_EXPLORAR_ACO|meta=45|
 *        celda=44|zona=1|enzona=1|x=812|y=430|obj=1|dueno=2|cobj=47|cerr=0|
 *        lleg=0|meet=0|vivos=1f|delib=312|cob=58|ev=ARRIVED|tev=..|reint=0|
 *        sinprog=3200|rep=3|racu=1|zfb=0
 *     vivos = máscara de los robots que el agente oye; rep = ronda del
 *     reparto; racu = 1 si el robot la dio por acordada; zfb = veces que se
 *     usó el reparto de respaldo. Se envía a 2 Hz, y de inmediato (con un piso
 *     de 100 ms) cuando cambia algo que el agente decidió: fase, deseo, plan,
 *     meta, zona, objetivo o cierre.
 *
 *   BMAP|id=2|t=..|g=filas,cols,celda,ox,oy|kz=5|z=r0.r1.c0.c1;..|
 *        of=id:zona,..|R=<hex>|V=<hex>|B=<hex>
 *     A 1 Hz. R: un byte de feromona por celda; V y B: mapas de bits de
 *     cobertura acumulada y de celdas bloqueadas. Cabe en un datagrama.
 *
 * BDIE_TELEM|0 apaga la telemetría (para medir latencia sin ella) y
 * BDIE_TELEM|1 deja solo THINK.
 *****************************************************************************/
#ifndef BDIE_TELEMETRY_H
#define BDIE_TELEMETRY_H

#include "bdi.h"
#include <cstdio>
#include <cstring>

namespace bdie {

static const char* const kPhaseName[] = {
    "IDLE", "DESPLEGAR", "BUSCAR", "CONGREGAR", "VOLVER", "FIN", "SEGURO"
};
static const char* const kEventName[] = {
    "-", "ARRIVED", "TARGET_SEEN", "TARGET_REPORTED", "TIMEOUT", "BATTERY_LOW",
    "MISSION_TIMEOUT", "CONG_CLOSED", "SESSION_START", "SESSION_ABORT", "ZONE_AGREED"
};
inline const char* PhaseName(int p) { return (p >= 0 && p <= PH_SEGURO) ? kPhaseName[p] : "?"; }
inline const char* EventName(int e) { return (e >= 0 && e <= EV_ZONE_AGREED) ? kEventName[e] : "?"; }

/*---------------------------------------------------------------------------*
 *  Cuándo mandar
 *---------------------------------------------------------------------------*/
struct TelemTicker {
    uint32_t thinkMs = 500, idleMs = 2000, mapMs = 1000, minGapMs = 100;
    uint32_t tThink = 0, tMap = 0;
    uint32_t firma = 0xFFFFFFFFu;
    const char* plan = 0;

    static uint32_t Firma(const Agent& a) {
        uint32_t h = 2166136261u;
        const int v[] = { (int)a.bel.phase, (int)a.desire, a.intention.goalCell,
                          a.bel.myZone, a.bel.targetKnown ? 1 : 0, a.bel.targetOwner,
                          a.bel.congregationClosed ? 1 : 0, a.bel.arrivedAtPlace ? 1 : 0 };
        for (int x : v) { h ^= (uint32_t)x; h *= 16777619u; }
        return h;
    }
    bool DueThink(const Agent& a, uint32_t now) {
        const bool activo = a.bel.phase != PH_IDLE;
        const uint32_t f = Firma(a);
        const bool cambio = (f != firma) || (a.lastPlan != plan);
        const uint32_t dt = now - tThink;
        if ((cambio && dt >= minGapMs) || tThink == 0 || dt >= (activo ? thinkMs : idleMs)) {
            tThink = now ? now : 1; firma = f; plan = a.lastPlan;
            return true;
        }
        return false;
    }
    bool DueMap(const Agent& a, uint32_t now) {
        if (a.bel.phase == PH_IDLE) return false;      // sin misión no hay mapa que ver
        if (tMap == 0 || now - tMap >= mapMs) { tMap = now ? now : 1; return true; }
        return false;
    }
    void Reset() { tThink = tMap = 0; firma = 0xFFFFFFFFu; plan = 0; }
};

/*---------------------------------------------------------------------------*
 *  Codificación
 *---------------------------------------------------------------------------*/
inline int EncodeThink(const Agent& a, uint32_t now, char* dst, int cap) {
    const Intention& in = a.intention;
    const uint32_t sinProg = (in.active && in.tProgress) ? now - in.tProgress : 0;
    const int n = std::snprintf(dst, (size_t)cap,
        "THINK|id=%d|t=%lu|fase=%s|deseo=%s|plan=%s|meta=%d|celda=%d|zona=%d|enzona=%d|"
        "x=%.0f|y=%.0f|obj=%d|dueno=%d|cobj=%d|cerr=%d|lleg=%d|meet=%d|vivos=%lx|"
        "delib=%lu|cob=%d|ev=%s|tev=%lu|reint=%d|sinprog=%lu|rep=%lu|racu=%d|zfb=%lu",
        a.selfId, (unsigned long)now, PhaseName(a.bel.phase),
        kDesireName[(int)a.desire < D_COUNT ? (int)a.desire : 0],
        (a.lastPlan && *a.lastPlan) ? a.lastPlan : "-",
        in.active ? in.goalCell : -1, a.bel.myCell, a.bel.myZone,
        a.bel.zoneReached ? 1 : 0, (double)a.bel.x, (double)a.bel.y,
        a.bel.targetKnown ? 1 : 0, a.bel.targetOwner, a.bel.targetCell,
        a.bel.congregationClosed ? 1 : 0, a.bel.arrivedAtPlace ? 1 : 0,
        a.bel.meetIssued ? 1 : 0, (unsigned long)a.TalkingMask(now),
        (unsigned long)a.deliberations, a.map.CoverageCells(),
        EventName(a.lastEvent), (unsigned long)a.tLastEvent, in.retries,
        (unsigned long)sinProg, (unsigned long)a.rp.rep, a.rp.adopted ? 1 : 0,
        (unsigned long)a.rp.fallbacks);
    return (n < 0 || n >= cap) ? -1 : n;
}

inline int PutHex(char* p, int cap, const uint8_t* v, int n) {
    static const char* H = "0123456789abcdef";
    if (2 * n + 1 > cap) return -1;
    for (int i = 0; i < n; ++i) { p[2*i] = H[v[i] >> 4]; p[2*i+1] = H[v[i] & 15]; }
    p[2*n] = '\0';
    return 2 * n;
}

inline int EncodeMap(const Agent& a, uint32_t now, char* dst, int cap) {
    const GridConfig& g = a.grid;
    const int n = a.map.n;
    int k = std::snprintf(dst, (size_t)cap, "BMAP|id=%d|t=%lu|g=%d,%d,%.1f,%.2f,%.2f|kz=%d|z=",
                          a.selfId, (unsigned long)now, g.rows, g.cols, (double)g.cellMm,
                          (double)g.originXmm, (double)g.originYmm,
                          a.assign.valid ? a.assign.kz : 0);
    if (k < 0 || k >= cap) return -1;
    if (a.assign.valid)
        for (int i = 0; i < a.assign.kz; ++i) {
            const Zone& z = a.assign.zones[i];
            k += std::snprintf(dst + k, (size_t)(cap - k), "%s%d.%d.%d.%d",
                               i ? ";" : "", z.r0, z.r1, z.c0, z.c1);
            if (k >= cap) return -1;
        }
    k += std::snprintf(dst + k, (size_t)(cap - k), "|of=");
    bool primero = true;
    if (a.assign.valid)
        for (int i = 0; i < MAX_ROBOTS; ++i) {
            if (a.assign.zoneOf[i] < 0) continue;
            k += std::snprintf(dst + k, (size_t)(cap - k), "%s%d:%d", primero ? "" : ",",
                               i, a.assign.zoneOf[i]);
            primero = false;
            if (k >= cap) return -1;
        }
    struct Campo { const char* nom; const uint8_t* v; int len; };
    uint8_t vis[VISITED_WORDS * 4], blk[VISITED_WORDS * 4];
    const int nb = (n + 7) / 8;
    for (int i = 0; i < nb; ++i) { vis[i] = 0; blk[i] = 0; }
    for (int i = 0; i < n; ++i) {
        if (a.map.Visited(i)) vis[i >> 3] |= (uint8_t)(1u << (i & 7));
        if (a.map.Blocked(i)) blk[i >> 3] |= (uint8_t)(1u << (i & 7));
    }
    const Campo campos[] = { {"R", a.map.repel, n}, {"V", vis, nb}, {"B", blk, nb} };
    for (const Campo& c : campos) {
        k += std::snprintf(dst + k, (size_t)(cap - k), "|%s=", c.nom);
        if (k >= cap) return -1;
        const int h = PutHex(dst + k, cap - k, c.v, c.len);
        if (h < 0) return -1;
        k += h;
    }
    return k;
}

}  // namespace bdie
#endif  // BDIE_TELEMETRY_H
