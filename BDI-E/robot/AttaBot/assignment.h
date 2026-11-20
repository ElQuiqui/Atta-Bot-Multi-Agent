/*****************************************************************************
 * assignment.h — Reparto de la arena entre los agentes de BDI-E
 *
 * La arena se divide en franjas verticales, una por robot que participa en la
 * búsqueda, y los robots acuerdan por mensajes qué franja toma cada uno
 * (sección «Reparto por mensajes», al final): cada robot reclama la franja
 * libre más cercana; si dos reclaman la misma, se la queda el más cercano y, a
 * igual distancia, el de menor ID, y el otro reclama la siguiente. Cuando
 * todos los reclamos de la ronda se han oído y ninguno se repite, cada robot
 * da su franja por acordada y lo reporta. Una caída o una reconexión abre una
 * ronda nueva. No hay subastador ni adjudicación.
 *
 * Un robot está vivo si se le oye (Agent::Talking en bdi.h), no si la cámara lo
 * ve: un robot apagado que sigue en la arena con su marcador visible se da por
 * caído y su franja se reparte.
 *
 * ComputeAssignment calcula el mismo reparto por conocimiento común, sin
 * mensajes: todos los robots reciben la misma difusión de posiciones, y una
 * función determinista sobre datos comunes da el mismo resultado en todos. Se
 * usa como oráculo de las pruebas (el reparto por mensajes converge a su mismo
 * resultado) y como respaldo si una ronda no se acuerda al arrancar.
 *
 * Sin dependencias de Arduino. Pruebas: pruebas_host/prueba_reparto.cpp.
 *****************************************************************************/
#ifndef BDIE_ASSIGNMENT_H
#define BDIE_ASSIGNMENT_H

#include "grid.h"

namespace bdie {

/*---------------------------------------------------------------------------*
 *  Pose difundida por la base
 *---------------------------------------------------------------------------*/
struct Pose {
    float    x = 0.0f, y = 0.0f, th = 0.0f;
    uint32_t epoch = 0;        // época del último POSES en que apareció
    bool     seen  = false;    // ¿apareció alguna vez?
};

struct Roster {
    Pose     p[MAX_ROBOTS];
    int      n     = 0;        // robots nominales (IDs 0..n-1)
    uint32_t epoch = 0;        // última época recibida
    /* Vejez medida en épocas de la difusión de posiciones, no en milisegundos
     * locales. Así todos los robots cruzan el umbral con el mismo paquete aunque
     * sus relojes difieran, y no hay ventana en que calculen repartos distintos.
     * Con POSES a 10 Hz, 30 épocas son unos 3 s. */
    uint32_t staleEpochs = 30;

    void OnPose(int id, float x, float y, float th, uint32_t ep) {
        if (id < 0 || id >= MAX_ROBOTS) return;
        p[id].x = x; p[id].y = y; p[id].th = th;
        p[id].epoch = ep; p[id].seen = true;
        if (ep > epoch) epoch = ep;
        if (id >= n) n = id + 1;
    }

    bool Alive(int id) const {
        if (id < 0 || id >= n || !p[id].seen) return false;
        return (epoch - p[id].epoch) <= staleEpochs;
    }

    /* Máscara de vivos. Es la entrada que congela la asignación: mientras no
     * cambie, no se recalcula. Recalcular en cada ciclo con robots que se mueven
     * los haría intercambiarse zonas sin fin. */
    uint32_t AliveMask() const {
        uint32_t m = 0;
        for (int i = 0; i < n && i < MAX_ROBOTS; ++i) if (Alive(i)) m |= (1u << i);
        return m;
    }

    int AliveSorted(int out[MAX_ROBOTS]) const {
        int k = 0;
        for (int i = 0; i < n && i < MAX_ROBOTS; ++i) if (Alive(i)) out[k++] = i;
        return k;                                  // ya vienen en orden ascendente
    }
    int AliveCount() const { int t[MAX_ROBOTS]; return AliveSorted(t); }
};

/*---------------------------------------------------------------------------*
 *  Zona: una franja rectangular de celdas, extremos incluidos
 *---------------------------------------------------------------------------*/
struct Zone {
    int r0 = 0, r1 = -1, c0 = 0, c1 = -1;
    bool Empty() const { return r1 < r0 || c1 < c0; }
    bool Has(const GridConfig& g, int k) const {
        if (!CellValid(g, k)) return false;
        const int r = CellRow(g, k), c = CellCol(g, k);
        return r >= r0 && r <= r1 && c >= c0 && c <= c1;
    }
    int Cells() const { return Empty() ? 0 : (r1 - r0 + 1) * (c1 - c0 + 1); }
};

/* Partición regular del área útil en kz franjas verticales. Una franja se
 * barre de arriba abajo sin cruzar la zona del vecino; un mosaico daría zonas
 * más cuadradas pero con más frontera compartida. El resto de la división
 * entera se reparte a las franjas de la izquierda, de forma determinista. */
inline int Partition(const GridConfig& g, int k, Zone z[MAX_ROBOTS]) {
    int kz = k;
    if (kz < 1) kz = 1;
    if (kz > g.cols) kz = g.cols;        // no puede haber más franjas que columnas
    if (kz > MAX_ROBOTS) kz = MAX_ROBOTS;
    const int base = g.cols / kz;
    const int rest = g.cols % kz;
    int c = 0;
    for (int i = 0; i < kz; ++i) {
        const int w = base + (i < rest ? 1 : 0);
        z[i].c0 = c;
        z[i].c1 = c + w - 1;
        z[i].r0 = 0;
        z[i].r1 = g.rows - 1;
        c += w;
    }
    return kz;
}

inline int ZoneCenterCell(const GridConfig& g, const Zone& z) {
    if (z.Empty()) return -1;
    return CellIndex(g, (z.r0 + z.r1) / 2, (z.c0 + z.c1) / 2);
}

inline void ZoneMask(const GridConfig& g, const Zone& z, uint8_t out[MAX_CELLS]) {
    for (int k = 0; k < g.Cells(); ++k) out[k] = z.Has(g, k) ? 1 : 0;
}

/*---------------------------------------------------------------------------*
 *  La asignación
 *---------------------------------------------------------------------------*/
struct Assignment {
    uint32_t mask   = 0;                  // máscara de vivos que la produjo
    uint32_t epoch  = 0;                  // época de anclaje
    int      k      = 0;                  // robots vivos
    int      kz     = 0;                  // zonas
    int      zoneOf[MAX_ROBOTS];          // zona por ID de robot, -1 si no vive
    Zone     zones [MAX_ROBOTS];
    bool     valid  = false;

    Assignment() { for (int i = 0; i < MAX_ROBOTS; ++i) zoneOf[i] = -1; }

    int MyZone(int selfId) const {
        return (valid && selfId >= 0 && selfId < MAX_ROBOTS) ? zoneOf[selfId] : -1;
    }
};

/* Celda de anclaje de un robot. La asignación se calcula sobre índices de
 * celda y no sobre milímetros: si una réplica pierde un paquete de posiciones
 * y ancla sobre el siguiente, el robot casi siempre sigue en la misma celda y
 * el resultado no cambia. El acuerdo es exacto dadas la misma máscara de vivos
 * y las mismas celdas ancla; solo puede diferir si un robot cruza una frontera
 * de celda en la misma época en que otro se declara caído. */
inline int AnchorCell(const GridConfig& g, const Pose& p) {
    return MmToIndexClamped(g, p.x, p.y);
}

/* Asignación voraz por costo (distancia en celdas), con desempate por ID.
 * El recorrido tiene orden fijo (robots ascendentes, zonas ascendentes) y la
 * comparación es estricta: ante empate gana el de menor ID y, dentro de él, la
 * zona de menor índice. Así la función es pura, sin estado ni mensajes, y da
 * el mismo resultado en todas las réplicas. */
inline Assignment ComputeAssignment(const GridConfig& g, const Roster& r) {
    Assignment a;
    int ids[MAX_ROBOTS];
    const int k = r.AliveSorted(ids);
    a.mask  = r.AliveMask();
    a.epoch = r.epoch;
    a.k     = k;
    if (k == 0) return a;

    a.kz = Partition(g, k, a.zones);

    int  cell[MAX_ROBOTS];
    for (int i = 0; i < k; ++i) cell[i] = AnchorCell(g, r.p[ids[i]]);

    bool robotDone[MAX_ROBOTS] = { false };
    bool zoneDone [MAX_ROBOTS] = { false };

    const int pairs = (k < a.kz) ? k : a.kz;
    for (int step = 0; step < pairs; ++step) {
        int bestR = -1, bestZ = -1, bestC = 0;
        for (int i = 0; i < k; ++i) {
            if (robotDone[i]) continue;
            for (int z = 0; z < a.kz; ++z) {
                if (zoneDone[z]) continue;
                const int c = CellDistance(g, cell[i], ZoneCenterCell(g, a.zones[z]));
                if (bestR < 0 || c < bestC) { bestR = i; bestZ = z; bestC = c; }
            }
        }
        robotDone[bestR] = true;
        zoneDone [bestZ] = true;
        a.zoneOf[ids[bestR]] = bestZ;
    }

    /* Si hubiera más robots que zonas (k > cols), los sobrantes comparten zona
     * según su posición en el orden. */
    for (int i = 0; i < k; ++i)
        if (!robotDone[i]) a.zoneOf[ids[i]] = i % a.kz;

    a.valid = true;
    return a;
}

/* Congelado: recalcula solo si cambió el conjunto de agentes vivos. */
inline bool MaybeReassign(const GridConfig& g, const Roster& r, Assignment& a) {
    const uint32_t m = r.AliveMask();
    if (a.valid && a.mask == m) return false;
    a = ComputeAssignment(g, r);
    return a.valid;
}

/*---------------------------------------------------------------------------*
 *  Desempate de hallazgos simultáneos: si dos robots encuentran el objetivo
 *  casi a la vez, gana el descubridor de menor ID. Todos aplican el mismo
 *  criterio sobre el mismo dato, y el enjambre no se parte en dos grupos.
 *---------------------------------------------------------------------------*/
inline bool TargetClaimWins(int newOwner, int curOwner) {
    if (curOwner < 0) return true;
    return newOwner < curOwner;
}

/*===========================================================================*
 *  Reparto por mensajes
 *
 *   1. Vivo = se le oye: un mensaje propio del robot en los últimos peerDeadMs
 *      (Agent::Talking en bdi.h). La cámara solo dice dónde está cada uno.
 *   2. Participantes: los vivos que están buscando (DESPLEGAR o BUSCAR).
 *   3. Ronda: cada vez que un robot ve cambiar el conjunto de participantes
 *      abre una ronda nueva, numerada como la mayor vista + 1. Quien recibe un
 *      reclamo de una ronda mayor se pasa a ella; los de rondas viejas se
 *      ignoran, para que un paquete atrasado no reabra un conflicto resuelto.
 *   4. Ancla congelada: al entrar a la ronda el robot fija su celda, y todos
 *      sus reclamos de esa ronda se miden desde ahí. Medir desde la celda
 *      actual haría que los robots se intercambiaran franjas mientras se
 *      mueven.
 *   5. Reclamo por cercanía: con k participantes hay k franjas (Partition), y
 *      cada uno reclama la franja libre más cercana a su ancla.
 *   6. Conflicto: se la queda el más cercano; a igual distancia, el menor ID.
 *      El que pierde reclama la siguiente libre más cercana.
 *   7. Acuerdo: cuando el robot oyó el reclamo de todos los participantes en
 *      su misma ronda y con su mismo conjunto, ninguna franja se repite y nada
 *      cambió durante settleMs, adopta su franja y lo reporta (ZA).
 *   8. Nadie bloquea a nadie: mientras la ronda no se acuerda, cada robot sigue
 *      con la franja que tenía. Si no se acuerda nunca, el enjambre conserva el
 *      reparto anterior y la búsqueda de frontera cubre el hueco.
 *
 *  Convergencia: la clave (distancia, ID) tiene un único punto fijo, que es el
 *  resultado voraz de ComputeAssignment con el mismo orden. El par
 *  robot–franja más cercano de todo el enjambre nunca pierde y queda fijo en
 *  la primera vuelta; por inducción, cada vuelta fija al menos uno más, así
 *  que bastan k vueltas. Mientras la ronda se cierra hay una ventana de
 *  desacuerdo, que se mide en el banco.
 *
 *  Las funciones de esta sección son puras: el estado de la ronda y los
 *  reclamos oídos entran por argumento. El Agent (bdi.h) escucha, decide
 *  cuándo abrir una ronda y envía los mensajes.
 *===========================================================================*/

inline int PopCount(uint32_t m) { int n = 0; while (m) { m &= m - 1u; ++n; } return n; }

/* Prioridad sobre una franja: gana la distancia menor y, a igual distancia,
 * el ID menor. */
inline bool ClaimBeats(int distA, int idA, int distB, int idB) {
    return distA < distB || (distA == distB && idA < idB);
}

/* Último reclamo oído de otro robot, con la ronda y el conjunto de
 * participantes con que lo hizo. */
struct ZoneClaim {
    uint32_t rep  = 0;           // ronda
    uint32_t mask = 0;           // participantes según ese par (bit i = robot i)
    int      zone = -1;          // franja que reclama
    int      dist = 0;           // celdas desde su ancla al centro de la franja
    bool     valid = false;
};

struct Reparto {
    // ── Parámetros ────────────────────────────────────────────────────────
    uint32_t settleMs     = 1000;   // nada cambió en este lapso → se puede acordar
    uint32_t resendMs     = 500;    // reenvío del reclamo mientras no hay acuerdo
    uint32_t keepMs       = 2000;   // reenvío del reporte (ZA) ya acordado
    uint32_t startGraceMs = 1500;   // al iniciar la sesión, esperar a oír a todos
    uint32_t fallbackMs   = 10000;  // sin acuerdo a los grace+fallback → respaldo
    uint32_t tickMs       = 50;     // ritmo del reparto dentro de Comunicar()

    // ── La ronda vigente ──────────────────────────────────────────────────
    uint32_t rep     = 0;           // 0 = todavía no hubo ronda
    uint32_t mask    = 0;           // participantes según yo
    uint32_t seenMax = 0;           // la ronda más alta que oí
    int      anchor  = -1;          // mi celda al entrar a la ronda (congelada)
    int      zone    = -1;          // lo que reclamo
    int      dist    = 0;
    bool     adopted = false;       // ¿la di por acordada?
    bool     sendNow = false;
    uint32_t tChange = 0, tSent = 0, tAdopt = 0, tTick = 0;

    ZoneClaim peer[MAX_ROBOTS];

    // ── Instrumentación: solo se cuentan, ninguna decisión las lee ────────
    uint32_t rounds = 0, sent = 0, agreements = 0, fallbacks = 0;

    /* Vuelve al estado inicial SIN tocar los parámetros, que el banco puede
     * haber cambiado. */
    void ResetState() {
        const uint32_t s = settleMs, r = resendMs, k = keepMs, g = startGraceMs,
                       f = fallbackMs, t = tickMs;
        *this = Reparto();
        settleMs = s; resendMs = r; keepMs = k; startGraceMs = g;
        fallbackMs = f; tickMs = t;
    }
};

/* ¿Algún participante de la ronda propia reclama esta franja con mejor clave? */
inline bool FranjaTomada(const Reparto& rp, int self, int zone, int myDist) {
    for (int j = 0; j < MAX_ROBOTS; ++j) {
        if (j == self || !(rp.mask & (1u << j))) continue;
        const ZoneClaim& c = rp.peer[j];
        if (!c.valid || c.rep != rp.rep || c.mask != rp.mask || c.zone != zone) continue;
        if (ClaimBeats(c.dist, j, myDist, self)) return true;
    }
    return false;
}

/* Franja que le toca reclamar al robot: la libre más cercana a su ancla y, a
 * igual distancia, la de menor índice (el mismo orden que ComputeAssignment).
 * Si todas parecen tomadas, lo que solo pasa con reclamos viejos mientras la
 * ronda se asienta, reclama la más cercana y la regla de conflicto decide. */
inline bool ElegirFranja(const GridConfig& g, const Reparto& rp, int self,
                         int& zoneOut, int& distOut) {
    const int k = PopCount(rp.mask);
    if (k <= 0 || rp.anchor < 0) return false;
    Zone z[MAX_ROBOTS];
    const int kz = Partition(g, k, z);
    int best = -1, bestD = 0, any = -1, anyD = 0;
    for (int i = 0; i < kz; ++i) {
        const int d = CellDistance(g, rp.anchor, ZoneCenterCell(g, z[i]));
        if (any < 0 || d < anyD) { any = i; anyD = d; }
        if (FranjaTomada(rp, self, i, d)) continue;
        if (best < 0 || d < bestD) { best = i; bestD = d; }
    }
    if (best < 0) { best = any; bestD = anyD; }
    zoneOut = best;
    distOut = bestD;
    return best >= 0;
}

/* ¿Se puede dar la ronda por acordada? Hay que haber oído a todos los
 * participantes en la ronda propia y con el mismo conjunto, sin franjas
 * repetidas. La condición de «nada cambió en settleMs» la aplica el Agent,
 * que tiene el reloj. */
inline bool RondaCompleta(const Reparto& rp, int self) {
    if (rp.rep == 0 || rp.zone < 0 || !(rp.mask & (1u << self))) return false;
    uint32_t usadas = 1u << rp.zone;
    for (int j = 0; j < MAX_ROBOTS; ++j) {
        if (j == self || !(rp.mask & (1u << j))) continue;
        const ZoneClaim& c = rp.peer[j];
        if (!c.valid || c.rep != rp.rep || c.mask != rp.mask || c.zone < 0) return false;
        if (usadas & (1u << c.zone)) return false;
        usadas |= 1u << c.zone;
    }
    return true;
}

/* El reparto acordado como Assignment, la estructura que usan la telemetría
 * (BMAP of=) y los planes. */
inline Assignment AssignmentFromReparto(const GridConfig& g, const Reparto& rp, int self) {
    Assignment a;
    a.mask  = rp.mask;
    a.epoch = rp.rep;
    a.k     = PopCount(rp.mask);
    if (a.k == 0) return a;
    a.kz = Partition(g, a.k, a.zones);
    for (int j = 0; j < MAX_ROBOTS; ++j) {
        if (!(rp.mask & (1u << j))) continue;
        a.zoneOf[j] = (j == self) ? rp.zone : rp.peer[j].zone;
    }
    a.valid = true;
    return a;
}

/* La ranura de cada robot en la congregación no se calcula aquí: la calcula
 * el MEET del laboratorio (MeetSlotIndex en navegacion.ino), con la misma idea
 * de una función determinista sobre la misma lista de vecinos. */

}  // namespace bdie
#endif  // BDIE_ASSIGNMENT_H
