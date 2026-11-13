/*****************************************************************************
 * assignment.h — Repartir la arena entre agentes que se hablan (P3)
 *
 * 🔁 14-oct-2026 — EL REPARTO SE ACUERDA POR MENSAJES (§REPARTO, al final).
 * Decisión del autor, con la aceptación del asesor: en un sistema multiagente
 * los agentes se hablan. Cada robot RECLAMA la franja libre más cercana; si
 * dos reclaman la misma se la queda el más cercano —a igual distancia, el de
 * menor ID— y el otro reclama la siguiente; cuando todos los reclamos de la
 * ronda están oídos y no se repiten, cada uno lo da por ACORDADO y lo reporta.
 * Una caída o una reconexión abre una ronda nueva.
 *
 * Y corrige un defecto de la versión anterior: «vivo» era «lo ve la cámara».
 * Un robot apagado que sigue en la arena con su ArUco visible no se daba por
 * caído nunca, su franja no se repartía y el quórum lo esperaba. Ahora vivo es
 * «lo oigo» (bdi.h, Agent::Talking).
 *
 * Lo de abajo hasta §REPARTO es la versión del 08-sep (conocimiento común,
 * cero mensajes). NO se borró: ComputeAssignment es el ORÁCULO de las pruebas
 * —el reparto por mensajes converge a exactamente el mismo resultado— y el
 * RESPALDO si una ronda no se acuerda al arrancar. Su texto se conserva tal
 * cual porque documenta el argumento que se evaluó.
 *
 * ── Versión del 08-sep-2026 ────────────────────────────────────────────────
 *
 * Ésta es la pieza donde vive el argumento de descentralización del PFG, así
 * que conviene decir la idea completa antes del código:
 *
 *     Todos los robots reciben el MISMO broadcast de poses. Comparten, por lo
 *     tanto, CONOCIMIENTO COMÚN. Y cualquier función determinista sobre
 *     conocimiento común da el mismo resultado en todos los agentes. Entonces
 *     repartir la arena no requiere negociar: requiere que los cinco corran la
 *     misma función.
 *
 * Lo que eso elimina, comparado con el Contract Net que estaba avalado en la
 * S4: el rol de subastador —que es un rol privilegiado dentro de un trabajo
 * titulado «descentralizada»—, los CFP/BID/AWARD sobre un canal sin acuse, el
 * roundTimeoutMs que depende de una latencia todavía no medida, la
 * adjudicación duplicada y el robot que llega tarde y no tiene cómo
 * reincorporarse. No se reparan esos modos de fallo: se elimina el mecanismo
 * que los tiene.
 *
 * Y la tolerancia a fallos sale gratis: si un robot deja de aparecer en el
 * broadcast, TODOS lo dan por caído en el mismo epoch y recalculan sobre el
 * conjunto reducido, llegando otra vez al mismo resultado. No hace falta
 * detectar el fallo, ni anunciarlo, ni elegir un reemplazo. Es lo que hace que
 * el experimento de fallo inducido de OE4 funcione sin mecanismo dedicado.
 *
 * SIN DEPENDENCIAS DE ARDUINO. Es, además, la pieza con la propiedad más
 * fuerte de toda la arquitectura y la que más conviene poder verificar fuera
 * del laboratorio (prueba T05).
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
    uint32_t epoch = 0;        // epoch del último POSES en que apareció
    bool     seen  = false;    // ¿apareció alguna vez?
};

struct Roster {
    Pose     p[MAX_ROBOTS];
    int      n     = 0;        // robots nominales (IDs 0..n-1)
    uint32_t epoch = 0;        // último epoch recibido
    /* Vejez, EN EPOCHS DE BROADCAST, no en milisegundos locales.
     *
     * Esto no es un detalle de implementación: es lo que hace exacta la
     * propiedad de acuerdo. Si la vejez se midiera con el millis() de cada
     * ESP32, dos robots con relojes corridos declararían muerto al vecino en
     * instantes distintos y recalcularían zonas distintas durante esa ventana.
     * Contando epochs del mismo broadcast, los cinco cruzan el umbral en el
     * MISMO paquete. Con POSES a 10 Hz, 30 epochs ≈ 3 s. */
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

    /* Máscara de vivos. Es la ENTRADA que congela la asignación: mientras no
     * cambie, no se recalcula. Recalcular en cada ciclo con poses que se mueven
     * produce robots intercambiándose zonas sin fin — defecto encontrado en
     * simulación, no una optimización teórica. */
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

/* Partición regular del área útil en kz franjas verticales.
 *
 * Franjas verticales y no un mosaico porque la arena es 2500×2000 y con cinco
 * robots cada franja queda en 2 columnas × 8 filas = 16 celdas: un rectángulo
 * que se barre de arriba a abajo sin cruzar la zona del vecino. Un mosaico
 * daría zonas más cuadradas pero con más frontera compartida.
 *
 * El resto de la división entera se reparte a las franjas de la IZQUIERDA, y
 * eso es determinista: los cinco robots calculan la misma partición. */
inline int Partition(const GridConfig& g, int k, Zone z[MAX_ROBOTS]) {
    int kz = k;
    if (kz < 1) kz = 1;
    if (kz > g.cols) kz = g.cols;        // la misión fija 5 robots y 10 columnas
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
    uint32_t epoch  = 0;                  // epoch de anclaje
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

/* Celda de anclaje de un robot.
 *
 * La asignación NO se calcula sobre coordenadas en mm, sino sobre ÍNDICES DE
 * CELDA, y esa cuantización es deliberada. Un paquete POSES perdido hace que
 * una réplica ancle sobre el siguiente paquete, ~100 ms después, en el que el
 * robot se movió unos 2 cm: en mm los costos difieren y el greedy puede
 * invertir un desempate; en celdas de 250 mm el índice es el mismo salvo que el
 * robot esté justo sobre una frontera.
 *
 * Lo que se puede afirmar, y es lo que va al informe: el acuerdo es EXACTO
 * dadas la misma máscara de vivos y las mismas celdas ancla; la cuantización
 * reduce la ventana de desacuerdo a un robot cruzando una frontera de celda en
 * el mismo epoch en que otro robot es declarado muerto. Lo que NO se puede
 * afirmar es acuerdo exacto bajo pérdida arbitraria, y no se afirma. */
inline int AnchorCell(const GridConfig& g, const Pose& p) {
    return MmToIndexClamped(g, p.x, p.y);
}

/* Asignación greedy por costo, con desempate por ID.
 *
 * El desempate no es un detalle: sin él, dos réplicas podrían diferir por ruido
 * numérico y romper el acuerdo, que es justamente la propiedad por la que se
 * eligió este mecanismo. Con él la función es PURA —sin estado, sin mensajes,
 * sin timeouts— y el acuerdo se verifica fuera del laboratorio.
 *
 * Orden de recorrido fijo (robots ascendentes, zonas ascendentes) y comparación
 * ESTRICTA: ante empate gana el primero encontrado, que es el de menor ID y,
 * dentro de él, la zona de menor índice. */
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

    /* Más robots que zonas (solo si k > cols, que la misión no produce):
     * los sobrantes comparten zona por su posición en el orden. Determinista. */
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
 *  Desempate de hallazgos simultáneos
 *
 *  Si dos robots encuentran el objetivo casi a la vez, gana el descubridor de
 *  MENOR ID, criterio que todos aplican igual sobre el mismo dato. Sin esto el
 *  enjambre se parte en dos grupos congregándose en puntos distintos, que es
 *  un modo de fallo espectacular y perfectamente evitable con una comparación.
 *---------------------------------------------------------------------------*/
inline bool TargetClaimWins(int newOwner, int curOwner) {
    if (curOwner < 0) return true;
    return newOwner < curOwner;
}

/*===========================================================================*
 *  §REPARTO — el reparto de franjas acordado por mensajes (14-oct-2026)
 *
 *  LAS REGLAS, en el orden en que se explican en el informe:
 *
 *   1. VIVO = LO OIGO. Lo decide bdi.h (Talking): un mensaje propio del par en
 *      los últimos peerDeadMs. La cámara dice DÓNDE está cada uno, ya no SI está.
 *   2. PARTICIPANTES: los vivos que están buscando (DESPLEGAR o BUSCAR).
 *   3. RONDA. Cada vez que un robot ve cambiar el conjunto de participantes
 *      abre una ronda nueva, con número = mayor visto + 1. Quien recibe un
 *      reclamo de una ronda mayor se pasa a ella; uno de una ronda vieja se
 *      ignora, así un paquete atrasado no reabre un conflicto ya resuelto.
 *   4. ANCLA CONGELADA. Al entrar a la ronda el robot fija su celda; todos sus
 *      reclamos de esa ronda miden desde ahí. Medir desde la celda actual
 *      reintroduce el defecto que el banco ya mostró: robots intercambiándose
 *      franjas sin fin mientras se mueven.
 *   5. RECLAMO POR CERCANÍA: con k participantes, k franjas (Partition); cada
 *      uno reclama la franja LIBRE más cercana a su ancla.
 *   6. CONFLICTO: se la queda el más cercano; a igual distancia, el menor ID.
 *      El que pierde reclama la siguiente libre más cercana.
 *   7. ACUERDO: cuando oí el reclamo de TODOS los participantes en mi misma
 *      ronda y con mi mismo conjunto, ninguna franja se repite y nada cambió
 *      durante settleMs, adopto mi franja y lo reporto (ZA).
 *   8. NADIE BLOQUEA A NADIE: mientras la ronda no se acuerda, cada robot sigue
 *      con la franja que tenía. Si no se acuerda nunca, el enjambre queda con
 *      el reparto anterior y la frontera cubre el hueco: degrada a cobertura
 *      redundante, no a parálisis.
 *
 *  POR QUÉ CONVERGE Y A QUÉ: la clave (distancia, ID) tiene un único punto
 *  fijo, y es el voraz de ComputeAssignment —mismo orden: distancia, ID e
 *  índice de franja—. El par robot–franja más cercano de todo el enjambre no
 *  pierde nunca, así que queda fijo en la primera vuelta; por inducción cada
 *  vuelta fija al menos uno más: a lo sumo k vueltas. Lo verifica
 *  pruebas_host/prueba_reparto.cpp contra ComputeAssignment como oráculo.
 *
 *  Lo que NO se afirma: acuerdo instantáneo bajo pérdida arbitraria. Mientras
 *  la ronda se cierra hay una ventana de desacuerdo, y se MIDE en el banco.
 *
 *  Todo lo de esta sección es puro: el estado de la ronda y los reclamos oídos
 *  entran por argumento. El Agent (bdi.h) es quien escucha, decide cuándo
 *  abrir una ronda y manda los mensajes.
 *===========================================================================*/

inline int PopCount(uint32_t m) { int n = 0; while (m) { m &= m - 1u; ++n; } return n; }

/* La clave de prioridad sobre una franja: gana la distancia menor y, a igual
 * distancia, el ID menor. Es la regla del autor («si tienen una igual, por ID
 * se reparte otra») con la cercanía delante, para que el ID solo desempate. */
inline bool ClaimBeats(int distA, int idA, int distB, int idB) {
    return distA < distB || (distA == distB && idA < idB);
}

/* Lo último que se oyó de un par: su reclamo, con la ronda y el conjunto de
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

/* ¿Algún participante de MI ronda reclama esta franja con mejor clave que yo? */
inline bool FranjaTomada(const Reparto& rp, int self, int zone, int myDist) {
    for (int j = 0; j < MAX_ROBOTS; ++j) {
        if (j == self || !(rp.mask & (1u << j))) continue;
        const ZoneClaim& c = rp.peer[j];
        if (!c.valid || c.rep != rp.rep || c.mask != rp.mask || c.zone != zone) continue;
        if (ClaimBeats(c.dist, j, myDist, self)) return true;
    }
    return false;
}

/* La franja que me toca reclamar: la LIBRE más cercana a mi ancla; a igual
 * distancia, la de menor índice (el mismo orden que ComputeAssignment). Si
 * todas parecen tomadas —solo con reclamos viejos, mientras la ronda se
 * asienta— reclamo la más cercana y la regla de conflicto lo resuelve. */
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

/* ¿Puedo dar la ronda por acordada? Oí a TODOS los participantes en mi ronda y
 * con mi conjunto, y ninguna franja se repite. (El «nada cambió en settleMs»
 * lo pone el Agent, que tiene el reloj.) */
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

/* El reparto acordado como Assignment: la misma estructura que usaban la
 * telemetría (BMAP of=) y los planes. */
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

/* La ranura de cada robot en la congregación NO se calcula acá: la calcula el
 * MEET del laboratorio (navegacion.ino MeetSlotIndex), que aplica la misma idea
 * —función determinista sobre la misma lista de vecinos— sin negociar. */

}  // namespace bdie
#endif  // BDIE_ASSIGNMENT_H
