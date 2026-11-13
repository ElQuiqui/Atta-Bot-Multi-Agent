/*****************************************************************************
 * pheromone_map.h — El ambiente estigmérgico (P2)
 *
 * Un campo escalar sobre la rejilla:
 *
 *   repel[]    se deposita donde pasa CUALQUIER robot y se evapora.
 *              Es lo que DISPERSA al enjambre durante la búsqueda.
 *
 * El reclutamiento al hallazgo NO usa feromona: lo hace el MEET del
 * laboratorio (ver bdi.h, P_MEET_LAB). El campo atractivo que había acá se
 * retiró el 28-sep-2026 para no duplicar lo que el laboratorio ya tiene.
 *
 * La estigmergia está en que el depósito se hace por todos los robots, no solo
 * por el propio: el rastro ajeno también me guía. Cada robot mantiene su propia
 * copia; no hay copia maestra. Las réplicas convergen porque todos reciben el
 * mismo broadcast de poses.
 *
 * LINAJE TEÓRICO — para el Capítulo 2, y no es una cita genérica de ACO:
 * la feromona repelente con evaporación tiene resultados formales de cobertura
 * en Wagner, Lindenbaum y Bruckstein, «Distributed Covering by Ant-Robots Using
 * Evaporating Traces», IEEE T-RA, 1999, y en la familia node counting / LRTA*.
 *
 * SIN DEPENDENCIAS DE ARDUINO.
 *****************************************************************************/
#ifndef BDIE_PHEROMONE_MAP_H
#define BDIE_PHEROMONE_MAP_H

#include "grid.h"
#include <cstring>
#include <cmath>

namespace bdie {

static const int VISITED_WORDS = (MAX_CELLS + 31) / 32;

/*===========================================================================*
 *  PRESUPUESTO DE MEMORIA — el número que cierra R9 para esta pieza
 *
 *  Punto de operación, 154 celdas de 190 mm — datos vivos:
 *      repel 154 B + visited 20 B + blocked 20 B = 194 B
 *
 *  Reserva estática real (arreglos al tope de MAX_CELLS = 320):
 *      320 + 40 + 40 = 400 B
 *
 *  Los dos números van al informe, no uno solo. La reserva es 2× el dato vivo
 *  A PROPÓSITO: es el margen de diseño para que el mapa pueda duplicar su
 *  resolución sin tocar la estructura, y el tope se movió junto con el punto de
 *  operación precisamente para que ese «2×» siga siendo cierto (ver la nota del
 *  tope en grid.h). Frente a los ~150–250 kB de heap libre estimado, 400 B sigue
 *  siendo ruido; la advertencia de R9 apunta al firmware existente, no a P2, y
 *  hay que poder decirlo con el getFreeHeap() medido en la mano (visita S5).
 *===========================================================================*/

struct PheromoneMap {
    uint8_t  repel  [MAX_CELLS];
    uint32_t visited[VISITED_WORDS];   // MÉTRICA acumulada — NO se evapora
    uint32_t blocked[VISITED_WORDS];   // celdas inhabilitadas (campaña con obstáculos)
    int      n = 0;

    /* Parámetros. Se barren en el banco de pruebas, no en el
     * laboratorio, y el barrido se reporta como figura del Cap. 5 en vez de
     * esconderse. Los valores de abajo son el PUNTO DE PARTIDA. */
    /* ⚠️ LEER LA NOTA DE CUANTIZACIÓN debajo de Evaporate() antes de barrer esto:
     * en el rango de valores en que opera el mapa, rho NO se comporta como la
     * ley geométrica que sugiere. El ritmo real lo fija el PERÍODO. */
    uint8_t  rho     = 8;      // evaporación, en 1/256 por aplicación
    /* 12-oct-2026: deposit 40 → 120 y beta 1,0 → 0,5, más el bono de novedad.
     * Medido en la corrida de laboratorio de las 16:01: en 24 de 39 decisiones
     * con una vecina NUNCA visitada disponible, el robot eligió una ya pisada.
     * Con deposit = 40 una celda pisada pesaba 0,80 contra 1,0, y el costo de
     * giro (0,10–1,0 con beta = 1) dominaba: seguir derecho sobre lo barrido le
     * ganaba a girar 90° hacia lo nuevo. Además la huella de 40 duraba ~136 s,
     * menos que una pasada por la franja. Con 120 dura ~5 min. */
    uint8_t  deposit = 120;    // incremento por visita
    float    alpha   = 1.5f;   // peso de la feromona en la ruleta
    float    beta    = 0.5f;   // peso del costo de giro en la ruleta
    /* Bono de novedad: multiplica la deseabilidad de una celda que YA FUE
     * VISITADA alguna vez (visited[], que no se evapora). La feromona dice
     * «hace poco pasó alguien»; visited[] dice «esto ya se miró». Para buscar un
     * objetivo estático lo segundo es lo que importa, y sin este término el
     * robot olvidaba lo barrido en cuanto se evaporaba la huella.
     * 1,0 = desactivado (comportamiento anterior). */
    float    novelty = 0.3f;

    // ── Ciclo de vida ─────────────────────────────────────────────────────
    void Reset(int cells) {
        n = (cells > MAX_CELLS) ? MAX_CELLS : cells;
        std::memset(repel,   0, sizeof(repel));
        std::memset(visited, 0, sizeof(visited));
        std::memset(blocked, 0, sizeof(blocked));
    }

    // ── Celdas inhabilitadas ──────────────────────────────────────────────
    void  Block  (int k)       { if (k >= 0 && k < n) blocked[k >> 5] |=  (1u << (k & 31)); }
    void  Unblock(int k)       { if (k >= 0 && k < n) blocked[k >> 5] &= ~(1u << (k & 31)); }
    bool  Blocked(int k) const { return (k >= 0 && k < n) && (blocked[k >> 5] >> (k & 31)) & 1u; }

    // ── Campo repelente ───────────────────────────────────────────────────

    /* Depósito. Se llama por CADA robot visto en el broadcast, no solo por el
     * propio: ahí está la estigmergia y ahí está la razón por la que las
     * réplicas convergen sin hablarse. */
    void Deposit(int k) {
        if (k < 0 || k >= n) return;
        repel[k] = (repel[k] > (uint8_t)(255 - deposit)) ? 255
                                                        : (uint8_t)(repel[k] + deposit);
        MarkVisited(k);
    }

    /* Evaporación. Sin ella el mapa se satura y el enjambre se paraliza: con
     * todas las celdas en 255 la ruleta se vuelve uniforme y el robot deja de
     * tener criterio para elegir.
     *
     * ─── NOTA DE CUANTIZACIÓN, y no es un detalle ────────────────────────────
     *
     * Esta cuenta es entera, y eso hace que la ley geométrica valga solo para
     * concentraciones altas. Para una celda en v, la evaporación deja
     *
     *     v' = ⌊v·(256−rho)/256⌋ = v − ⌈v·rho/256⌉
     *
     * y mientras v < 256/rho el término de arriba vale 1: **la celda baja de a
     * uno por aplicación, sea cual sea rho**. Con rho = 8 eso ocurre por debajo
     * de 32, y `deposit` valía 40: o sea que estábamos casi siempre en la zona
     * lineal, no en la geométrica.
     *
     * 14-oct-2026: desde el 12-oct `deposit` vale 120, por encima de 256/rho.
     * Un depósito fresco pasa ahora sus primeras aplicaciones en la zona
     * geométrica y llega a cero en 59 aplicaciones con rho = 8 (82 con 4, 102
     * con 2, 120 con 1): rho vuelve a mover algo. La tabla de abajo es la del
     * depósito viejo y se deja porque es la que justificó el período de 4 s.
     *
     * Medido sobre este mismo código, partiendo de un depósito (v = 40):
     *
     *     rho    vida media que dice la ley     vida media real     a cero
     *      8              21,8 aplic.               14 aplic.      34 aplic.
     *      4              44,0 aplic.               20 aplic.      40 aplic.
     *      2              88,4 aplic.               20 aplic.      40 aplic.
     *      1             177,1 aplic.               20 aplic.      40 aplic.
     *
     * De rho = 8 a rho = 1 —ocho veces más lento en la ley— la huella pasa de
     * durar 34 aplicaciones a durar 40. **rho no es la palanca que parece.**
     *
     * Dos consecuencias, las dos importan:
     *   1. Para cambiar el ritmo de evaporación se mueve `evaporatePeriodMs`
     *      (bdi.h), que sí lo divide exacto. Hoy está en 4 s.
     *   2. El barrido de parámetros del Cap. 5 no debe presentar rho como una
     *      palanca de la persistencia sin esta salvedad. Sí lo es si algún día
     *      se sube `deposit` por encima de 256/rho, o si la feromona pasa a
     *      punto fijo de 16 bits — que es la corrección de fondo, cuesta +320 B
     *      y está deliberadamente NO hecha: se decidió mover el período, que es
     *      una línea y riesgo cero. Queda anotado para cuando haga falta.
     *
     * Cubierto por T02 (propiedades 8 a 11), que fija este comportamiento por
     * escrito para que nadie lo «mejore» sin enterarse. */
    void Evaporate() {
        const uint16_t keep = (uint16_t)(255 - rho);
        for (int i = 0; i < n; ++i)
            repel[i] = (uint8_t)(((uint16_t)repel[i] * keep) >> 8);
    }

    // ── Cobertura acumulada: LA MÉTRICA, y vive aparte a propósito ─────────
    /*
     * Las dos métricas de cobertura son distintas y confundirlas cuesta una
     * vuelta entera. La feromona SE EVAPORA porque eso es lo que la hace útil
     * para decidir; medir la cobertura sobre ella da una curva que BAJA con el
     * tiempo, que es absurdo como métrica de exploración. La cobertura del
     * Capítulo 5 se lleva en visited[], que nunca se evapora.
     * Son dos estructuras con dos propósitos, separadas POR DISEÑO.
     */
    void MarkVisited(int k) { if (k >= 0 && k < n) visited[k >> 5] |= (1u << (k & 31)); }
    bool Visited(int k) const {
        return (k >= 0 && k < n) && (visited[k >> 5] >> (k & 31)) & 1u;
    }
    int CoverageCells() const {
        int c = 0;
        for (int i = 0; i < n; ++i) if (Visited(i)) ++c;
        return c;
    }
    float CoverageFraction() const {
        return (n > 0) ? (float)CoverageCells() / (float)n : 0.0f;
    }

    // ── Exploración: ruleta ACO sobre las ocho vecinas ────────────────────
    /*
     *   deseabilidad_k ∝ (1 / (1 + tau_k))^alpha  ·  eta_k^beta
     *
     *   tau_k = repel[k]/255   feromona normalizada  → huir de lo ya pisado
     *   eta_k = 1 - |Δrumbo|/180 · 0.9              → girar cuesta tiempo y batería
     *
     * Nótese el signo: es feromona REPELENTE, así que la deseabilidad es
     * INVERSA a la concentración. En un ACO clásico de rutas es al revés, y
     * copiar la fórmula sin invertirla produce un enjambre que se amontona en
     * la celda que más ha pisado. Es el error de una línea más caro de esta
     * pieza.
     *
     * mask: si no es nulo, solo se consideran celdas con mask[k] != 0 (mi zona).
     * Devuelve -1 si ninguna vecina es elegible; el llamador decide qué hacer
     * (en la práctica, ampliar a toda la arena: la zona se agotó).
     */
    int NextCell(const GridConfig& g, int from, float headingDeg,
                 const uint8_t* mask, uint32_t& rng) const {
        if (from < 0 || from >= n) return -1;
        int nb[8];
        const int m = Neighbors8(g, from, nb);
        float w[8];
        float sum = 0.0f;
        int   cand = 0;
        int   idx[8];

        for (int i = 0; i < m; ++i) {
            const int k = nb[i];
            if (Blocked(k)) continue;
            if (mask && !mask[k]) continue;
            const float tau = (float)repel[k] / 255.0f;
            const float dth = std::fabs(WrapDeg(HeadingDeg(g, from, k) - headingDeg));
            const float eta = 1.0f - 0.9f * (dth / 180.0f);        // ∈ [0.1, 1.0]
            float       wk  = std::pow(1.0f / (1.0f + tau), alpha) * std::pow(eta, beta);
            if (Visited(k)) wk *= novelty;                        // ya se miró
            idx[cand] = k;
            w[cand]   = wk;
            sum      += wk;
            ++cand;
        }
        if (cand == 0 || sum <= 0.0f) return -1;

        /* Ruleta. El sorteo es lo que dispersa al enjambre: con selección
         * greedy dos robots en celdas vecinas con mapas casi iguales toman
         * siempre la misma decisión y se pegan. */
        float r = RandUnit(rng) * sum;
        for (int i = 0; i < cand; ++i) {
            r -= w[i];
            if (r <= 0.0f) return idx[i];
        }
        return idx[cand - 1];
    }
    /* Frontera: la celda NUNCA visitada más cercana a `from` (distancia
     * euclídea en celdas, que prefiere avanzar recto). Respeta mask y las
     * bloqueadas. Empates por sorteo, para que dos robots no elijan la misma.
     * Devuelve -1 si no queda ninguna. Barre las n celdas: 160 comparaciones
     * enteras, nada frente al período de 100 ms del ciclo. */
    int NearestUnvisited(const GridConfig& g, int from, const uint8_t* mask,
                         uint32_t& rng) const {
        if (from < 0 || from >= n) return -1;
        const int fr = CellRow(g, from), fc = CellCol(g, from);
        int best = -1, bestD = 1 << 30, ties = 0;
        for (int k = 0; k < n; ++k) {
            if (Visited(k) || Blocked(k)) continue;
            if (mask && !mask[k]) continue;
            const int dr = CellRow(g, k) - fr, dc = CellCol(g, k) - fc;
            const int d = dr * dr + dc * dc;
            if (d < bestD) { bestD = d; best = k; ties = 1; }
            else if (d == bestD && (XorShift32(rng) % (uint32_t)(++ties)) == 0) best = k;
        }
        return best;
    }

    /* ¿Alguna vecina elegible (mask, no bloqueada) sin visitar todavía? */
    bool HasUnvisitedNeighbor(const GridConfig& g, int from, const uint8_t* mask) const {
        if (from < 0 || from >= n) return false;
        int nb[8];
        const int m = Neighbors8(g, from, nb);
        for (int i = 0; i < m; ++i) {
            const int k = nb[i];
            if (Blocked(k) || (mask && !mask[k])) continue;
            if (!Visited(k)) return true;
        }
        return false;
    }
};

}  // namespace bdie
#endif  // BDIE_PHEROMONE_MAP_H
