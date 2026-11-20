/*****************************************************************************
 * pheromone_map.h — Mapa de feromona repelente de BDI-E
 *
 * Campo escalar sobre la cuadrícula: cada robot deposita feromona en la celda
 * por la que pasa cualquier robot del enjambre, y la feromona se evapora con el
 * tiempo. La deseabilidad de una celda es inversa a su concentración, lo que
 * dispersa al enjambre durante la búsqueda (estigmergia: el rastro ajeno
 * también guía al robot).
 *
 * Cada robot mantiene su propia copia; no hay copia maestra. Las réplicas
 * tienden a coincidir porque todas reciben la misma difusión de posiciones,
 * pero solo de forma eventual.
 *
 * El reclutamiento al hallazgo no usa feromona: lo hace la congregación
 * MEET del firmware del laboratorio (ver P_MEET_LAB en bdi.h).
 *
 * Fundamento: la feromona repelente con evaporación tiene resultados formales
 * de cobertura (Wagner, Lindenbaum y Bruckstein, «Distributed Covering by
 * Ant-Robots Using Evaporating Traces», IEEE T-RA, 1999).
 *
 * Sin dependencias de Arduino.
 *****************************************************************************/
#ifndef BDIE_PHEROMONE_MAP_H
#define BDIE_PHEROMONE_MAP_H

#include "grid.h"
#include <cstring>
#include <cmath>

namespace bdie {

static const int VISITED_WORDS = (MAX_CELLS + 31) / 32;

/*===========================================================================*
 *  Memoria. Los arreglos se dimensionan a MAX_CELLS = 320:
 *      repel 320 B + visited 40 B + blocked 40 B = 400 B de reserva estática.
 *  Con la cuadrícula de operación (266 celdas) los datos vivos son unos
 *  340 B. La reserva deja margen para duplicar la resolución sin cambiar la
 *  estructura (ver MAX_CELLS en grid.h).
 *===========================================================================*/

struct PheromoneMap {
    uint8_t  repel  [MAX_CELLS];
    uint32_t visited[VISITED_WORDS];   // cobertura acumulada: no se evapora
    uint32_t blocked[VISITED_WORDS];   // celdas inhabilitadas (obstáculos conocidos)
    int      n = 0;

    /* Parámetros del mapa. Se ajustan en el banco de simulación. Antes de cambiar
     * rho conviene leer la nota de cuantización de Evaporate(): en el rango en que
     * opera el mapa, el ritmo real de evaporación lo fija el período de
     * aplicación (evaporatePeriodMs en bdi.h) más que rho. */
    uint8_t  rho     = 8;      // evaporación, en 1/256 por aplicación
    /* Con 120 por visita, una celda pisada pesa bastante menos que una nueva y
     * la huella dura unos 5 min, más que una pasada del robot por su zona. Con un
     * depósito menor el costo de giro dominaba la ruleta y el robot seguía derecho
     * sobre lo ya barrido. */
    uint8_t  deposit = 120;    // incremento por visita
    float    alpha   = 1.5f;   // peso de la feromona en la ruleta
    float    beta    = 0.5f;   // peso del costo de giro en la ruleta
    /* Bono de novedad: multiplica la deseabilidad de una celda que ya fue
     * visitada alguna vez (visited[], que no se evapora). La feromona dice «hace
     * poco pasó alguien»; visited[] dice «esto ya se miró», que es lo que importa
     * para buscar un objetivo estático. 1,0 lo desactiva. */
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

    /* Depósito. Se llama por cada robot visto en la difusión de posiciones, no
     * solo por el propio: ahí está la estigmergia, y por eso las réplicas
     * convergen sin intercambiar el mapa. */
    void Deposit(int k) {
        if (k < 0 || k >= n) return;
        repel[k] = (repel[k] > (uint8_t)(255 - deposit)) ? 255
                                                        : (uint8_t)(repel[k] + deposit);
        MarkVisited(k);
    }

    /* Evaporación. Sin ella el mapa se satura, la ruleta se vuelve uniforme y el
     * robot pierde criterio para elegir.
     *
     * Nota de cuantización. La cuenta es entera, así que la ley geométrica solo
     * vale para concentraciones altas. Para una celda en v,
     *
     *     v' = ⌊v·(256−rho)/256⌋ = v − ⌈v·rho/256⌉
     *
     * y mientras v < 256/rho la celda baja de a uno por aplicación, sea cual sea
     * rho. Con deposit = 120 y rho = 8, un depósito fresco pasa sus primeras
     * aplicaciones en la zona geométrica y llega a cero en 59 aplicaciones (82 con
     * rho = 4, 102 con 2, 120 con 1). Para depósitos pequeños rho casi no influye:
     * partiendo de v = 40, la huella dura 34 aplicaciones con rho = 8 y 40 con
     * rho = 1.
     *
     * Por eso, para cambiar el ritmo de evaporación se mueve evaporatePeriodMs
     * (bdi.h), que lo divide exacto. La corrección de fondo sería una feromona en
     * punto fijo de 16 bits (+320 B). La prueba T02 fija este comportamiento. */
    void Evaporate() {
        const uint16_t keep = (uint16_t)(255 - rho);
        for (int i = 0; i < n; ++i)
            repel[i] = (uint8_t)(((uint16_t)repel[i] * keep) >> 8);
    }

    // ── Cobertura acumulada: LA MÉTRICA, y vive aparte a propósito ─────────
    /* La cobertura se lleva aparte, en visited[], que nunca se evapora. Medirla
     * sobre la feromona daría una curva que baja con el tiempo. Son dos
     * estructuras con dos propósitos distintos. */
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
    /* Elige la siguiente celda con una ruleta sobre las ocho vecinas:
     *
     *   deseabilidad_k ∝ (1 / (1 + tau_k))^alpha · eta_k^beta · novedad_k
     *
     *   tau_k = repel[k]/255          feromona normalizada: huir de lo pisado
     *   eta_k = 1 − 0,9·|Δrumbo|/180   girar cuesta tiempo y batería
     *
     * La deseabilidad es inversa a la concentración porque la feromona es
     * repelente; en un ACO de rutas es al revés, y sin invertirla el enjambre se
     * amontona en las celdas más pisadas.
     *
     * mask: si no es nulo, solo se consideran celdas con mask[k] != 0 (la zona del
     * robot). Devuelve -1 si ninguna vecina es elegible; el llamador decide qué
     * hacer. */
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

        /* Ruleta: el sorteo dispersa al enjambre. Con selección determinista, dos
         * robots vecinos con mapas casi iguales toman la misma decisión y se pegan. */
        float r = RandUnit(rng) * sum;
        for (int i = 0; i < cand; ++i) {
            r -= w[i];
            if (r <= 0.0f) return idx[i];
        }
        return idx[cand - 1];
    }
    /* Frontera: la celda nunca visitada más cercana a `from`, por distancia
     * euclídea en celdas (prefiere avanzar recto). Respeta mask y las celdas
     * bloqueadas, y desempata por sorteo para que dos robots no elijan la misma.
     * Devuelve -1 si no queda ninguna. */
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
