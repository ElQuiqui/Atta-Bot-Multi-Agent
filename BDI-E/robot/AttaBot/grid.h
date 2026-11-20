/*****************************************************************************
 * grid.h — Capa de cuadrícula de BDI-E
 *
 * Convierte entre el marco de la cámara (mm) y la cuadrícula de celdas sobre
 * la que decide el agente, enumera vecindades y fija el umbral de llegada.
 * No depende de ningún otro módulo de BDI-E.
 *
 * Marco de la cámara: X hacia la derecha, Y hacia abajo, ángulo 0° sobre +X y
 * positivo en sentido horario. Es un marco zurdo; todas las funciones de este
 * archivo lo respetan, y un signo invertido aquí hace que el robot se aleje
 * del objetivo sin que se note como error.
 *
 * Sin dependencias de Arduino: compila igual en el robot, en las pruebas de
 * la PC y en el banco SITL.
 *****************************************************************************/
#ifndef BDIE_GRID_H
#define BDIE_GRID_H

#include <cmath>
#include <cstdint>

namespace bdie {

/* Tope de celdas y de robots. Los arreglos del mapa se dimensionan al tope y
 * no a la configuración, de modo que la cuadrícula puede afinarse (hasta el
 * doble de celdas) sin cambiar ninguna estructura de datos. El costo es
 * memoria estática reservada, del orden de cientos de bytes; ver
 * pheromone_map.h. */
static const int MAX_CELLS  = 320;
static const int MAX_ROBOTS = 8;

struct GridConfig {
    /* Esquina de la celda (0,0) en el marco de la cámara. El marcador ArUco de
     * referencia está en la esquina superior izquierda de la arena, así que
     * todas las coordenadas son positivas.
     *
     * Cuadrícula de operación: 19 × 14 = 266 celdas de 100 mm sobre el área
     * libre x ∈ [10, 1910] mm, y ∈ [0, 1400] mm (últimos centros en x = 1860 y
     * y = 1350). Cabe en MAX_CELLS y el mapa de bits de cobertura (~800 B)
     * cabe en un datagrama de 1024 B.
     *
     * La celda es la unidad de exploración, no de ocupación: el sensor de
     * color ve solo de frente y a pocos centímetros, y con celdas de 100 mm
     * todo punto del área queda a menos de 71 mm de algún centro, dentro del
     * alcance del barrido que el robot hace al llegar a cada celda. Los choques
     * entre robots los resuelven la capa reactiva y el reparto de zonas.
     *
     * Para usar el área completa de la arena (24 columnas, 336 celdas) hay que
     * subir MAX_CELLS a 336 o más y el búfer del mapa de bits en bdie.ino a
     * 1200 B o más:
     *     originXmm = 55.0f; originYmm = 0.0f; rows = 14; cols = 24; */
    float originXmm =    10.0f;
    float originYmm =     0.0f;
    float cellMm    =   100.0f;
    int   rows      =    14;
    int   cols      =    19;

    int  Cells() const { return rows * cols; }
    bool Valid() const {
        return rows > 0 && cols > 0 && Cells() <= MAX_CELLS && cellMm > 1.0f;
    }
};

inline int CellIndex(const GridConfig& g, int r, int c) { return r * g.cols + c; }
inline int CellRow  (const GridConfig& g, int k)        { return k / g.cols; }
inline int CellCol  (const GridConfig& g, int k)        { return k % g.cols; }

inline bool CellValid(const GridConfig& g, int k) { return k >= 0 && k < g.Cells(); }

/* Centro de la celda (r, c) en mm. El robot navega siempre a centros de
 * celda: apuntar a un borde lo deja oscilando entre dos celdas. */
inline void CellToMm(const GridConfig& g, int r, int c, float& x, float& y) {
    x = g.originXmm + ((float)c + 0.5f) * g.cellMm;
    y = g.originYmm + ((float)r + 0.5f) * g.cellMm;
}

inline void IndexToMm(const GridConfig& g, int k, float& x, float& y) {
    CellToMm(g, CellRow(g, k), CellCol(g, k), x, y);
}

inline bool MmToCell(const GridConfig& g, float x, float y, int& r, int& c) {
    const int ci = (int)std::floor((x - g.originXmm) / g.cellMm);
    const int ri = (int)std::floor((y - g.originYmm) / g.cellMm);
    if (ri < 0 || ci < 0 || ri >= g.rows || ci >= g.cols) return false;
    r = ri; c = ci;
    return true;
}

/* Índice de celda, o -1 si la pose cae fuera del área útil. */
inline int MmToIndex(const GridConfig& g, float x, float y) {
    int r, c;
    if (!MmToCell(g, x, y, r, c)) return -1;
    return CellIndex(g, r, c);
}

/* Índice de celda saturado al borde del área útil. Un robot que arranca
 * parcialmente fuera de la cuadrícula obtiene el mismo índice en todas las
 * réplicas, lo que el reparto de zonas (assignment.h) necesita. */
inline int MmToIndexClamped(const GridConfig& g, float x, float y) {
    int ci = (int)std::floor((x - g.originXmm) / g.cellMm);
    int ri = (int)std::floor((y - g.originYmm) / g.cellMm);
    if (ci < 0)        ci = 0;
    if (ci >= g.cols)  ci = g.cols - 1;
    if (ri < 0)        ri = 0;
    if (ri >= g.rows)  ri = g.rows - 1;
    return CellIndex(g, ri, ci);
}

/* Distancia Manhattan en celdas: el robot se mueve por la cuadrícula, así que
 * describe su recorrido mejor que la euclídea. Entra en el costo del reparto
 * de zonas. */
inline int CellDistance(const GridConfig& g, int k1, int k2) {
    const int dr = CellRow(g, k1) - CellRow(g, k2);
    const int dc = CellCol(g, k1) - CellCol(g, k2);
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

/* Distancia Chebyshev en celdas, la que corresponde a la vecindad de 8. */
inline int CellDistance8(const GridConfig& g, int k1, int k2) {
    int dr = CellRow(g, k1) - CellRow(g, k2);
    int dc = CellCol(g, k1) - CellCol(g, k2);
    if (dr < 0) dr = -dr;
    if (dc < 0) dc = -dc;
    return (dr > dc) ? dr : dc;
}

/* Vecindad de 8 en orden fijo. El orden es parte del contrato: la ruleta de
 * exploración lo recorre igual en todos los robots. Devuelve cuántas vecinas
 * quedan dentro de la cuadrícula. */
inline int Neighbors8(const GridConfig& g, int k, int out[8]) {
    static const int DR[8] = { -1, -1, -1,  0,  0,  1,  1,  1 };
    static const int DC[8] = { -1,  0,  1, -1,  1, -1,  0,  1 };
    const int r = CellRow(g, k), c = CellCol(g, k);
    int n = 0;
    for (int i = 0; i < 8; ++i) {
        const int rr = r + DR[i], cc = c + DC[i];
        if (rr < 0 || cc < 0 || rr >= g.rows || cc >= g.cols) continue;
        out[n++] = CellIndex(g, rr, cc);
    }
    return n;
}

/* Rumbo, en grados, del centro de k1 al centro de k2 en el marco de la
 * cámara. Se usa para penalizar los giros en la ruleta de exploración. */
inline float HeadingDeg(const GridConfig& g, int k1, int k2) {
    float x1, y1, x2, y2;
    IndexToMm(g, k1, x1, y1);
    IndexToMm(g, k2, x2, y2);
    return (float)(std::atan2((double)(y2 - y1), (double)(x2 - x1)) * 180.0 / 3.14159265358979);
}

/* Diferencia angular con signo, llevada a (-180, 180]. */
inline float WrapDeg(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

/* Umbral de llegada recomendado para moverse de celda en celda: el 32 % de
 * la celda, saturado entre 50 y 200 mm (el rango que acepta el comando
 * NAV_CONFIG|ARRIVAL_THRESHOLD del firmware). Con celdas de 100 mm queda en
 * 50 mm, unas cuatro veces la desviación estándar de la posición por
 * marcadores. Un umbral demasiado apretado se manifiesta como llegadas que no
 * se registran. */
inline float RecommendedArrivalMm(const GridConfig& g) {
    float t = g.cellMm * 0.32f;
    if (t <  50.0f) t =  50.0f;
    if (t > 200.0f) t = 200.0f;   // rango que acepta el comando del firmware
    return t;
}

/* Generador pseudoaleatorio xorshift32. La exploración es estocástica (la
 * ruleta dispersa al enjambre), pero las pruebas y el banco deben ser
 * reproducibles: cada agente lleva su propia semilla, derivada de su ID. */
inline uint32_t XorShift32(uint32_t& s) {
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
inline float RandUnit(uint32_t& s) {
    return (float)(XorShift32(s) >> 8) / 16777216.0f;   // [0,1)
}

}  // namespace bdie
#endif  // BDIE_GRID_H
