/*****************************************************************************
 * grid.h — Capa de cuadrícula (P4)
 *
 * Traduce entre el marco de la cámara (mm) y la cuadrícula de celdas sobre la
 * que razona el agente. Es la dependencia de todo lo demás y no depende de
 * nada: por eso se construye primero.
 *
 * MARCO DE LA CÁMARA — se respeta, no se corrige:
 *     X → derecha, Y → ABAJO, ángulo 0° = +X, positivo HORARIO.
 * Es un marco zurdo. Está documentado en Plataforma/DOCUMENTACION_CODIGO_
 * AttaBot.md §1 y hay que respetarlo: un signo invertido aquí produce un robot
 * que se aleja suavemente del objetivo, y eso cuesta horas de depuración
 * porque no se ve como un error, se ve como un robot lento.
 *
 * Rescatado de Archivo/Arquitectura_BDI-BS/Codigo_BDI-BS/firmware/grid.h: la
 * capa de cuadrícula es independiente de P2 y P3, así que sobrevivió intacta
 * al cambio de arquitectura. Lo que se agregó aquí es la enumeración de
 * vecindades, que la feromona necesita y el mapa bayesiano no necesitaba.
 *
 * SIN DEPENDENCIAS DE ARDUINO: compila en la PC para las pruebas de host y
 * para el banco SITL. Ver Codigo/LEEME.md.
 *****************************************************************************/
#ifndef BDIE_GRID_H
#define BDIE_GRID_H

#include <cmath>
#include <cstdint>

namespace bdie {

/* Tope de celdas.
 *
 * El punto de operación son 16×10 = 160 celdas de 150 mm sobre el área útil
 * medida de 2510,5×1587,5 mm (antes: 14×11 = 154 de 190 mm sobre 2500×2000
 * supuestos). El tope es 320, el doble, y es una decisión de diseño deliberada: que el mapa pueda duplicar
 * su resolución —hasta unos 135 mm de celda— sin tocar la estructura de datos ni recompilar nada más. La medición
 * de σ_ArUco en el laboratorio puede pedir una celda más fina, y el momento de
 * dejar sitio para eso es ahora, no cuando pase.
 *
 * El tope SUBIÓ junto con el punto de operación, y ésa es la regla que conviene
 * respetar: antes eran 80 celdas y un tope de 160 —el mismo factor 2—. Al bajar
 * la celda a 190 mm el punto de operación pasó a 154, y con el tope viejo el
 * margen habría quedado en SEIS celdas, o sea ninguno. Cuesta 360 B más de
 * reserva estática, ruido frente a los ~150–250 kB de heap libre estimado.
 *
 * El costo de esa decisión es explícito: los arreglos del mapa se dimensionan
 * al tope, no a la configuración. Ver pheromone_map.h §presupuesto. */
static const int MAX_CELLS  = 320;
static const int MAX_ROBOTS = 8;

struct GridConfig {
    /* Esquina de la celda (0,0), en el marco de la cámara.
     *
     * MEDIDO EN LA ARENA (paso 0 de la guía de puesta en marcha, 28-sep-2026):
     * el marcador ArUco de referencia está en la esquina SUPERIOR IZQUIERDA del
     * área útil, así que la base publica X ∈ [0, 2510,5] e Y ∈ [0, 1587,5] mm,
     * todo positivo, hacia la derecha y hacia abajo. La suposición anterior
     * —arena centrada en (0,0), origen (−1330, −1045)— dejaba a TODOS los
     * robots fuera de la cuadrícula: MmToIndex = −1 y la misión no arrancaba.
     *
     * 2510,5 y 1587,5 no son múltiplos de la celda. Se usa floor, no ceil:
     * 16 × 150 = 2400 y 10 × 150 = 1500, y la cuadrícula queda DENTRO de la
     * arena, centrada, con 55,25 mm libres a cada lado en X y 43,75 mm en Y. El
     * centro de celda más cercano a una pared queda a ≥ 118 mm de ella, más que
     * el radio del robot (75 mm). Las franjas que quedan fuera de la cuadrícula
     * no las puede ocupar el centro de un robot, y para el arranque existe
     * MmToIndexClamped.
     *
     * CELDA 150 mm (28-sep-2026, antes 190). σ_ArUco medido ≈ 8 mm por eje,
     * σr ≈ 11,6 mm en el peor tramo (base/herramientas/sigma_aruco.py, una
     * esquina: PRELIMINAR), contra los 30 mm supuestos. 150 mm es el piso
     * físico: el diámetro del robot. Por debajo, dos robots en celdas vecinas
     * se tocan. */
    /* 17-oct-2026 (tarde): celdas de 120 mm sobre el ÁREA LIBRE de hoy.
     *
     * Medido en el cuadro corregido de la cámara (marker 0 = 39,8 px → 2,51 mm/px):
     *   - a la derecha hay una fila de robots ESTACIONADOS que empieza en
     *     x ≈ 2 110–2 240 mm: un robot que iba a la columna de x = 2 305 quedó
     *     trabado contra ellos (Atta_3, corridas de las 12:51 y 13:03);
     *   - abajo la imagen termina en y ≈ 1 566 mm: el robot que iba a la fila de
     *     y = 1 469 salió de cuadro y quedó ciego (Atta_2, 12:51).
     * Área usada: x ∈ [10, 1 910], y ∈ [0, 1 400] → 19 × 14 = 266 celdas de
     * 100 mm. Últimos centros: x = 1 860, y = 1 350. 266 ≤ MAX_CELLS = 320, y el
     * BMAP (~800 B) sigue cabiendo en el datagrama de 1 024 B.
     * 19 y no 20 columnas (14:30): con 20, el Atta_3 llegó con el frente a la
     * fila de robots estacionados en x ≈ 1 937 (centro) — el borde libre real
     * está en x ≈ 2 000, no en los 2 110 que daba la medición por píxeles.
     *
     * POR QUÉ 100 mm Y NO 120 (misma tarde): con 120 mm la corrida de las 13:40
     * llegó al 93 % de cobertura sin tocar el cubo: la celda del cubo figuraba
     * VISITADA porque un centro de robot la cruzó a ~150 mm del cubo. El sensor
     * de color ve a centímetros y solo de frente, así que pisar una celda no es
     * mirarla. Ahora, además, el robot gira 360° al llegar a cada celda en
     * BUSCAR (bdie.ino, «barrido»), y con celdas de 100 mm todo punto del área
     * queda a ≤ 71 mm de algún centro de celda: dentro del alcance del giro.
     *
     * 100 mm es MENOR que el robot (150 mm): dos robots no caben en celdas
     * vecinas, pero la celda acá es la unidad de exploración, no de ocupación;
     * los choques los resuelve la capa reactiva (IR) y el reparto por franjas.
     * El umbral de llegada sigue saturado en 50 mm (RecommendedArrivalMm).
     *
     * ── ÁREA COMPLETA, para cuando se retiren los robots estacionados ──────
     * Arena útil medida el 28-sep: 2 510,5 × 1 587,5 mm, pero la cámara ve hasta
     * y ≈ 1 560. Con celdas de 100 mm (¡subir MAX_CELLS a ≥ 336 y el buffer del
     * BMAP en bdie.ino a ≥ 1 200 B!):
     *     originXmm = 55.0f;  originYmm = 0.0f;  rows = 14;  cols = 24;   // 336 celdas
     *     (últimos centros x = 2 405, y = 1 350)
     * Anterior (13:16 del 17-oct): 17 × 12 de 120 mm, origen (10, 0).
     * Anterior (mañana del 17-oct): 15 × 9 de 150 mm, origen (130, 43,75).
     * Anterior (28-sep): 16 × 10 de 150 mm, origen (55,25, 43,75). */
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

/* Centro de la celda, en mm. El robot navega a CENTROS de celda, nunca a
 * bordes: apuntar a un borde con σ_ArUco = 30 mm es pedirle al robot que se
 * quede oscilando entre dos celdas. */
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

/* Índice de celda SATURADO al borde del área útil.
 *
 * Esto no es una comodidad: es un requisito de acuerdo entre réplicas. Un
 * robot que arranca medio cuerpo fuera de la arena tiene que producir el mismo
 * índice de celda en las cinco réplicas, y devolver -1 en unas y un índice en
 * otras rompería la asignación de zonas (assignment.h). */
inline int MmToIndexClamped(const GridConfig& g, float x, float y) {
    int ci = (int)std::floor((x - g.originXmm) / g.cellMm);
    int ri = (int)std::floor((y - g.originYmm) / g.cellMm);
    if (ci < 0)        ci = 0;
    if (ci >= g.cols)  ci = g.cols - 1;
    if (ri < 0)        ri = 0;
    if (ri >= g.rows)  ri = g.rows - 1;
    return CellIndex(g, ri, ci);
}

/* Distancia Manhattan EN CELDAS: el robot se mueve por la cuadrícula, así que
 * describe el costo real mejor que la euclídea. Es la que entra en el costo de
 * la asignación de zonas. */
inline int CellDistance(const GridConfig& g, int k1, int k2) {
    const int dr = CellRow(g, k1) - CellRow(g, k2);
    const int dc = CellCol(g, k1) - CellCol(g, k2);
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

/* Distancia Chebyshev EN CELDAS: la que corresponde a la vecindad de 8, que es
 * sobre la que se propaga el gradiente atractivo. Con ella se verifica que el
 * gradiente no tiene mínimos locales (prueba T04). */
inline int CellDistance8(const GridConfig& g, int k1, int k2) {
    int dr = CellRow(g, k1) - CellRow(g, k2);
    int dc = CellCol(g, k1) - CellCol(g, k2);
    if (dr < 0) dr = -dr;
    if (dc < 0) dc = -dc;
    return (dr > dc) ? dr : dc;
}

/* Vecindad de 8, en orden fijo. El orden es parte del contrato: la ruleta de
 * la feromona y la relajación del gradiente lo recorren igual en los cinco
 * robots, y un orden distinto daría campos ligeramente distintos. */
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

/* Rumbo, en grados, del centro de k1 al centro de k2, EN EL MARCO DE LA
 * CÁMARA (Y hacia abajo, positivo horario). Se usa para penalizar el giro en
 * la ruleta de exploración. */
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

/* Umbral de llegada recomendado para movimiento celda a celda.
 *
 * ReactiveNav trae 50 mm de fábrica, pensado para navegación libre. Con celdas
 * de 190 mm eso es exigirle el 26 % de una celda a un sistema con σ = 30 mm, y
 * el robot rebota alrededor del punto. No hace falta tocar código para
 * arreglarlo: el comando NAV_CONFIG|ARRIVAL_THRESHOLD|<mm> ya existe en el
 * firmware (punto de contacto ninguno; es configuración).
 *
 * ⚠️ (28-sep) Con celda de 150 mm, 0,32 × 150 = 48 → se satura en 50 mm, que
 * son ~4 σr con el σ_ArUco medido (≈ 11,6 mm). Historia: a 190 mm el umbral salía 61 mm, que son ~2,0 σ de ArUco — antes, con celdas
 * de 250 mm, eran 80 mm y ~2,7 σ. Es el precio de la celda más fina y hay que
 * mirarlo en el banco, no suponerlo: un umbral apretado se manifiesta como
 * llegadas que no se registran, no como un error. */
inline float RecommendedArrivalMm(const GridConfig& g) {
    float t = g.cellMm * 0.32f;
    if (t <  50.0f) t =  50.0f;
    if (t > 200.0f) t = 200.0f;   // rango que acepta el comando del firmware
    return t;
}

/* Generador congruencial xorshift32.
 *
 * La exploración es deliberadamente estocástica —la ruleta ACO es lo que
 * dispersa al enjambre— pero las pruebas y el banco tienen que ser
 * reproducibles. Cada agente lleva su propia semilla, derivada de su ID, así
 * que dos robots no sortean igual y una corrida se puede repetir exacta. */
inline uint32_t XorShift32(uint32_t& s) {
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
inline float RandUnit(uint32_t& s) {
    return (float)(XorShift32(s) >> 8) / 16777216.0f;   // [0,1)
}

}  // namespace bdie
#endif  // BDIE_GRID_H
