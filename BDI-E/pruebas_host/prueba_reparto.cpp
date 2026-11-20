/*****************************************************************************
 * prueba_reparto.cpp — Reparto de franjas por mensajes, contra el oráculo
 *
 * Corre N agentes reales (bdi.h + mission.h, el mismo C++ del ESP32) sobre un
 * bus en memoria con pérdida inyectada: cada mensaje que un agente difunde
 * llega a cada uno de los demás con probabilidad 1 − p. No hay física: los
 * robots están quietos en posiciones sorteadas, el caso en que el resultado
 * debe ser exacto.
 *
 *   R1  sin pérdida, 1000 configuraciones de 1 a 5 robots: el reparto acordado
 *       es idéntico al de ComputeAssignment (el oráculo)
 *   R2  con 5, 15 y 30 % de pérdida: al acordar, sin franjas repetidas e
 *       idéntico al oráculo; rondas y tiempo hasta el acuerdo
 *   R3  un robot se apaga: los demás dejan de oírlo y reparten entre k − 1
 *   R4  un robot queda en silencio 10 s (BDIE_SILENCIO): se reparte de nuevo
 *       al caer y otra vez al volver
 *   R5  un robot se reinicia y se reincorpora (SESSION|START por unicast = JOIN)
 *   R6  todos los mensajes del reparto caben en el analizador de comandos
 *
 *   make -f Makefile.mk reparto
 *****************************************************************************/
#include "../firmware/mission.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <string>

using namespace bdie;

static int fallas = 0;
static void check(const char* nombre, bool ok, const char* detalle = "") {
    std::printf("  %s %s%s%s\n", ok ? "ok  " : "FALLA", nombre, ok ? "" : "  <- ", ok ? "" : detalle);
    if (!ok) ++fallas;
}

static uint32_t rng = 12345u;
static double U() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (rng & 0xFFFFFF) / 16777216.0; }

struct Robot {
    Agent a;
    float x = 0, y = 0;
    bool vivo = true, mudo = false;
};

struct Mundo {
    GridConfig g;
    std::vector<Robot> r;
    double perdida = 0.0;
    uint32_t t = 0;
    uint32_t epoch = 1;
    long mensajes = 0;

    void Poses() {
        char buf[256];
        int n = std::snprintf(buf, sizeof(buf), "POSES|%lu|", (unsigned long)epoch++);
        for (size_t i = 0; i < r.size(); ++i)   // la cámara ve a todos, vivos o no
            n += std::snprintf(buf + n, sizeof(buf) - n, "%d:%.0f:%.0f:0;", (int)i, r[i].x, r[i].y);
        for (auto& q : r) if (q.vivo && !q.mudo) Feed(q.a, buf, t);
    }
    void Paso() {
        t += 50;
        if (t % 100 == 0) Poses();
        for (size_t i = 0; i < r.size(); ++i) {
            if (!r[i].vivo) continue;
            r[i].a.Comunicar(t);
            Wire w;
            while (Emit(r[i].a, w)) {
                if (w.len == 0) continue;
                if (w.buf[0] == 'Z') ++mensajes;
                if (r[i].mudo) continue;
                for (size_t j = 0; j < r.size(); ++j) {
                    if (j == i || !r[j].vivo || r[j].mudo) continue;
                    if (U() < perdida) continue;
                    Feed(r[j].a, w.buf, t);
                }
            }
        }
    }
    void Arrancar(int k) {
        r.clear(); r.resize(k);
        for (int i = 0; i < k; ++i) {
            r[i].a.Init(i, g);
            /* En mm enteros: POSES viaja con %.0f, y el oráculo debe ver exactamente lo
             * que vio el agente (un redondeo cerca de una frontera de celda cambia la
             * celda ancla). */
            r[i].x = (float)(int)(g.originXmm + 20 + U() * (g.cols * g.cellMm - 40));
            r[i].y = (float)(int)(g.originYmm + 20 + U() * (g.rows * g.cellMm - 40));
        }
        t = 1000;
        Poses();
        char s[64]; std::snprintf(s, sizeof(s), "SESSION|START|1|%.0f|%.0f", 100.0, 100.0);
        for (auto& q : r) Feed(q.a, s, t);
    }
    /* ¿Todos los vivos que hablan dieron por acordada la misma ronda, con el
     * conjunto exacto de los que hablan, y con franjas distintas? */
    bool Acordado() const {
        uint32_t esperado = 0;
        for (size_t i = 0; i < r.size(); ++i) if (r[i].vivo && !r[i].mudo) esperado |= 1u << i;
        uint32_t rep = 0; uint32_t usadas = 0;
        for (size_t i = 0; i < r.size(); ++i) {
            if (!r[i].vivo || r[i].mudo) continue;
            const Agent& a = r[i].a;
            if (!a.rp.adopted || a.rp.mask != esperado) return false;
            if (rep == 0) rep = a.rp.rep; else if (a.rp.rep != rep) return false;
            if (a.bel.myZone < 0 || (usadas & (1u << a.bel.myZone))) return false;
            usadas |= 1u << a.bel.myZone;
        }
        return rep != 0;
    }
    /* El oráculo: ComputeAssignment sobre los que hablan, con sus celdas. */
    bool IgualAlOraculo() const {
        Roster ro;
        for (size_t i = 0; i < r.size(); ++i) ro.n = (int)r.size();
        for (size_t i = 0; i < r.size(); ++i)
            if (r[i].vivo && !r[i].mudo) ro.OnPose((int)i, r[i].x, r[i].y, 0, 100);
            else { ro.p[i].seen = false; }
        ro.epoch = 100;
        const Assignment o = ComputeAssignment(g, ro);
        for (size_t i = 0; i < r.size(); ++i) {
            if (!r[i].vivo || r[i].mudo) continue;
            if (r[i].a.bel.myZone != o.zoneOf[i]) return false;
            if (r[i].a.assign.kz != o.kz) return false;
        }
        return true;
    }
    void Feed_Join(int j) {    // BASE.SESSION|JOIN|j: el mismo START, solo a él
        char s[64]; std::snprintf(s, sizeof(s), "SESSION|START|1|%.0f|%.0f", 100.0, 100.0);
        Feed(r[j].a, s, t);
    }
    bool Duplicado() const {     // dos vivos con la misma franja de la misma partición
        for (size_t i = 0; i < r.size(); ++i) for (size_t j = i + 1; j < r.size(); ++j) {
            if (!r[i].vivo || !r[j].vivo || r[i].mudo || r[j].mudo) continue;
            const Agent &a = r[i].a, &b = r[j].a;
            if (a.bel.myZone >= 0 && a.bel.myZone == b.bel.myZone && a.assign.kz == b.assign.kz)
                return true;
        }
        return false;
    }
    /* Corre hasta el acuerdo o hasta `tope` ms; devuelve los ms que tardó (-1 si no). */
    long HastaAcuerdo(uint32_t desde, uint32_t tope, long* dupTicks = 0, long* ticks = 0) {
        while (t - desde < tope) {
            Paso();
            if (ticks) ++*ticks;
            if (dupTicks && Duplicado()) ++*dupTicks;
            if (Acordado()) return (long)(t - desde);
        }
        return -1;
    }
};

static double Mediana(std::vector<long> v) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
static long P90(std::vector<long> v) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[(size_t)(0.9 * (v.size() - 1))];
}

int main() {
    Mundo M;   // GridConfig por omisión: la cuadrícula de operación (grid.h)

    std::printf("R1 · sin pérdida: el reparto acordado es el del oráculo\n");
    {
        int iguales = 0, acordadas = 0, n = 1000;
        std::vector<long> tA; std::vector<long> rondas;
        for (int c = 0; c < n; ++c) {
            M.perdida = 0.0;
            M.Arrancar(1 + c % 5);
            const long dt = M.HastaAcuerdo(M.t, 20000);
            if (dt >= 0) { ++acordadas; tA.push_back(dt); if (M.IgualAlOraculo()) ++iguales; }
            uint32_t mx = 0; for (auto& q : M.r) mx = std::max(mx, q.a.rp.rounds);
            rondas.push_back(mx);
        }
        char d[160];
        std::snprintf(d, sizeof(d), "%d de %d acordaron, %d iguales al oráculo", acordadas, n, iguales);
        check("1 000 configuraciones de 1-5 robots: todas acuerdan", acordadas == n, d);
        check("y el reparto es IDÉNTICO a ComputeAssignment", iguales == n, d);
        std::printf("       tiempo desde el inicio de la sesión: mediana %.0f ms, p90 %ld ms;"
                    " rondas mediana %.0f\n", Mediana(tA), P90(tA), Mediana(rondas));
    }

    std::printf("R2 · con pérdida, 5 robots, 300 configuraciones por nivel\n");
    for (double p : {0.05, 0.15, 0.30}) {
        int iguales = 0, acordadas = 0, n = 300; long dup = 0, ticks = 0, msgs = 0;
        std::vector<long> tA;
        for (int c = 0; c < n; ++c) {
            M.perdida = p; M.mensajes = 0;
            M.Arrancar(5);
            const long dt = M.HastaAcuerdo(M.t, 30000, &dup, &ticks);
            if (dt >= 0) { ++acordadas; tA.push_back(dt); if (M.IgualAlOraculo()) ++iguales; }
            msgs += M.mensajes;
        }
        char nom[96], d[160];
        std::snprintf(nom, sizeof(nom), "pérdida %2.0f %%: acuerdan e iguales al oráculo", 100 * p);
        std::snprintf(d, sizeof(d), "%d acordaron, %d iguales", acordadas, iguales);
        check(nom, acordadas == n && iguales == n, d);
        std::printf("       tiempo al acuerdo: mediana %.0f ms, p90 %ld ms · mensajes ZC+ZA hasta"
                    " acordar: %.1f por robot · ticks con franja repetida: %.2f %%\n",
                    Mediana(tA), P90(tA), msgs / (5.0 * n), ticks ? 100.0 * dup / ticks : 0.0);
    }

    std::printf("R3 · un robot se apaga (la cámara lo sigue viendo)\n");
    {
        int ok = 0, n = 200; std::vector<long> tR;
        for (int c = 0; c < n; ++c) {
            M.perdida = 0.05;
            M.Arrancar(5);
            M.HastaAcuerdo(M.t, 30000);
            M.r[c % 5].vivo = false;
            const uint32_t t0 = M.t;
            const long dt = M.HastaAcuerdo(t0, 30000);
            if (dt >= 0 && M.IgualAlOraculo()) { ++ok; tR.push_back(dt); }
        }
        char d[96]; std::snprintf(d, sizeof(d), "%d de %d", ok, n);
        check("los 4 restantes re-reparten entre 4 = oráculo sobre los 4", ok == n, d);
        std::printf("       tiempo desde el apagado: mediana %.0f ms, p90 %ld ms"
                    " (incluye los 3 s hasta darlo por muerto)\n", Mediana(tR), P90(tR));
    }

    std::printf("R4 · un robot en silencio 10 s y de vuelta\n");
    {
        int okCae = 0, okSolo = 0, okVuelve = 0, n = 200; std::vector<long> tV;
        for (int c = 0; c < n; ++c) {
            M.perdida = 0.05;
            M.Arrancar(5);
            M.HastaAcuerdo(M.t, 30000);
            const int j = c % 5;
            M.r[j].mudo = true;
            const uint32_t t0 = M.t;
            if (M.HastaAcuerdo(t0, 9000) >= 0 && M.IgualAlOraculo()) ++okCae;
            while (M.t - t0 < 10000) M.Paso();
            /* el silenciado, solo, se quedó con toda el área (k = 1) */
            const Agent& s = M.r[j].a;
            if (s.rp.adopted && s.rp.mask == (1u << j) && s.bel.myZone == 0 && s.assign.kz == 1) ++okSolo;
            M.r[j].mudo = false;
            const uint32_t t1 = M.t;
            const long dt = M.HastaAcuerdo(t1, 30000);
            if (dt >= 0 && M.IgualAlOraculo()) { ++okVuelve; tV.push_back(dt); }
        }
        char d[96];
        std::snprintf(d, sizeof(d), "%d de %d", okCae, n);   check("al callarse, los otros 4 re-reparten", okCae == n, d);
        std::snprintf(d, sizeof(d), "%d de %d", okSolo, n);  check("el callado se queda solo con toda el área", okSolo == n, d);
        std::snprintf(d, sizeof(d), "%d de %d", okVuelve, n); check("al volver, re-reparto entre 5 = oráculo", okVuelve == n, d);
        std::printf("       tiempo desde que vuelve: mediana %.0f ms, p90 %ld ms\n", Mediana(tV), P90(tV));
    }

    std::printf("R5 · un robot se reinicia y se reincorpora con SESSION|START unicast (JOIN)\n");
    {
        int ok = 0, n = 200; std::vector<long> tJ;
        for (int c = 0; c < n; ++c) {
            M.perdida = 0.05;
            M.Arrancar(5);
            M.HastaAcuerdo(M.t, 30000);
            const int j = c % 5;
            M.r[j].vivo = false;
            const uint32_t t0 = M.t;
            M.HastaAcuerdo(t0, 30000);
            M.r[j].a.Init(j, M.g);                 // reinicio: estado de fábrica
            M.r[j].vivo = true;
            M.Poses();
            M.Feed_Join(j);
            const uint32_t t1 = M.t;
            const long dt = M.HastaAcuerdo(t1, 30000);
            if (dt >= 0 && M.IgualAlOraculo()) { ++ok; tJ.push_back(dt); }
        }
        char d[96]; std::snprintf(d, sizeof(d), "%d de %d", ok, n);
        check("el reincorporado entra al reparto y todos re-reparten entre 5", ok == n, d);
        std::printf("       tiempo desde el JOIN: mediana %.0f ms, p90 %ld ms\n", Mediana(tJ), P90(tJ));
    }

    std::printf("R6 · todos los mensajes del reparto caben en el analizador\n");
    {
        Wire w; EncodeZone(w, false, 4294967295u, 0xffu, 7, 7, 99);
        char d[96]; std::snprintf(d, sizeof(d), "%d campos, %d B: %s", w.fields, w.len, w.buf);
        check("ZC en el peor caso: 5 campos", w.fields == 5 && w.len < 64, d);
        Decoded dd; Decode(w.buf, dd);
        check("y se decodifica igual", dd.kind == W_ZC && dd.epoch == 4294967295u && dd.mask == 0xffu &&
              dd.id == 7 && dd.zone == 7 && dd.dist == 99);
        EncodeZone(w, true, 12, 0x1f, 3, 2, 4);
        Decode(w.buf, dd);
        check("ZA|12:1f|3|2|4", dd.kind == W_ZA && dd.epoch == 12 && dd.mask == 0x1f && dd.zone == 2, w.buf);
    }

    std::printf(fallas ? "FALLAN %d\n" : "TODO OK (0 fallas)\n", fallas);
    return fallas ? 1 : 0;
}
