/*****************************************************************************
 * latencia.ino — Latencia ENTRE ROBOTS para OE3 (05-nov-2026)
 *
 *   A  ── LAT_PING|seq|t_us|idA ──▶ broadcast (el canal de HB/TGT/ZC/ZA)
 *   B  ── LAT_PONG|seq|t_us|idB ──▶ A, directo (eco del t_us de A)
 *   A  ── LAT|idB|rtt_ms|enviados|recibidos ──▶ base (ConsoleLog)
 *
 * rtt = micros() de A al recibir el pong − t_us que A mismo puso en el ping:
 * un solo reloj, sin sincronizar nada. Es cota superior del tramo de difusión.
 * El resumen lo saca herramientas/analisis_corrida.py (--lat para solo esto).
 *
 * Contesta siempre que tenga ID y no esté en BDIE_SILENCIO, aunque su propio
 * ping esté apagado: con un solo robot midiendo ya se mide contra todos.
 * Máximo 5 campos por mensaje (restricción del parser): LAT tiene 5.
 *****************************************************************************/

static const uint32_t LAT_PERIODO_OMISION_MS = 1000;
static const uint32_t LAT_PERIODO_MIN_MS     = 200;
static const uint32_t LAT_SEQ_VENTANA        = 8;      // pongs más viejos se descartan
static const float    LAT_RTT_MAX_MS         = 5000.0f;

// NAV_CONFIG|LAT|n — 0 apaga; 1 = cada LAT_PERIODO_OMISION_MS; n ≥ 2 = cada n ms
// (mínimo LAT_PERIODO_MIN_MS). Encender reinicia los contadores.
void LatConfigurar(long n) {
  if (n <= 0) {
    latPeriodoMs = 0;
  } else {
    latPeriodoMs = (n == 1) ? LAT_PERIODO_OMISION_MS
                            : (uint32_t)max((long)LAT_PERIODO_MIN_MS, n);
    latEnviados = 0;
    latRecibidos.clear();
    latArrancado = false;
  }
  char buf[64];
  if (latPeriodoMs == 0) snprintf(buf, sizeof(buf), "NAV_CONFIG: latencia=apagada");
  else snprintf(buf, sizeof(buf), "NAV_CONFIG: latencia=cada %lu ms", (unsigned long)latPeriodoMs);
  SendMessage(robots["Base"], buf);
}

// Desde loop(): un ping por período, por difusión.
void LatTick() {
  if (latPeriodoMs == 0 || robotID == "-1" || BdieEnSilencio()) return;
  if (robots.find("Broadcast") == robots.end()) return;
  const uint32_t ahora = millis();
  if (latArrancado && (uint32_t)(ahora - latUltimoMs) < latPeriodoMs) return;
  latArrancado = true;
  latUltimoMs = ahora;
  latSeq++;
  latEnviados++;
  char buf[64];
  snprintf(buf, sizeof(buf), "LAT_PING|%lu|%lu|%s", (unsigned long)latSeq,
           (unsigned long)(uint32_t)micros(), robotID.c_str());
  SendMessage(robots["Broadcast"], buf);
}

// LAT_PING|seq|t_us|idA — devolverlo a quien lo mandó, con mi ID.
void HandleLatPing(const std::array<String, 6> &arguments) {
  if (robotID == "-1" || BdieEnSilencio()) return;
  if (arguments[3] == robotID) return;                  // eco de mi propio broadcast
  char buf[64];
  snprintf(buf, sizeof(buf), "LAT_PONG|%s|%s|%s", arguments[1].c_str(),
           arguments[2].c_str(), robotID.c_str());
  SendMessage(udp.remoteIP(), buf);
}

// LAT_PONG|seq|t_us|idB — medir y reportar a la base.
void HandleLatPong(const std::array<String, 6> &arguments) {
  if (latPeriodoMs == 0 || BdieEnSilencio()) return;
  const uint32_t seq = strtoul(arguments[1].c_str(), nullptr, 10);
  const uint32_t t0  = strtoul(arguments[2].c_str(), nullptr, 10);
  const int      de  = arguments[3].toInt();
  if (seq == 0 || seq > latSeq || latSeq - seq > LAT_SEQ_VENTANA) return;   // ajeno o viejo
  if (de < 0 || String(de) == robotID) return;
  const float rttMs = (float)(uint32_t)((uint32_t)micros() - t0) / 1000.0f;
  if (rttMs > LAT_RTT_MAX_MS) return;
  const uint32_t rec = ++latRecibidos[de];
  char buf[64];
  snprintf(buf, sizeof(buf), "LAT|%d|%.1f|%lu|%lu", de, rttMs,
           (unsigned long)latEnviados, (unsigned long)rec);
  SendMessage(robots["Base"], buf);
}
