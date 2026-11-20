// ============================================================================
// bdie.ino — Acople entre el agente BDI-E y la máquina de estados del robot
//
// Todo lo que BDI-E agrega al firmware del laboratorio está en este archivo,
// salvo las llamadas de una línea que lo enganchan, marcadas «BDI-E» en
// AttaBot.ino, estados.ino, comandos.ino, motores.ino y utils.h. Así la
// diferencia con el firmware del laboratorio (Controller/AttaBot en
// jcbrenes/Atta-Bot, commit 82b9b8b) es corta.
//
// El agente no reemplaza la máquina de estados, la alimenta: produce una
// Action, mission.h la traduce a instrucciones, y el PID, la IMU, el EKF y la
// evasión siguen siendo los del laboratorio.
//
// Arduino concatena los .ino (primero AttaBot.ino y después el resto en orden
// alfabético), así que las globales y el #include "mission.h" están en
// AttaBot.ino y desde aquí se ven directamente.
//
// BDI-E reutiliza tres capacidades del laboratorio en vez de duplicarlas:
//   · la detección por color: SEARCH_APPROACH se acerca al candidato y
//     MatchColor() lo clasifica con el balance de blancos de cada robot
//     (COLOR_WB, en NVS); BDI-E arma esa búsqueda al explorar y escucha el
//     resultado;
//   · la estimación de la pose: el EKF del laboratorio (encoders + IMU +
//     ArUco) mantiene la pose entre cuadros de la cámara;
//   · la congregación: el MEET del laboratorio (HandleMeet, MeetSlotIndex,
//     SafeRingSlotAngle, UpdateCongregationGoal), que BDI-E solo dispara al
//     conocer el objetivo. Ver la sección de congregación.

// ============================================================================
// ARRANQUE E IDENTIDAD
// ============================================================================

// El ID real llega con CONFIG|<id>, y ahí se vuelve a llamar a Init(). Hasta
// entonces el agente usa el ID provisional 0.
void BdieInitGrid() {
  bdie::GridConfig g;   // 19 × 14 celdas de 100 mm, origen (10, 0); ver grid.h
  agent.Init(0, g);
  nav.arrivalThreshold = bdie::RecommendedArrivalMm(g);
  DebugSerialPrintf("[12] BDI-E: %d celdas de %.0f mm, umbral %.1f mm, color '%s'\n",
                    g.Cells(), (double)g.cellMm, (double)nav.arrivalThreshold,
                    bdieTargetColor);
}

// El ID real llega con CONFIG. Dos cosas dependen de él:
//
// 1 · La ranura de emisión IR. Si lateralCycle y centralCycle se sortean antes
//     de sembrar el generador, todos los robots obtienen la misma ranura,
//     emiten a la vez y se leen mutuamente como obstáculo. Derivarla del ID
//     garantiza ranuras distintas.
// 2 · El agente, que desempata hallazgos y reparte zonas por ID.
void BdieOnConfigId() {
  const int idNum = robotID.toInt();
  if (idNum < 0) return;
  lateralCycle = idNum % numberOfCycles;
  centralCycle = (lateralCycle + numberOfCycles / 2) % numberOfCycles;
  agent.Init(idNum, agent.grid);
  nav.arrivalThreshold = bdie::RecommendedArrivalMm(agent.grid);
  MessageDebugf("DEBUG: -1, ID: %s, BDI-E: agente %d, ranuras IR L%d C%d",
                robotID.c_str(), idNum, lateralCycle, centralCycle);
}

// ============================================================================
// CICLO DE RAZONAMIENTO — estado DELIBERATE
// ============================================================================

// Suelta el cuerpo: sin navegación, sin cola y sin búsqueda armada.
void BdieReleaseBody() {
  nav.Reset();
  instructionList.clear();
  navRequestRetries = 0;
  BdieDropMeet();
}

void StateDeliberate() {
  const uint32_t t0 = micros();
  ArduinoOTA.handle();          // OTA se atiende en WAIT, por donde en misión casi no se pasa

  /* Barrido de la celda. El sensor de color solo ve de frente y a pocos
   * centímetros, así que pisar una celda no es mirarla: al llegar a una celda en
   * BUSCAR, el robot gira ±bdieScanHalfDeg (45°) antes de deliberar. Si durante
   * el giro el sensor frontal ve algo, StateTurn se interrumpe y el obstáculo
   * central se vuelve candidato (StateActiveEvasion → SEARCH_APPROACH), igual
   * que en un avance. Al terminar el giro, la máquina de estados vuelve a WAIT y
   * de ahí aquí, ya sin barrido pendiente. */
  if (bdieScanPending) {
    bdieScanPending = false;
    if (bdieOwnsSearch && search.active && agent.bel.phase == bdie::PH_BUSCAR) {
      instructionList.clear();
      if (bdieScanHalfDeg >= 180.0f) {           // giro completo
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = DegToArc(360.0f);
        instructionList.push_back(fsmInstruction);
      } else {                                   // barrido corto: +h, luego −2h
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = DegToArc(bdieScanHalfDeg);
        instructionList.push_back(fsmInstruction);
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = DegToArc(-2.0f * bdieScanHalfDeg);
        instructionList.push_back(fsmInstruction);
      }
      deliberationUs = micros() - t0;
      state = READ_INSTRUCTION;
      return;
    }
  }

  bdie::Action act    = agent.Cycle(millis());

  /* Fuera de la congregación el MEET del laboratorio debe estar apagado: con
   * la congregación activa, cada POSITION_RESPONSE llama a
   * UpdateCongregationGoal() y reescribe la meta hacia la ranura, y el robot
   * volvería al objetivo en vez de ir a la base. */
  if (agent.bel.phase != bdie::PH_CONGREGAR) BdieDropMeet();
  /* Fuera de CONGREGAR no debe haber ninguna congregación activa, tampoco una
   * que no haya pedido BDI-E: se apaga igual y se avisa a la base. */
  if (missionActive && agent.bel.phase != bdie::PH_CONGREGAR &&
      (congregation.IsActive() || congregation.hasGlobalTarget)) {
    char buf[96];
    snprintf(buf, sizeof(buf), "WARNING: MEET del lab seguia activo en fase %d (lider=%d, staging=%d) - se apaga",
             (int)agent.bel.phase, (int)congregation.isLeader, (int)congregation.stagingDone);
    SendMessage(robots["Base"], buf);
    congregation.Reset();
    bdieOwnsMeet = false;
    nav.Reset();
    instructionList.clear();
  }

  const bool   frenar = bdie::ApplyAction(act);
  if (act.kind == bdie::Action::A_MEET) {
    /* El descubridor se queda: ya está frente al objetivo y es el ancla de la
     * congregación; los demás arman el anillo a su alrededor. Cuenta como llegado
     * de inmediato y espera quieto. */
    if (agent.bel.iFoundIt) {
      agent.OnSlotReached();
      instructionList.clear();
    } else {
      BdieStartMeet(act.x, act.y);
    }
  }
  BdiePump();

  /* Con la misión cerrada, el cuerpo debe quedar limpio aquí: A_ABORT es lo
   * único que llama a nav.Reset() desde el agente, elegir ese plan exige
   * deliberar, y deliberar exige !nav.isActive. Al llegar a PH_FIN,
   * missionActive pasa a false y la intercepción de WAIT ya no se dispara. */
  if (agent.bel.phase == bdie::PH_FIN || agent.bel.phase == bdie::PH_SEGURO) {
    BdieReleaseBody();
  }
  missionActive = BdieFaseActiva();   // el tiempo límite de la misión lleva a SEGURO sin pasar por HandleBdieMessage

  deliberationUs = micros() - t0;
  state = frenar ? STOP : READ_INSTRUCTION;
}

// ============================================================================
// LO QUE NO PUEDE DEPENDER DEL ESTADO DE LA MÁQUINA — se llama en cada loop()
// ============================================================================

void BdieSyncSearch();

void BdieLoopHook() {
  /* 1 · Guardián de la navegación. nav.HasTimedOut() solo se evalúa dentro de
   *     ReactiveNavStep(), que corre cuando llega una pose; si las poses dejan
   *     de llegar, el temporizador nunca se evalúa. Evaluarlo aquí suelta la
   *     navegación y vuelve a habilitar la deliberación. */
  if (nav.isActive && nav.HasTimedOut()) {
    MessageDebugf("DEBUG: -1, ID: %s, NAV: guardian — timeout sin progreso, se suelta",
                  robotID.c_str());
    BdieReleaseBody();
  }

  /* 1b · Tope de viaje en misión. El tope del laboratorio (maxNavTime) es de
   *      3 min y es absoluto; mientras el robot navega, el agente no delibera y
   *      ni su detección de falta de progreso ni el bloqueo temporal pueden
   *      actuar. En BUSCAR y DESPLEGAR un viaje dura a lo sumo BDIE_NAV_BASE_MS
   *      + BDIE_NAV_MS_POR_MM × distancia inicial; al pasarse, se suelta el
   *      cuerpo y el agente marca la celda (EV_TIMEOUT → bloqueo temporal). El
   *      MEET no entra: la congregación tiene su propio cierre. */
  static unsigned long navTopeInicio = 0, navTopeMs = 0;
  if (nav.isActive && nav.startTime != navTopeInicio) {
    navTopeInicio = nav.startTime;
    const float d = CalculateDistance(robotPose.x, robotPose.y, nav.goalX, nav.goalY);
    navTopeMs = BDIE_NAV_BASE_MS + (unsigned long)(BDIE_NAV_MS_POR_MM * d);
  }
  if (missionActive && nav.isActive && !bdieOwnsMeet &&
      (agent.bel.phase == bdie::PH_BUSCAR || agent.bel.phase == bdie::PH_DESPLEGAR) &&
      millis() - nav.startTime > navTopeMs) {
    char buf[90];
    snprintf(buf, sizeof(buf), "WARNING: viaje a (%.0f,%.0f) sin llegar en %lu ms - se suelta",
             nav.goalX, nav.goalY, navTopeMs);
    SendMessage(robots["Base"], buf);
    const float gx = nav.goalX, gy = nav.goalY;
    const float dMeta = CalculateDistance(robotPose.x, robotPose.y, gx, gy);
    BdieReleaseBody();
    navTopeInicio = 0;
    state = STOP;
    // Si lo que impide llegar está en la meta, mirarlo antes de irse (ver AttaBot.ino).
    if (agent.bel.phase == bdie::PH_BUSCAR && bdieOwnsSearch && search.active &&
        frontSensorInitialized && dMeta < BDIE_REVISAR_META_MM && dMeta > 40.0f) {
      const float rumbo = atan2(gy - robotPose.y, gx - robotPose.x) * RAD_TO_DEG;
      const float giro  = NormalizeAngle(rumbo - robotPose.angle);
      fsmInstruction[0] = TURN;   fsmInstruction[1] = DegToArc(giro);  instructionList.push_back(fsmInstruction);
      fsmInstruction[0] = WAIT;   fsmInstruction[1] = 150;             instructionList.push_back(fsmInstruction);
      fsmInstruction[0] = SEARCH_APPROACH; fsmInstruction[1] = -1;     instructionList.push_back(fsmInstruction);
      isEvading = false;
      state = READ_INSTRUCTION;
      SearchTrace("viaje cortado a %.0f mm de la meta: giro %.0f y reviso", dMeta, giro);
    }
  }

  /* 1c · Ya está en la base. FIN lo decide el agente al deliberar, y con el
   *      cuerpo navegando no delibera. En VOLVER, dentro de returnDoneMm de la
   *      base, se suelta el cuerpo (a lo sumo cada 3 s) para que el agente
   *      evalúe el cierre. */
  static unsigned long ultimaSueltaBase = 0;
  if (missionActive && agent.bel.phase == bdie::PH_VOLVER && agent.bel.session &&
      (nav.isActive || nav.pendingInit) && millis() - ultimaSueltaBase > 3000 &&
      CalculateDistance(robotPose.x, robotPose.y, agent.bel.baseX, agent.bel.baseY) <=
          agent.returnDoneMm) {
    ultimaSueltaBase = millis();
    BdieReleaseBody();
    state = STOP;
  }

  /* 2 · Latido y reparto. Mientras el robot navega no delibera, así que el
   *     latido y el reparto de franjas (Repartir) corren aquí, en Comunicar(),
   *     con reloj propio y sin duplicar mensajes. */
  if (missionActive) {
    agent.Comunicar(millis());
    BdiePump();
  }

  // 3 · La búsqueda por color del laboratorio, armada solo mientras sirve.
  BdieSyncSearch();

  // 4 · Telemetría de razonamiento a la base. Solo observación.
  BdieTelemetry();
}

// ============================================================================
// DETECCIÓN DEL OBJETIVO — reutiliza la búsqueda por color del laboratorio
// ============================================================================

// La búsqueda del laboratorio (SEARCH_OBJECT) funciona así: con search.active,
// un obstáculo central deja de evadirse y se vuelve candidato; SEARCH_APPROACH
// se acerca despacio hasta SEARCH_PROX_NEAR, lee RGBC y MatchColor() decide con
// el balance de blancos del propio robot. BDI-E la arma mientras el agente
// explora sin conocer el objetivo y la desarma en cuanto lo conoce: durante la
// congregación el objeto es un obstáculo más.
//
// bdieOwnsSearch distingue la búsqueda que armó BDI-E de un SEARCH_OBJECT
// manual desde la base, que no se toca.
void BdieSyncSearch() {
  if (bdieOwnsSearch && !search.active) bdieOwnsSearch = false;   // la cerró SEARCH_APPROACH

  const bool want = missionActive && frontSensorInitialized &&
                    !agent.bel.targetKnown &&
                    (agent.bel.phase == bdie::PH_DESPLEGAR ||
                     agent.bel.phase == bdie::PH_BUSCAR);

  if (want && !search.active) {
    strncpy(search.targetColor, bdieTargetColor, sizeof(search.targetColor) - 1);
    search.targetColor[sizeof(search.targetColor) - 1] = '\0';
    search.active = true;
    bdieOwnsSearch = true;
    frontSensor.enableColor(true);
    MessageDebugf("DEBUG: -1, ID: %s, BDI-E: busqueda por color '%s' armada",
                  robotID.c_str(), bdieTargetColor);
  } else if (!want && bdieOwnsSearch) {
    search.Reset();
    bdieOwnsSearch = false;
    if (frontSensorInitialized) frontSensor.enableColor(false);
    // Si estaba a mitad de un acercamiento, StateSearchApproach ve
    // !search.active, frena y devuelve la navegación (ver estados.ino).
  }
}

// MatchColor() dio positivo en SEARCH_APPROACH. El robot está frente al objeto.
void BdieTargetConfirmed() {
  agent.OnTargetSeen(millis());   // dispara la difusión epidémica del TGT
  BdieReleaseBody();              // el siguiente paso lo decide el agente al deliberar
  MessageDebugf("DEBUG: -1, ID: %s, BDI-E: objetivo confirmado en la celda %d",
                robotID.c_str(), agent.bel.myCell);
}

// En misión, un acercamiento que no terminó en hallazgo devuelve el control a
// la navegación en curso, y no a la caminata aleatoria de 10 min de la
// búsqueda manual del laboratorio, que dejaría al agente sin deliberar. Si no
// había navegación, la cola vacía lleva a DELIBERATE.
void BdieResumeAfterCandidate() {
  if (nav.isActive || nav.pendingInit) {
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }
}

// ============================================================================
// CONGREGACIÓN — el MEET del laboratorio, sin modificarlo
// ============================================================================

// El agente decidió congregarse en (x, y). Se usa el mismo manejador que
// atiende MEET|x|y desde la base: cada robot obtiene su ranura del anillo a
// partir de la lista NEIGHBOR_POSITIONS (MeetSlotIndex), la aleja de las
// paredes (SafeRingSlotAngle), entra en dos etapas (UpdateCongregationGoal) y
// ReactiveNavStep declara la llegada.
void BdieStartMeet(float x, float y) {
  char bx[16], by[16];
  snprintf(bx, sizeof(bx), "%.0f", x);
  snprintf(by, sizeof(by), "%.0f", y);
  std::array<String, 6> args = { String("MEET"), String(bx), String(by),
                                 String(""), String(""), String("") };
  HandleMeet(args);
  bdieOwnsMeet = true;
  MessageDebugf("DEBUG: -1, ID: %s, BDI-E: MEET del lab en (%s,%s)",
                robotID.c_str(), bx, by);
}

// Apaga la congregación del laboratorio si la había pedido BDI-E. Un MEET
// manual desde la base (bdieOwnsMeet == false) no se toca.
void BdieDropMeet() {
  if (!bdieOwnsMeet) return;
  congregation.Reset();
  bdieOwnsMeet = false;
}

// ReactiveNavStep llegó a su meta final. Si era la ranura del MEET que pidió
// el agente, cuenta como «llegué a mi lugar» para el quórum.
void BdieOnNavArrived() {
  if (missionActive && bdieOwnsMeet && congregation.IsActive() &&
      congregation.stagingDone)
    agent.OnSlotReached();
  BdieMarcarBarrido();   // llegó a una celda buscando: barrerla antes de seguir
}

// Separada de BdieOnNavArrived porque el banco compila esa función tal cual
// (sitl/extraer_lab.py) y no tiene la búsqueda por color.
void BdieMarcarBarrido() {
  if (missionActive && bdieScanEnabled && bdieOwnsSearch && search.active &&
      agent.bel.phase == bdie::PH_BUSCAR)
    bdieScanPending = true;
}

// ============================================================================
// MENSAJES — una sola puerta de entrada, la misma que usa el banco
// ============================================================================

/* ¿El cuerpo lo maneja la misión? Solo en las fases activas: en SEGURO (tras
 * un ABORT o el tiempo límite) y en FIN el robot queda quieto y vuelve a
 * obedecer los comandos manuales de la base, como el firmware del laboratorio. */
bool BdieFaseActiva() {
  const uint8_t f = agent.bel.phase;
  return f == bdie::PH_DESPLEGAR || f == bdie::PH_BUSCAR ||
         f == bdie::PH_CONGREGAR || f == bdie::PH_VOLVER;
}

void HandleBdieMessage() {
  /* Un robot encendido sin CONFIG (robotID "-1") tiene el agente con el ID
   * provisional 0; si escuchara al enjambre se sumaría al reparto como «robot 0»
   * y la ronda esperaría un reclamo que nunca llega. Hasta tener su ID real, el
   * robot no escucha al enjambre. */
  if (robotID == "-1") return;
  /* BDIE_SILENCIO simula que el canal del enjambre está caído. SESSION entra
   * igual, para poder abortar una corrida durante la prueba. */
  if (BdieEnSilencio() && strncmp(receivedPacket, "SESSION", 7) != 0) return;
  const uint8_t faseAntes = agent.bel.phase;
  bdie::Feed(agent, receivedPacket, millis());   // el búfer crudo: POSES trae ';' y ':'
  missionActive = BdieFaseActiva();

  /* SESSION|ABORT cambia la fase dentro de Feed, sin pasar por Cycle(): es el
   * único camino por el que la misión se cierra sin deliberar. Si el cuerpo no
   * se limpia aquí, sigue navegando y la sesión siguiente lo hereda. */
  if (faseAntes != agent.bel.phase &&
      (agent.bel.phase == bdie::PH_IDLE ||
       agent.bel.phase == bdie::PH_FIN  ||
       agent.bel.phase == bdie::PH_SEGURO)) {
    BdieReleaseBody();
    state = STOP;
  }

  /* El objetivo llegó por mensaje a mitad de un viaje. Como el agente solo
   * delibera con el cuerpo libre, el robot seguiría tras su celda con la misión
   * ya en congregación. Al pasar a CONGREGAR o a VOLVER se suelta el cuerpo y el
   * agente decide en el ciclo siguiente. */
  if (faseAntes != agent.bel.phase &&
      (agent.bel.phase == bdie::PH_CONGREGAR || agent.bel.phase == bdie::PH_VOLVER) &&
      (nav.isActive || nav.pendingInit || !instructionList.empty())) {
    BdieReleaseBody();
    state = STOP;
  }

  /* Cambió el objetivo (desempate por menor ID) con un MEET en curso hacia el
   * punto anterior: se suelta el cuerpo y el agente pide el MEET al punto nuevo. */
  if (bdieOwnsMeet && agent.MeetStale()) {
    BdieReleaseBody();
    state = STOP;
  }
}

// TARGET_COLOR|rojo|verde|azul — color que busca la misión. Es un parámetro:
// los planes no cambian y MatchColor() ya conoce los tres colores.
void HandleTargetColor(const std::array<String, 6> &arguments) {
  const String &c = arguments[1];
  char buf[70];
  if (c == "rojo" || c == "verde" || c == "azul") {
    strncpy(bdieTargetColor, c.c_str(), sizeof(bdieTargetColor) - 1);
    bdieTargetColor[sizeof(bdieTargetColor) - 1] = '\0';
    snprintf(buf, sizeof(buf), "TARGET_COLOR: %s", bdieTargetColor);
  } else {
    snprintf(buf, sizeof(buf), "TARGET_COLOR: '%s' no valido (rojo|verde|azul)",
             c.c_str());
  }
  SendMessage(robots["Base"], buf);
}

// ============================================================================
// TELEMETRÍA — el razonamiento del agente, hacia la base
// ============================================================================

// Unicast a la base, nunca difusión: ningún otro robot la necesita. Si la base
// todavía no está configurada, no se envía nada. El formato y los períodos
// están en telemetry.h, el mismo código que usa el banco.
void BdieTelemetry() {
  if (bdieTelemMode == 0 || robotID == "-1") return;
  if (robots.find("Base") == robots.end() || robots["Base"] == IPAddress(0, 0, 0, 0))
    return;
  static char buf[1024];
  const uint32_t now = millis();
  if (bdieTelem.DueThink(agent, now) &&
      bdie::EncodeThink(agent, now, buf, sizeof(buf)) > 0)
    SendMessage(robots["Base"], buf);
  if (bdieTelemMode >= 2 && bdieTelem.DueMap(agent, now) &&
      bdie::EncodeMap(agent, now, buf, sizeof(buf)) > 0)
    SendMessage(robots["Base"], buf);
}

// ============================================================================
// BDIE_SILENCIO — desconexión reproducible para pruebas
// ============================================================================

// ¿El canal del enjambre está «caído» por BDIE_SILENCIO? La resta va con
// signo por el desborde de millis().
bool BdieEnSilencio() {
  return bdieSilencioHasta != 0 && (int32_t)(millis() - bdieSilencioHasta) < 0;
}

// Saca lo que el agente encoló. En silencio se descarta, como si la radio no
// llegara a nadie, para que la cola no se llene y no salga todo junto al volver.
void BdiePump() {
  if (!BdieEnSilencio()) { bdie::PumpOutgoing(agent); return; }
  bdie::Wire w;
  while (bdie::Emit(agent, w)) {}
}

// BDIE_SILENCIO|<s> — durante s segundos (0 = cortar ya) el robot no envía ni
// escucha HB, TGT, ZC, ZA ni POSES. Sigue navegando, porque la pose de la
// navegación llega por POSITION_RESPONSE, que no es del enjambre. Los demás
// dejan de oírlo a los 3 s y reparten de nuevo; él se queda con toda el área;
// al volver, todos se oyen otra vez y se reparte entre todos.
void HandleBdieSilencio(const std::array<String, 6> &arguments) {
  long s = arguments[1].toInt();
  if (s < 0) s = 0;
  if (s > 120) s = 120;
  bdieSilencioHasta = s > 0 ? (millis() + (uint32_t)s * 1000u) : 0;
  if (bdieSilencioHasta == 0 && s > 0) bdieSilencioHasta = 1;
  char buf[48];
  snprintf(buf, sizeof(buf), "BDIE_SILENCIO: %ld s", s);
  SendMessage(robots["Base"], buf);
}

// BDIE_TELEM|0|1|2 — apagada, solo THINK, THINK + BMAP.
void HandleBdieTelem(const std::array<String, 6> &arguments) {
  const int m = arguments[1].toInt();
  if (m >= 0 && m <= 2) {
    bdieTelemMode = (uint8_t)m;
    bdieTelem.Reset();
  }
  char buf[48];
  snprintf(buf, sizeof(buf), "BDIE_TELEM: %d", (int)bdieTelemMode);
  SendMessage(robots["Base"], buf);
}
