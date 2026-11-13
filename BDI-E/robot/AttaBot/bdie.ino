// BDI-E — el pegamento entre el agente y esta máquina de estados.
//
// Todo lo que BDI-E agrega al firmware del laboratorio vive en este archivo,
// salvo las llamadas de una línea que lo enganchan (marcadas "BDI-E" en
// AttaBot.ino, estados.ino, comandos.ino, motores.ino y utils.h). Así el
// diff contra el firmware del laboratorio es corto y se lee de un vistazo:
//     diff -r ../_original_lab .
//
// El agente NO reemplaza la máquina de estados: la alimenta (R5). Produce una
// Action; mission.h §2 la traduce a instructionList / nav; el PID, la IMU, el
// EKF y la evasión son los del laboratorio, sin tocar.
//
// Arduino concatena los .ino — primero AttaBot.ino, después el resto en orden
// alfabético —, así que las globales y el #include "mission.h" están en
// AttaBot.ino y desde acá se ven directo.
//
// ⚠ Qué NO está acá, a propósito (V2, 28-sep-2026):
//   · target_sensor.h — el firmware del laboratorio ya detecta el objeto por
//     color: SEARCH_APPROACH se acerca al candidato y MatchColor() clasifica
//     con el balance de blancos POR ROBOT (COLOR_WB, en NVS). BDI-E solo arma
//     esa búsqueda durante la exploración y escucha el resultado.
//   · odometry.h (CC-3) — el laboratorio ya tiene un EKF (encoders + IMU +
//     ArUco) que mantiene la pose entre cuadros y navega a ciegas si la cámara
//     no contesta (StateRequestPosition, EKF_NAV).
//   · una congregación propia — la hace el MEET del laboratorio (HandleMeet,
//     MeetSlotIndex, SafeRingSlotAngle, UpdateCongregationGoal). BDI-E solo lo
//     dispara al conocer el objetivo y escucha la llegada. Ver §CONGREGACIÓN.

// ============================================================================
// ARRANQUE E IDENTIDAD
// ============================================================================

// ID 0 provisional: el real llega con CONFIG|<id> y ahí se vuelve a llamar
// Init(). Sin ese segundo Init los cinco robots creerían ser el 0 y la
// asignación de zonas los mandaría a todos a la misma franja.
void BdieInitGrid() {
  bdie::GridConfig g;   // 19 x 14 de 100 mm, origen (10, 0) — área libre 17-oct, ver grid.h
  agent.Init(0, g);
  nav.arrivalThreshold = bdie::RecommendedArrivalMm(g);
  DebugSerialPrintf("[12] BDI-E: %d celdas de %.0f mm, umbral %.1f mm, color '%s'\n",
                    g.Cells(), (double)g.cellMm, (double)nav.arrivalThreshold,
                    bdieTargetColor);
}

// El ID real llega en CONFIG. Dos cosas dependen de él:
//
// 1 · LA RANURA IR. En el firmware del laboratorio lateralCycle y centralCycle
//     eran `const int = random(...)`, inicializados ANTES de randomSeed() en
//     setup(): con el generador sin sembrar, todos los robots sacaban la MISMA
//     ranura, emitían a la vez y se leían mutuamente como obstáculo. Medido en
//     el banco: 6 % de misión completa con eso, 98 % derivándola del ID.
//     El defecto sigue en el firmware nuevo del laboratorio (AttaBot.ino:131).
// 2 · EL AGENTE, que desempata hallazgos y reparte zonas por ID.
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
// EL CICLO DE RAZONAMIENTO — estado DELIBERATE
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
  ArduinoOTA.handle();          // WAIT es donde vive OTA y en misión casi no se pasa

  /* BARRIDO DE LA CELDA (17-oct-2026). En el laboratorio, con celdas de 120 mm,
   * el enjambre llegó al 93 % de cobertura sin tocar el cubo: pisar la celda no
   * es mirarla, porque el sensor de color solo ve de frente y a centímetros. Al
   * llegar a una celda en BUSCAR el robot barre ±bdieScanHalfDeg (45°) antes de
   * deliberar (la primera versión giraba 360°; ver AttaBot.ino). Si
   * durante el giro el sensor frontal ve algo, StateTurn se interrumpe y el
   * obstáculo central se vuelve candidato (StateActiveEvasion →
   * SEARCH_APPROACH), igual que en un avance. Cuando el giro termina, la FSM
   * vuelve a WAIT y de ahí acá, ya sin barrido pendiente. */
  if (bdieScanPending) {
    bdieScanPending = false;
    if (bdieOwnsSearch && search.active && agent.bel.phase == bdie::PH_BUSCAR) {
      instructionList.clear();
      if (bdieScanHalfDeg >= 180.0f) {           // giro completo (versión anterior)
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

  /* Fuera de la congregación el MEET del laboratorio tiene que estar APAGADO:
   * con la congregación activa, cada POSITION_RESPONSE llama a
   * UpdateCongregationGoal() y reescribe nav.goal hacia la ranura — el robot
   * volvería al objetivo en vez de ir a la base. */
  if (agent.bel.phase != bdie::PH_CONGREGAR) BdieDropMeet();
  /* 24-oct-2026 (corrida 14:32): en el retorno dos robots navegaron 3–4 min con
   * rumbos al azar (error mediano 102° contra 6° explorando). Sospecha: el
   * MEET del laboratorio seguía vivo aunque BdieDropMeet no lo apagara (solo lo
   * apaga si lo pidió BDI-E) y su congregación reescribía la meta o encolaba
   * pasos propios. Fuera de CONGREGAR no tiene que haber congregación activa:
   * se apaga igual y se avisa a la base, así el log dice si era esto. */
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
    /* EL DESCUBRIDOR SE QUEDA (17-oct-2026). Ya está tocando el objetivo: es el
     * ancla de la congregación, como el líder de la CONGREGATION del
     * laboratorio, y los demás arman el anillo alrededor. Antes también corría
     * el MEET hacia una ranura a pocos cm, tenía que despegarse del cubo para
     * llegar y, si no podía, el quórum no se cumplía nunca: la congregación
     * esperó los 5 min del temporizador (corrida de las 14:24). Ahora cuenta
     * como llegado en el acto y espera quieto. */
    if (agent.bel.iFoundIt) {
      agent.OnSlotReached();
      instructionList.clear();
    } else {
      BdieStartMeet(act.x, act.y);
    }
  }
  BdiePump();

  /* Si la misión quedó cerrada, el cuerpo tiene que quedar limpio ACÁ: A_ABORT
   * es lo único que llama a nav.Reset() desde el lado del agente, elegir ese
   * plan exige deliberar, y deliberar exige !nav.isActive. Al llegar a PH_FIN
   * missionActive se vuelve false y la intercepción del WAIT no vuelve a
   * dispararse. → BDI-E/pruebas_robot/HALLAZGO_ApplyAction_no_arranca_la_navegacion.md §4 */
  if (agent.bel.phase == bdie::PH_FIN || agent.bel.phase == bdie::PH_SEGURO) {
    BdieReleaseBody();
  }
  missionActive = BdieFaseActiva();   // timeout de misión → SEGURO sin pasar por HandleBdieMessage

  deliberationUs = micros() - t0;
  state = frenar ? STOP : READ_INSTRUCTION;
}

// ============================================================================
// LO QUE NO PUEDE DEPENDER DEL ESTADO DE LA FSM — se llama en cada loop()
// ============================================================================

void BdieSyncSearch();

void BdieLoopHook() {
  /* 1 · EL GUARDIÁN DE LA NAVEGACIÓN. nav.HasTimedOut() solo se evalúa dentro
   *     de ReactiveNavStep(), que corre cuando llega una pose. Si las poses
   *     dejan de llegar —que es exactamente el fallo— el temporizador no se
   *     evalúa nunca. Evaluarlo acá lo descuelga, y soltar `nav` es lo que
   *     vuelve a habilitar la deliberación. */
  if (nav.isActive && nav.HasTimedOut()) {
    MessageDebugf("DEBUG: -1, ID: %s, NAV: guardian — timeout sin progreso, se suelta",
                  robotID.c_str());
    BdieReleaseBody();
  }

  /* 1b · TOPE DE VIAJE EN MISIÓN (17-oct-2026). El tope del laboratorio
   *     (maxNavTime) es de 3 min y es absoluto. En la corrida de las 15:02 el
   *     Atta_3 pasó exactamente esos 3 min (228 → 408 s) tras la celda (560,50),
   *     evadiendo la pared y alejándose hasta (1100,480): mientras tanto el agente
   *     no delibera, así que ni su propio «sin progreso» ni el bloqueo temporal
   *     de la celda podían actuar. En BUSCAR/DESPLEGAR un viaje dura a lo sumo
   *     BDIE_NAV_BASE_MS + BDIE_NAV_MS_POR_MM × distancia inicial; al pasarse se
   *     suelta el cuerpo y el agente marca la celda (EV_TIMEOUT → bloqueo temporal).
   *     El MEET no entra: la congregación tiene su propio cierre. */
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
    // ¿Lo que no lo deja llegar está en la meta? Mirarlo antes de irse (ver AttaBot.ino).
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

  /* 1c · YA ESTÁ EN LA BASE (17-oct-2026). FIN lo decide el agente al deliberar,
   *     y con el cuerpo navegando no delibera. En la corrida de las 15:02 el Atta_2
   *     volvía a la celda 59 —donde ya estaba estacionado el Atta_3— y pasó 30 s
   *     dando vueltas a 120 mm de la base sin declarar FIN, hasta que se abortó.
   *     En VOLVER, dentro de returnDoneMm de la base, se suelta el cuerpo (a lo
   *     sumo cada 3 s) para que el agente evalúe el cierre. */
  static unsigned long ultimaSueltaBase = 0;
  if (missionActive && agent.bel.phase == bdie::PH_VOLVER && agent.bel.session &&
      (nav.isActive || nav.pendingInit) && millis() - ultimaSueltaBase > 3000 &&
      CalculateDistance(robotPose.x, robotPose.y, agent.bel.baseX, agent.bel.baseY) <=
          agent.returnDoneMm) {
    ultimaSueltaBase = millis();
    BdieReleaseBody();
    state = STOP;
  }

  /* 2 · EL LATIDO Y EL REPARTO. Mientras el robot navega no delibera, y el HB
   *     se encolaba solo en el ciclo: medido 0,37 Hz contra los 2 Hz del
   *     diseño. Comunicar() corre por reloj propio y no duplica. Desde el
   *     14-oct-2026 corre ahí también el reparto de franjas por mensajes
   *     (Repartir): una ronda no puede esperar a que el cuerpo quede libre. */
  if (missionActive) {
    agent.Comunicar(millis());
    BdiePump();
  }

  // 3 · La búsqueda por color del laboratorio, armada solo mientras sirve.
  BdieSyncSearch();

  // 4 · Qué está pensando, a la base. Solo observación.
  BdieTelemetry();
}

// ============================================================================
// DETECCIÓN DEL OBJETIVO — reutiliza la búsqueda por color del laboratorio
// ============================================================================

// La búsqueda del laboratorio (SEARCH_OBJECT) funciona así: con search.active,
// un obstáculo CENTRAL deja de evadirse y se vuelve candidato; SEARCH_APPROACH
// se acerca lento hasta SEARCH_PROX_NEAR, lee RGBC y MatchColor() decide con el
// balance de blancos del propio robot. BDI-E la arma mientras el agente
// explora y no conoce el objetivo, y la desarma en cuanto lo conoce: durante
// la congregación el objeto es un obstáculo más, no un candidato.
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
  agent.OnTargetSeen(millis());   // dispara TGT por difusión epidémica
  BdieReleaseBody();              // lo que decida el agente, lo decide deliberando
  MessageDebugf("DEBUG: -1, ID: %s, BDI-E: objetivo confirmado en la celda %d",
                robotID.c_str(), agent.bel.myCell);
}

// En misión, el acercamiento que no terminó en hallazgo tiene que devolver el
// control a la navegación en curso — no a un RANDOM_WALK de 10 minutos, que es
// lo que hace la búsqueda manual del laboratorio y que dejaría al agente sin
// deliberar. Si no había navegación, la cola vacía lleva a DELIBERATE.
void BdieResumeAfterCandidate() {
  if (nav.isActive || nav.pendingInit) {
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }
}

// ============================================================================
// CONGREGACIÓN — el MEET del laboratorio, sin reescribirlo
// ============================================================================

// El agente decidió congregarse en (x, y). Se le pasa al MISMO manejador que
// atiende MEET|x|y desde la base: cada robot saca su ranura del anillo de la
// lista NEIGHBOR_POSITIONS (MeetSlotIndex), la corre de las paredes
// (SafeRingSlotAngle), entra en dos etapas (UpdateCongregationGoal) y
// ReactiveNavStep declara la llegada. Nada de eso lo toca BDI-E.
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

// ReactiveNavStep llegó a su meta final. Si esa meta era la ranura del MEET que
// pidió el agente, eso es "llegué a mi lugar" para el quórum.
void BdieOnNavArrived() {
  if (missionActive && bdieOwnsMeet && congregation.IsActive() &&
      congregation.stagingDone)
    agent.OnSlotReached();
  BdieMarcarBarrido();   // llegó a una celda buscando: barrerla antes de seguir
}

// Aparte de BdieOnNavArrived porque el banco compila esa función verbatim
// (sitl/extraer_lab.py) y no tiene la búsqueda por color.
void BdieMarcarBarrido() {
  if (missionActive && bdieScanEnabled && bdieOwnsSearch && search.active &&
      agent.bel.phase == bdie::PH_BUSCAR)
    bdieScanPending = true;
}

// ============================================================================
// MENSAJES — una sola puerta de entrada, la MISMA que usa el banco
// ============================================================================

/* ¿El cuerpo lo maneja la misión? (17-oct-2026) Antes era «fase distinta de
 * IDLE y de FIN», así que SEGURO —la fase tras un ABORT o el timeout de 20 min—
 * seguía contando como misión: el WAIT pasaba a DELIBERATE, P_ABORTAR devolvía
 * A_HOLD y ApplyAction vaciaba la cola en cada ciclo. Resultado en el
 * laboratorio: después de abortar, el robot ignoraba los comandos manuales
 * (un MOVE|-100 al Atta_1 no hizo nada) hasta reiniciarlo. En SEGURO el robot
 * queda quieto y vuelve a obedecer a la base, como el firmware del laboratorio. */
bool BdieFaseActiva() {
  const uint8_t f = agent.bel.phase;
  return f == bdie::PH_DESPLEGAR || f == bdie::PH_BUSCAR ||
         f == bdie::PH_CONGREGAR || f == bdie::PH_VOLVER;
}

void HandleBdieMessage() {
  /* 17-oct-2026: un robot encendido SIN CONFIG (robotID "-1") tiene el agente
   * con el ID provisional 0. Recibía el SESSION|START por broadcast, entraba a
   * la misión como «robot 0» y se sumaba al reparto: la ronda esperaba su
   * reclamo, que no llegaba nunca, y el reparto caía al respaldo a los 11,5 s
   * (corrida de las 12:51, Atta_4 encendido y sin registrar). Hasta tener ID
   * real, el robot no escucha al enjambre. */
  if (robotID == "-1") return;
  /* BDIE_SILENCIO: el canal del enjambre está «caído». SESSION entra igual:
   * abortar una corrida no puede depender de una prueba de desconexión. */
  if (BdieEnSilencio() && strncmp(receivedPacket, "SESSION", 7) != 0) return;
  const uint8_t faseAntes = agent.bel.phase;
  bdie::Feed(agent, receivedPacket, millis());   // el buffer CRUDO: POSES trae ';' y ':'
  missionActive = BdieFaseActiva();

  /* SESSION|ABORT cambia la fase DENTRO de Feed, sin pasar por Cycle(): es el
   * único camino por el que la misión se cierra sin deliberar. Si no se limpia
   * acá, el cuerpo queda navegando y la sesión siguiente lo hereda. */
  if (faseAntes != agent.bel.phase &&
      (agent.bel.phase == bdie::PH_IDLE ||
       agent.bel.phase == bdie::PH_FIN  ||
       agent.bel.phase == bdie::PH_SEGURO)) {
    BdieReleaseBody();
    state = STOP;
  }

  /* EL OBJETIVO LLEGÓ POR MENSAJE A MITAD DE UN VIAJE (17-oct-2026). En la
   * corrida de las 15:02 el Atta_3 recibió el TGT (fase BUSCAR → CONGREGAR) a los
   * 282 s mientras navegaba hacia una celda de borde, (560,50). Como el agente solo
   * delibera con el cuerpo libre, siguió 2 min más tras esa celda con la misión ya
   * en congregación; los otros dos lo esperaron en el anillo. Al pasar a CONGREGAR
   * o a VOLVER se suelta el cuerpo y el agente decide en el ciclo siguiente. */
  if (faseAntes != agent.bel.phase &&
      (agent.bel.phase == bdie::PH_CONGREGAR || agent.bel.phase == bdie::PH_VOLVER) &&
      (nav.isActive || nav.pendingInit || !instructionList.empty())) {
    BdieReleaseBody();
    state = STOP;
  }

  /* Cambió el objetivo (desempate por menor ID) con un MEET en curso hacia el
   * punto viejo: se suelta el cuerpo y el agente, al deliberar, pide el MEET
   * al punto nuevo. */
  if (bdieOwnsMeet && agent.MeetStale()) {
    BdieReleaseBody();
    state = STOP;
  }
}

// TARGET_COLOR|rojo|verde|azul — el color que busca la misión. Es un parámetro,
// no código: los planes BDI son declarativos y MatchColor ya sabe los tres.
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
// TELEMETRÍA — qué está pensando el agente, contado a la base
// ============================================================================

// Unicast a la base, nunca broadcast: ningún par la necesita. Si la base no
// está configurada todavía, no se manda nada. El formato y los períodos están
// en telemetry.h, que es el MISMO código que usa el banco.
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
// BDIE_SILENCIO — desconexión reproducible (prueba, 14-oct-2026)
// ============================================================================

// ¿El canal del enjambre está «caído» por BDIE_SILENCIO? Resta con signo, por
// el desborde de millis().
bool BdieEnSilencio() {
  return bdieSilencioHasta != 0 && (int32_t)(millis() - bdieSilencioHasta) < 0;
}

// Saca lo que el agente encoló. En silencio se DESCARTA —como si la radio no
// llegara a nadie— para que la cola no se llene y no salga todo junto al volver.
void BdiePump() {
  if (!BdieEnSilencio()) { bdie::PumpOutgoing(agent); return; }
  bdie::Wire w;
  while (bdie::Emit(agent, w)) {}
}

// BDIE_SILENCIO|<s> — durante s segundos (0 = cortar ya) el robot ni manda ni
// escucha HB/TGT/ZC/ZA/POSES. Sigue navegando: la pose de la navegación llega
// por POSITION_RESPONSE, que no es del enjambre. Los demás dejan de oírlo a los
// 3 s y re-reparten; él, solo, se queda con toda el área; al volver, todos se
// oyen de nuevo y se re-reparte entre todos. Es la prueba O5.5 de
// PRUEBAS_observacion.md.
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
