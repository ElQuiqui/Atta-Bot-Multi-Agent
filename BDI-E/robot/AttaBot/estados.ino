// La maquina de estados: loop() y los estados que despacha.
//
// Cada ciclo lee sensores, atiende la radio y avanza el estado en curso. Los
// estados se encolan en instructionList y READ_INSTRUCTION los va sacando.
//
// Arduino concatena todos los .ino de la carpeta en una sola unidad de
// traduccion — primero AttaBot.ino, despues el resto en orden alfabetico — asi
// que las constantes, las variables globales y las declaraciones forward viven
// en AttaBot.ino y desde aca se ven directo, sin extern ni cabeceras.

// ============================================================================
// ESTADOS DE LA FSM
// ============================================================================

// Espera pasiva de instructionValue ms. Es el unico estado donde se atiende el
// OTA, asi que el robot solo acepta una actualizacion cuando esta quieto. Al
// vencer el plazo pasa a leer la proxima instruccion, salvo que el movimiento
// anterior no haya terminado: ahi retrocede primero.
void StateWait() {
  ArduinoOTA.handle();

  // BDI-E (punto 3): cuerpo libre y misión activa → deliberar
  if (missionActive && instructionList.empty() && !nav.isActive &&
      !nav.pendingInit) {
    agent.OnArrived();                       // EV_ARRIVED
    state = DELIBERATE;
    return;
  }

  if ((millis() - movement.previousMillis) >= instructionValue) {
    movement.previousMillis = millis();
    ResetPID();
    if (movementReady) {
      state = READ_INSTRUCTION;
    } else {
      direction = '-';
      state = REVERSE;
    }
  }
}

// Avanza instructionValue mm. Si un sensor ve un obstaculo a mitad de camino,
// guarda en intContext cuanto faltaba y corta: RESUME_AFTER_EVASION retoma
// justo ese resto en vez de repetir el tramo entero.
void StateMove() {
  if (MotionTimedOut()) { AbortMotion("MOVE"); return; }
  movementReady = MoveDistanceByWheel(instructionValue, instructionValue);

  if (movementReady) {
    MessageDebugf("DEBUG: -1, ID: %s, Movimiento completado",
                  robotID.c_str());
    intContext.Clear();
    isEvading = false;
    state = STOP;

  } else if (obstacles.HasAnyObstacle() && !isEvading) {
    obstacles.UpdateBitmap();

    intContext.wasInterrupted = true;
    intContext.previousState = MOVE;
    intContext.leftPulsesBeforeStop = movement.pastLeftPulseCount;
    intContext.rightPulsesBeforeStop = movement.pastRightPulseCount;

    float avgTraveled =
        (movement.pastLeftPulseCount + movement.pastRightPulseCount) / 2.0 *
        millimetersPerPulse;
    intContext.remainingValue = instructionValue - avgTraveled;

    isEvading = true;
    evasionStartTime = millis();

    MessageDebugf("DEBUG: -1, ID: %s, MOVE interrumpido: restante=%.1fmm",
                  robotID.c_str(), intContext.remainingValue);

    state = STOP;
  }
}

// Giro en el lugar. Con IMU disponible es lazo cerrado en yaw: acumula el giro
// real con unwrap incremental, re-apunta el arco restante en cada ciclo y al
// terminar mide el residuo y encola correcciones hasta entrar en tolerancia o
// agotar imuTurnMaxCorrections. El objetivo lo define el IMU y no la geometria
// supuesta, asi que la rueda loca, el stiction y un centerToWheelDistance mal
// calibrado dejan de producir deficit.
//
// Si el yaw deja de avanzar (IMU muda o congelada) cierra por encoders para no
// girar infinito. Sin IMU cae al giro a ciegas por distancia de rueda.
void StateTurn() {
  if (MotionTimedOut()) { AbortMotion("TURN"); return; }
  if (imuAvailable && !imuTurnActive) {
    imuTurnActive      = true;
    imuTurnStartYaw    = yaw;
    imuTurnPrevYaw     = yaw;
    imuTurnAccumDeg    = 0.0f;
    imuTurnSettleUntil = 0;
    imuTurnTargetDeg = (instructionValue / centerToWheelDistance) * RAD_TO_DEG;
  }

  if (imuAvailable && imuTurnActive) {
    // Giro cerrado en yaw: acumular el giro real con unwrap incremental y
    // re-apuntar el arco restante en cada ciclo. El objetivo lo define el
    // IMU, no la geometría supuesta — rueda loca, stiction y wheel_dist
    // dejan de producir déficit.
    float dYaw = yaw - imuTurnPrevYaw;
    if (dYaw >  180.0f) dYaw -= 360.0f;
    if (dYaw < -180.0f) dYaw += 360.0f;
    imuTurnAccumDeg += dYaw * yawScale;
    imuTurnPrevYaw   = yaw;

    if (imuTurnSettleUntil != 0) {
      // Motores ya cortados: seguir midiendo hasta que el coast termine,
      // así la verificación final incluye la inercia (la cámara mostraba
      // 2-12° de giro extra después del corte que el IMU no contaba)
      if (millis() < imuTurnSettleUntil) {
        return;
      }
      imuTurnSettleUntil = 0;

      // delta viene sin wrap, asi que sirve para arcos de mas de 180°; un error
      // positivo significa que el robot giro de menos.
      float delta = imuTurnAccumDeg;
      float error = imuTurnTargetDeg - delta;
      MessageDebugf("DEBUG: -1, ID: %s, TURN IMU: objetivo=%.1f° real=%.1f° error=%.1f° corr#%d (yaw %.1f→%.1f)",
                    robotID.c_str(), imuTurnTargetDeg, delta, error,
                    imuTurnCorrCount, imuTurnStartYaw, yaw);

      if (abs(error) > imuTurnTolerance &&
          imuTurnCorrCount < imuTurnMaxCorrections) {
        // Closed-loop iterativo: encolar otra corrección y volver a medir
        // tras su settle. Ya no es de un solo tiro — itera hasta |error|<tol
        // o agotar imuTurnMaxCorrections. La corrección corre SIN brake-lead
        // (ver rama de manejo) para que el arco chico no se cancele solo.
        float corrArc = DegToArc(error);
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = corrArc;
        instructionList.push_front(fsmInstruction);
        imuTurnIsCorrection = true;
        imuTurnCorrCount++;
        MessageDebugf("DEBUG: -1, ID: %s, TURN corrección #%d: %.1f° (arc=%.1fmm)",
                      robotID.c_str(), imuTurnCorrCount, error, corrArc);
      } else {
        imuTurnIsCorrection = false;
        imuTurnCorrCount    = 0;
      }
      imuTurnActive = false;
      movementReady = true;
    } else {
      float leftTraveled  = movement.leftPulseCount * millimetersPerPulse;
      float rightTraveled = movement.rightPulseCount * millimetersPerPulse;

      // Failsafe: si el yaw no avanza (IMU muda/congelada), cerrar por
      // encoders para no girar infinito
      if (fabs(leftTraveled) > fabs(instructionValue) * 1.5f + 20.0f) {
        MessageDebugf("DEBUG: -1, ID: %s, TURN failsafe: yaw estancado "
                      "(%.1f° de %.1f°), cierro por encoders",
                      robotID.c_str(), imuTurnAccumDeg, imuTurnTargetDeg);
        imuTurnActive       = false;
        imuTurnIsCorrection = false;
        imuTurnCorrCount    = 0;
        movementReady       = true;
      } else {
        // Frenar imuTurnBrakeLead antes del objetivo: la inercia completa el
        // giro. PERO en una corrección el arco es chico y no hay momento que
        // costear → lead=0; con lead la corrección se cancelaría a sí misma.
        float lead = imuTurnIsCorrection
                         ? 0.0f
                         : ((imuTurnTargetDeg >= 0) ? imuTurnBrakeLead
                                                    : -imuTurnBrakeLead);
        float remainingArc =
            DegToArc(imuTurnTargetDeg - lead - imuTurnAccumDeg);
        bool reached = MoveDistanceByWheel(leftTraveled + remainingArc,
                                           rightTraveled - remainingArc);
        if (reached) {
          ConfigureHBridge(0, 0);
          imuTurnSettleUntil = millis() + imuTurnSettleMs;
        }
        movementReady = false;
      }
    }
  } else {
    movementReady = MoveDistanceByWheel(instructionValue, -instructionValue);
  }

  if (movementReady) {
    MessageDebugf("DEBUG: -1, ID: %s, Giro completado", robotID.c_str());
    intContext.Clear();
    isEvading = false;
    state = STOP;

  } else if (obstacles.HasAnyObstacle() && !isEvading) {
    obstacles.UpdateBitmap();

    intContext.wasInterrupted = true;
    intContext.previousState = TURN;
    intContext.leftPulsesBeforeStop = movement.pastLeftPulseCount;

    float traveled = movement.pastLeftPulseCount * millimetersPerPulse;
    intContext.remainingValue = instructionValue - traveled;

    isEvading = true;
    evasionStartTime = millis();

    float angleRemaining =
        (intContext.remainingValue / centerToWheelDistance) * RAD_TO_DEG;
    MessageDebugf("DEBUG: -1, ID: %s, TURN interrumpido: restante=%.1f°",
                  robotID.c_str(), angleRemaining);

    if (obstacles.centralObstacle) {
      movementReady = false;
    }

    imuTurnActive = false;
    imuTurnIsCorrection = false;
    imuTurnCorrCount = 0;
    state = STOP;
  }
}

// Camina al azar durante instructionValue ms, encolando tramos que arma
// SelectMovementRW. Al vencerse vuelve a leer la cola.
void StateRandomWalk() {
  if (previousMillisRW == 0) {
    previousMillisRW = millis();
  }

  currentMillis = millis();
  millisDifference = currentMillis - previousMillisRW;
  if (millisDifference < instructionValue) {
    previousMillisRW = currentMillis;
    fsmInstruction[0] = RANDOM_WALK;
    fsmInstruction[1] = instructionValue - millisDifference;
    instructionList.push_front(fsmInstruction);
    SelectMovementRW();
  } else {
    previousMillisRW = 0;
    MessageDebugf("DEBUG: -1, ID: %s, Random Walk terminado",
                  robotID.c_str());
  }

  state = READ_INSTRUCTION;
}

// Retrocede una distancia fija para despegarse de lo que sea que bloquee, antes
// de reintentar.
void StateReverse() {
  if (MotionTimedOut()) { AbortMotion("REVERSE"); return; }
  movementReady = MoveDistanceByWheel(reverseDistance, reverseDistance);

  // 17-oct-2026: el retroceso YA NO se corta por obstáculos. Los sensores (IR
  // laterales y APDS frontal) miran hacia adelante y a los costados: lo que ven
  // es justo aquello de lo que el robot se está alejando. Antes, si el
  // retroceso arrancaba con isEvading = false (descartar un candidato de la
  // búsqueda), el obstáculo de enfrente lo frenaba en el primer ciclo y el robot
  // no podía despegarse nunca: Atta_3 quedó >2 min contra la fila de robots
  // estacionados y Atta_1 pegado al cubo después de encontrarlo.
  if (movementReady) {
    obstacleDetected = true;
    MessageDebugf("DEBUG: -1, ID: %s, Retroceso completado", robotID.c_str());
    state = STOP;
  }
}

// Corta los motores y decide que sigue. Si el movimiento habia terminado bien,
// vuelve a WAIT; si se corto por un obstaculo, va a identificarlo y de paso le
// manda la pose a la Base.
void StateStop() {
  ConfigureHBridge(0, 0);

  if (movementReady == true) {
    if (!intContext.wasInterrupted) {
      isEvading = false;
      resumeScheduled = false;
    }

    state = WAIT;
    instructionValue = instructionCompletedDelay;

  } else {
    SendPose();
    /* 07-nov-2026: re-escanear cuenta como detección NUEVA. Sin esto, si se llega
     * acá desde ACTIVE_EVASION por «datos obsoletos», IDENTIFY_OBSTACLE espera
     * otros 600 ms, ACTIVE_EVASION vuelve a ver el mismo evasionStartTime viejo y
     * manda a STOP otra vez: bucle sin evadir nunca (ver el hallazgo en
     * StateSearchApproach). Los sensores se leen en cada loop(), así que el
     * bitmap de obstáculos ya es actual. */
    evasionStartTime = millis();
    state = IDENTIFY_OBSTACLE;
  }
}

// Saca la proxima instruccion de la cola y salta a su estado. Con la cola vacia
// vuelve a WAIT. El bitmap de obstaculos se limpia solo si no venimos de una
// evasion ni de una instruccion interrumpida, porque en esos casos todavia hace
// falta saber que sensor disparo.
void StateReadInstruction() {
  if (!isEvading && !intContext.wasInterrupted) {

    obstacles.Clear();
  }

  if (!instructionList.empty()) {
    fsmInstruction = instructionList.front();
    instructionList.pop_front();
    instructionValue = fsmInstruction[1];
    state = static_cast<RobotState>(fsmInstruction[0]);
    // Vigilante de movimiento (17-oct-2026): presupuesto de tiempo por instrucción.
    if (state == MOVE || state == REVERSE || state == TURN) {
      motionStartMs  = millis();
      motionBudgetMs = (state == TURN)
          ? MOTION_TURN_BASE_MS + (unsigned long)(fabs(instructionValue) * MOTION_TURN_MS_PER_MM)
          : MOTION_MOVE_BASE_MS + (unsigned long)(fabs(instructionValue) * MOTION_MOVE_MS_PER_MM);
    } else {
      motionBudgetMs = 0;
    }
    direction = instructionValue > 0 ? '+' : '-';
    // Sonda: qué valor sale realmente de la cola (2026-07-29, TURN con
    // objetivo=0°). Solo con debug activo; MOVE=1 TURN=2 WAIT=3.
    if (state == TURN || state == MOVE) {
      MessageDebugf("DEBUG: -1, ID: %s, READ: estado=%d valor=%.2f (quedan %d)",
                    robotID.c_str(), (int)state, instructionValue,
                    (int)instructionList.size());
    }

  } else {
    state = WAIT;
    instructionValue = instructionCompletedDelay;
  }
}

// Le avisa a la Base que termino la secuencia de instrucciones.
void StateMessageBase() {
  const char *message = "";
  if (instructionValue == 1) {
    message = "READY";
  }

  SendMessage(robots["Base"], message);
  state = WAIT;
  instructionValue = instructionCompletedDelay;
}

// Decide si lo que bloquea es otro robot o un obstaculo del escenario. Espera
// obstacleWaitTime a que llegue un OBSTACLE_DETECTED de algun companero: si
// llega, alcanza con esperar a que se aparte; si no, es fijo y hay que evadir.
void SearchTrace(const char *fmt, ...);
void StateIdentifyObstacle() {
  currentMillis = millis();
  // 17-oct-2026: tres robots alrededor del cubo se identificaban MUTUAMENTE como
  // «el obstáculo es un robot» (cada uno ve a otro del otro lado) y esperaban
  // para siempre a que el otro se moviera, con CHECK_OBSTACLE a 10 Hz. Si la
  // respuesta «es un robot» se repite IDENT_ROBOT_MAX veces sin pausa, se trata
  // como obstáculo fijo: evasión normal, y en búsqueda, candidato.
  static int identRobotSeguidas = 0;
  static unsigned long identRobotUltima = 0;
  if (obstacles.robotDetected && search.active && searchCandAunqueRobot &&
      obstacles.centralObstacle) {
    SearchTrace("central con robot %s cerca: se revisa igual", obstacles.fromRobotID.c_str());
    obstacles.robotDetected = false;
    movement.previousMillis = currentMillis;
    state = ACTIVE_EVASION;            // ahí se vuelve candidato (o se evade si ya se descartó)
    instructionValue = 0;
    return;
  }
  // 05-nov-2026: negociación de paso. El otro dijo que no se va a mover (QUIETO)
  // o que tiene el paso (SIGO): esperarlo es perder el tiempo, se evade ya.
  if (obstacles.robotDetected && pasoNegociado &&
      (obstacles.robotPaso == 'Q' || obstacles.robotPaso == 'S')) {
    if (search.active) SearchTrace("robot %s %s: evado sin esperar", obstacles.fromRobotID.c_str(),
                                   NombrePaso(obstacles.robotPaso));
    MessageDebugf("DEBUG: -1, ID: %s, PASO: robot %s %s, evado sin esperar",
                  robotID.c_str(), obstacles.fromRobotID.c_str(), NombrePaso(obstacles.robotPaso));
    identRobotSeguidas = 0;
    obstacles.robotDetected = false;
    obstacles.robotPaso = 0;
    movement.previousMillis = currentMillis;
    state = ACTIVE_EVASION;
    instructionValue = 0;
    return;
  }
  if (obstacles.robotDetected) {
    if (currentMillis - identRobotUltima > IDENT_ROBOT_WINDOW_MS) identRobotSeguidas = 0;
    identRobotUltima = currentMillis;
    if (++identRobotSeguidas >= IDENT_ROBOT_MAX) {
      identRobotSeguidas = 0;
      obstacles.robotDetected = false;
      movement.previousMillis = currentMillis;
      state = ACTIVE_EVASION;
      instructionValue = 0;
      MessageDebugf("DEBUG: -1, ID: %s, %d esperas seguidas a un robot: se trata como obstáculo fijo",
                    robotID.c_str(), IDENT_ROBOT_MAX);
      return;
    }
    state = WAIT;
    instructionValue = instructionCompletedDelay / 2;
    if (search.active) SearchTrace("es robot %s (vez %d)", obstacles.fromRobotID.c_str(), identRobotSeguidas);
    MessageDebugf("DEBUG: -1, ID: %s, Obstáculo encontrado es robot id: %s",
                  robotID.c_str(), obstacles.fromRobotID.c_str());
  } else if ((currentMillis - movement.previousMillis) >= obstacleWaitTime) {
    movement.previousMillis = currentMillis;
    state = ACTIVE_EVASION;
    instructionValue = 0;
    MessageDebugf("DEBUG: -1, ID: %s, Obstáculo encontrado no es un robot",
                  robotID.c_str());
  }
}

// Le pide la pose a la Base y espera el POSITION_RESPONSE para dar el siguiente
// paso de navegacion. Sin IP de base aborta, y si la respuesta no llega a
// tiempo cae a la pose del EKF y sigue navegando a ciegas, porque perder de
// vista el marcador un rato no puede abortar la corrida entera.
void StateRequestPosition() {
  if (robots.find("Base") == robots.end() ||
      robots["Base"] == IPAddress(0, 0, 0, 0)) {
    MessageDebugf("DEBUG: -1, ID: %s, No hay IP de base — abortando nav",
                  robotID.c_str());
    congregation.CompleteRequest();
    state = WAIT;
    instructionValue = 500;
    return;
  }

  if (!congregation.waitingForResponse) {
    SendMessage(robots["Base"], "REQUEST_POSITION");
    MessageDebugf("DEBUG: -1, ID: %s, Solicitud enviada: REQUEST_POSITION",
                  robotID.c_str());
    congregation.StartRequest();
  }

  if (congregation.HasTimedOut()) {
    congregation.CompleteRequest();

    // La cámara no contestó: seguir con la estimación propia mientras el filtro
    // esté sano y su incertidumbre siga siendo utilizable. Es lo que evita que
    // una oclusión de dos segundos deje al robot plantado el resto de la prueba.
    if (ekf.initialized && ekf.IsHealthy() &&
        blindNavSteps < BLIND_NAV_MAX_STEPS &&
        ekf.PosSigma() < BLIND_NAV_MAX_SIGMA) {
      blindNavSteps++;
      robotPose.x = ekf.x;
      robotPose.y = ekf.y;
      robotPose.angle = ekf.AngleDeg();
      congregation.poseFresh = true;

      char buffer[110];
      snprintf(buffer, sizeof(buffer),
               "BLIND_NAV|%d|%d|%.0f|%.0f|%.0f|%.0f", blindNavSteps,
               BLIND_NAV_MAX_STEPS, robotPose.x, robotPose.y, robotPose.angle,
               ekf.PosSigma());
      SendMessage(robots["Base"], buffer);
      MessageDebugf("DEBUG: -1, ID: %s, Timeout de cámara — navegando con EKF "
                    "(%d/%d, σ=%.0fmm)",
                    robotID.c_str(), blindNavSteps, BLIND_NAV_MAX_STEPS,
                    ekf.PosSigma());

      // Mismo encadenado que HandlePositionResponse cuando sí llega la pose:
      // recalcular el slot, dar el paso de navegación y recién ahí pasar a
      // consumir las instrucciones que ese paso dejó en la lista.
      if (congregation.hasGlobalTarget && !congregation.isLeader) {
        UpdateCongregationGoal(congregation.globalTargetX,
                               congregation.globalTargetY, 0.0f);
      }
      if (nav.isActive) {
        ReactiveNavStep();
      } else if (nav.pendingInit) {
        nav.pendingInit = false;
        nav.Start(nav.goalX, nav.goalY);
        ReactiveNavStep();
      }
      state = READ_INSTRUCTION;
      instructionValue = 0;
      return;
    }

    // BDI-E: sin EKF utilizable, reintentar la pregunta un número ACOTADO de
    // veces antes de rendirse: una sola respuesta perdida no puede matar el
    // movimiento en curso.
    if ((nav.isActive || nav.pendingInit) &&
        ++navRequestRetries <= kMaxNavRequestRetries) {
      fsmInstruction[0] = REQUEST_POSITION;
      fsmInstruction[1] = 0;
      instructionList.push_front(fsmInstruction);
      MessageDebugf("DEBUG: -1, ID: %s, Reintentando REQUEST_POSITION (nav %d/%d)",
                    robotID.c_str(), navRequestRetries, kMaxNavRequestRetries);
      state = READ_INSTRUCTION;
      instructionValue = 0;
      return;
    }
    navRequestRetries = 0;
    // BDI-E: en misión, soltar la navegación. Si quedara nav.isActive en alto
    // el agente no volvería a deliberar hasta el guardián de 3 min; así la
    // intención caduca por su propio reloj y el agente replanifica.
    if (missionActive) BdieReleaseBody();

    // Se agotó el respaldo: detenerse y decir por qué, en vez de seguir a ciegas
    // con una estimación en la que ya no se puede confiar.
    char buffer[120];
    snprintf(buffer, sizeof(buffer),
             "WARNING: nav detenida sin pose. EKF %s, pasos a ciegas %d/%d, "
             "σ=%.0fmm",
             (ekf.initialized && ekf.IsHealthy()) ? "sano" : "no disponible",
             blindNavSteps, BLIND_NAV_MAX_STEPS,
             ekf.initialized ? ekf.PosSigma() : 0.0f);
    SendMessage(robots["Base"], buffer);
    MessageDebugf("DEBUG: -1, ID: %s, %s", robotID.c_str(), buffer);
    state = WAIT;
    instructionValue = 500;
    return;
  }

  if (congregation.positionReceived) {
    MessageDebugf("DEBUG: -1, ID: %s, Posición recibida",
                  robotID.c_str());
    congregation.CompleteRequest();
    congregation.positionReceived = false;
    blindNavSteps = 0;
    navRequestRetries = 0;          // BDI-E: la base contestó, cuenta a cero

    // nav activo: obstáculo ya en obstacles struct, ReactiveNavStep lo procesará
    state = READ_INSTRUCTION;
    instructionValue = 0;
  }
}

// Maniobra de evasion. Elige el rodeo segun el patron de bits de los sensores y
// encola retroceso, giro y avance. Tras varias evasiones seguidas en el mismo
// punto asume deadlock y hace un escape comprometido hacia el interior de la
// arena, en vez del giro de 180° a ciegas que dejaba al robot rebotando contra
// la misma esquina.
/* TRAZA DE LA BÚSQUEDA (24-oct-2026). En la corrida de las 13:52 el enjambre
 * cubrió el 99 % de la arena y pasó varias veces junto al cubo sin leerlo; los
 * mensajes de la búsqueda eran DEBUG (no llegan al log) y no había forma de
 * saber en qué paso se perdía. Cada decisión de la búsqueda va ahora a la base
 * como «SEARCH|...», a lo sumo unas pocas por candidato. */
void SearchTrace(const char *fmt, ...) {
  if (robots.find("Base") == robots.end()) return;
  char cuerpo[100];
  va_list ap; va_start(ap, fmt); vsnprintf(cuerpo, sizeof(cuerpo), fmt, ap); va_end(ap);
  char buf[128];
  snprintf(buf, sizeof(buf), "SEARCH|%s|(%.0f,%.0f,%.0f)|%s", robotID.c_str(),
           robotPose.x, robotPose.y, robotPose.angle, cuerpo);
  SendMessage(robots["Base"], buf);
}

void StateActiveEvasion() {
  if (!obstacles.HasAnyObstacle()) {
    // Sin obstáculo real — puede haber desaparecido entre detección y aquí
    MessageDebugf(
        "DEBUG: -1, ID: %s, ACTIVE_EVASION sin obstáculo. Abortando.",
        robotID.c_str());
    obstacles.Clear();
    state = READ_INSTRUCTION;
    return;
  }

  // Si solo disparó el sensor frontal APDS9960 (sin IR laterales),
  // normalizarlo como obstáculo central para el pattern matching.
  if (obstacles.obstacleSensors == 0 && obstacles.centralObstacle) {
    obstacles.obstacleSensors = 0b010;
  }

  // SEARCH activo: un obstáculo central es un CANDIDATO — aproximarse y
  // leerle el color en vez de evadir (los laterales evaden normal)
  // Memoria de candidatos descartados (17-oct-2026): lo que se descartó hace
  // poco en el mismo lugar no se vuelve a revisar — se evade normal. Sin esto
  // un objeto que no es el objetivo y no refleja (otro robot, oscuro: proximidad
  // 65 al tocarlo) se revisaba en bucle sin fin.
  const bool yaDescartado =
      (millis() - searchRejectMs) < SEARCH_REJECT_MEMORY_MS &&
      CalculateDistance(robotPose.x, robotPose.y, searchRejectX, searchRejectY) < SEARCH_REJECT_RADIUS_MM;
  if (search.active) {
    SearchTrace("evasion c=%d l=%d r=%d ir=%d%d%d descartado=%d",
                obstacles.centralObstacle, obstacles.leftObstacle, obstacles.rightObstacle,
                (obstacles.obstacleSensors >> 2) & 1, (obstacles.obstacleSensors >> 1) & 1,
                obstacles.obstacleSensors & 1, yaDescartado);
  }
  if (search.active && obstacles.centralObstacle && !yaDescartado) {
    instructionList.clear();
    obstacleDetected = false;
    isEvading = false;
    resumeScheduled = false;
    intContext.Clear();
    obstacles.Clear();
    search.BeginApproach(millis());
    ConfigureHBridge(0, 0);                       // avanza por pasos con PID, ver abajo
    SearchTrace("candidato prox=%u", frontSensor.readProximity());
    state = SEARCH_APPROACH;
    return;
  }

  // SEARCH activo y contacto SOLO lateral (12-oct-2026): antes se evadía sin
  // mirar, y como el sensor de color ve a pocos cm el objeto que tocaba un
  // costado no se revisaba nunca. Ahora se gira hacia ese lado y se avanza un
  // tramo corto: si era el objeto, el APDS9960 central lo ve y el flujo de
  // arriba lo convierte en candidato en la próxima detección. Se evade normal
  // si el punto de contacto cae fuera de la arena (es la pared) o si ya hubo
  // un sondeo hace menos de LATERAL_PROBE_COOLDOWN_MS (no rebotar en lo mismo).
  if (search.active && !obstacles.centralObstacle &&
      (obstacles.leftObstacle != obstacles.rightObstacle)) {
    const bool  izq  = obstacles.leftObstacle;
    const float lado = izq ? -1.0f : 1.0f;      // TURN > 0 gira a la derecha
    const float th   = DegToRad(robotPose.angle + lado * 50.0f);
    const float cx   = robotPose.x + 130.0f * cos(th);
    const float cy   = robotPose.y + 130.0f * sin(th);
    const bool  esPared = (cx < 0 || cx > arenaWidthMm || cy < 0 || cy > arenaHeightMm);
    const bool  mismoLugar = CalculateDistance(robotPose.x, robotPose.y,
                                               lastLateralProbeX, lastLateralProbeY) < LATERAL_PROBE_SAME_MM;
    const bool  enfriando = (millis() - lastLateralProbeMs) < LATERAL_PROBE_COOLDOWN_MS &&
                            !(mismoLugar && lateralProbeSeguidos < 2);
    if (!esPared && !enfriando) {
      lateralProbeSeguidos = mismoLugar ? lateralProbeSeguidos + 1 : 1;
      lastLateralProbeX = robotPose.x; lastLateralProbeY = robotPose.y;
      lastLateralProbeMs = millis();
      instructionList.clear();
      obstacleDetected = false;
      isEvading = false;
      resumeScheduled = false;
      intContext.Clear();
      obstacles.Clear();
      evasionTracker.Reset();
      fsmInstruction[0] = REVERSE;                 // despegarse lo justo para poder girar
      fsmInstruction[1] = LATERAL_PROBE_REVERSE_MM;
      instructionList.push_back(fsmInstruction);
      for (int k = 0; k < LATERAL_PROBE_STEPS; k++) {   // barrido fino: el central interrumpe
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = DegToArc(lado * LATERAL_PROBE_STEP_DEG);
        instructionList.push_back(fsmInstruction);
        fsmInstruction[0] = WAIT;
        fsmInstruction[1] = LATERAL_PROBE_PAUSE_MS;
        instructionList.push_back(fsmInstruction);
      }
      fsmInstruction[0] = TURN;                    // nada en 60°: al centro del barrido
      fsmInstruction[1] = DegToArc(-lado * LATERAL_PROBE_STEP_DEG * LATERAL_PROBE_STEPS / 2);
      instructionList.push_back(fsmInstruction);
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = LATERAL_PROBE_MM;
      instructionList.push_back(fsmInstruction);
      if (nav.isActive) {                          // sin objeto: retomar la ruta
        fsmInstruction[0] = REQUEST_POSITION;
        fsmInstruction[1] = 0;
        instructionList.push_back(fsmInstruction);
      }
      SearchTrace("sondeo lateral %s (barrido %dx%.0f, seguidos %d)", izq ? "izq" : "der",
                  LATERAL_PROBE_STEPS, LATERAL_PROBE_STEP_DEG, lateralProbeSeguidos);
      state = READ_INSTRUCTION;
      return;
    }
  }

  unsigned long timeSinceDetection = millis() - evasionStartTime;
  if (timeSinceDetection > 1500) {
    MessageDebugf("DEBUG: -1, ID: %s, Datos de obstáculo obsoletos (%lums). "
                  "Re-escaneando.",
                  robotID.c_str(), timeSinceDetection);
    state = STOP;
    return;
  }

  evasionTracker.RecordEvasion();

  int avoidanceDistance = 0;
  int avoidanceAngle = 0;
  bool needsRetreat = evasionTracker.ShouldRetreat();

  if (needsRetreat) {
    MessageDebugf(
        "DEBUG: -1, ID: %s, Ejecutando retroceso forzado (evasiones: %d)",
        robotID.c_str(), evasionTracker.consecutiveEvasions);

    std::deque<std::array<float, 2>> retreatSequence;

    fsmInstruction[0] = REVERSE;
    fsmInstruction[1] = reverseDistance * 3;
    retreatSequence.push_back(fsmInstruction);

    // Lado del rodeo hacia el INTERIOR de la arena (no ciego): proyecta el
    // tramo por cada lado y toma el que queda más adentro (lejos de paredes).
    // Corrige el thrash en esquina; espeja escape_turn() de la sim.
    float escAng   = escapeTurnDeg;
    float bestScore = -1.0e9f;
    for (int s = -1; s <= 1; s += 2) {
      float th = DegToRad(robotPose.angle + s * escapeTurnDeg);
      float ex = robotPose.x + escapeMoveMm * cos(th);
      float ey = robotPose.y + escapeMoveMm * sin(th);
      float score = min(min(ex, arenaWidthMm - ex),
                        min(ey, arenaHeightMm - ey));
      if (score > bestScore) {
        bestScore = score;
        escAng    = s * escapeTurnDeg;
      }
    }

    MessageDebugf("DEBUG: -1, ID: %s, Escape de deadlock: rodeo %.0f° hacia interior",
                  robotID.c_str(), escAng);

    fsmInstruction[0] = TURN;
    fsmInstruction[1] = DegToArc(escAng);
    retreatSequence.push_back(fsmInstruction);

    fsmInstruction[0] = MOVE;
    fsmInstruction[1] = escapeMoveMm;
    retreatSequence.push_back(fsmInstruction);

    for (auto it = retreatSequence.rbegin(); it != retreatSequence.rend();
         ++it) {
      instructionList.push_front(*it);
    }

    evasionTracker.Reset();

  } else {
    if (obstacles.obstacleSensors == 0b100) {
      avoidanceAngle = 45;
      avoidanceDistance = 120;
    } else if (obstacles.obstacleSensors == 0b001) {
      avoidanceAngle = -45;
      avoidanceDistance = 120;
    } else if (obstacles.obstacleSensors == 0b010) {
      avoidanceAngle = (random(2) == 0) ? 60 : -60;
      avoidanceDistance = 150;
    } else if (obstacles.obstacleSensors == 0b110) {
      avoidanceAngle = 90;
      avoidanceDistance = 150;
    } else if (obstacles.obstacleSensors == 0b011) {
      avoidanceAngle = -90;
      avoidanceDistance = 150;
    } else if (obstacles.obstacleSensors == 0b111) {
      avoidanceAngle = (random(2) == 0) ? 135 : -135;
      avoidanceDistance = 100;
    }

    if (!obstacles.HasAnyObstacle()) {
      MessageDebugf("DEBUG: -1, ID: %s, Obstáculo desapareció durante "
                    "cálculo de evasión",
                    robotID.c_str());
      obstacles.Clear();
      state = READ_INSTRUCTION;
      return;
    }

    std::deque<std::array<float, 2>> evasionSequence;

    if (obstacles.obstacleSensors & 0b010 ||
        obstacles.obstacleSensors == 0b111) {
      fsmInstruction[0] = REVERSE;
      fsmInstruction[1] = reverseDistance * 1.5;
      evasionSequence.push_back(fsmInstruction);
    }

    if (avoidanceAngle != 0) {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = DegToArc(avoidanceAngle);
      evasionSequence.push_back(fsmInstruction);
    }

    if (avoidanceDistance > 0) {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = avoidanceDistance;
      evasionSequence.push_back(fsmInstruction);
    }

    for (auto it = evasionSequence.rbegin(); it != evasionSequence.rend();
         ++it) {
      instructionList.push_front(*it);
    }
  }

  if (intContext.wasInterrupted && !resumeScheduled) {
    fsmInstruction[0] = RESUME_AFTER_EVASION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
    resumeScheduled = true;
  }

  obstacleDetected = false;

  state = READ_INSTRUCTION;

  MessageDebugf("DEBUG: -1, ID: %s, Evasión: patrón=%s, giro=%d°, "
                "avance=%dmm, forzado=%d",
                robotID.c_str(), obstacles.GetObstaclePattern().c_str(),
                avoidanceAngle, avoidanceDistance, needsRetreat);
}

// Retoma la instruccion que la evasion interrumpio, con el resto que quedo
// guardado en intContext, y limpia el rastro de la evasion.
void StateResumeAfterEvasion() {
  if (!intContext.wasInterrupted) {
    resumeScheduled = false;
    obstacles.Clear();
    state = READ_INSTRUCTION;
    return;
  }

  MessageDebugf(
      "DEBUG: -1, ID: %s, Resumiendo: estado=%d, valor=%.1f",
      robotID.c_str(), intContext.previousState, intContext.remainingValue);

  if (nav.isActive) {
    // Navegación activa: re-solicitar posición para recalcular ruta en vez
    // de terminar a ciegas el segmento interrumpido (nav.isActive faltaba
    // tras la migración a ReactiveNav — causaba desvíos largos al evadir)
    MessageDebugf(
        "DEBUG: -1, ID: %s, GT activo: solicitando posición post-evasión",
        robotID.c_str());
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_front(fsmInstruction);

  } else {
    switch (intContext.previousState) {
    case MOVE: {
      if (intContext.remainingValue > 20) {
        fsmInstruction[0] = MOVE;
        fsmInstruction[1] = intContext.remainingValue;
        instructionList.push_front(fsmInstruction);

        MessageDebugf("DEBUG: -1, ID: %s, Reanudando MOVE: %.1fmm restantes",
                      robotID.c_str(), intContext.remainingValue);
      }
      break;
    }

    case TURN: {
      float angleRemaining = abs(
          (intContext.remainingValue / centerToWheelDistance) * RAD_TO_DEG);
      if (angleRemaining > 5) {
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = intContext.remainingValue;
        instructionList.push_front(fsmInstruction);

        MessageDebugf("DEBUG: -1, ID: %s, Reanudando TURN: %.1f° restantes",
                      robotID.c_str(), angleRemaining);
      }
      break;
    }
    }
  }

  intContext.Clear();
  resumeScheduled = false;
  isEvading = false;
  obstacles.Clear();

  evasionTracker.Reset();

  state = READ_INSTRUCTION;
}

// Aproximacion lenta al candidato hasta el alcance del APDS9960, unos pocos cm,
// que es donde la lectura de color es confiable. Al llegar se detiene, espera
// una lectura valida y clasifica; si el color no era el buscado o no alcanza a
// llegar, evade y vuelve a patrullar. Validado en Webots.
void StateSearchApproach() {
  /* 24-oct: entrada desde la cola de instrucciones ({SEARCH_APPROACH, -1}), la
   * usa la revisión de la meta cuando se corta un viaje (bdie.ino). Arranca el
   * acercamiento con los contadores en cero. */
  if (instructionValue < 0) {
    instructionValue = 0;
    search.BeginApproach(millis());
    ConfigureHBridge(0, 0);
    SearchTrace("candidato por viaje cortado prox=%u", frontSensorInitialized ? frontSensor.readProximity() : 0);
  }
  if (!search.active || !frontSensorInitialized) {
    ConfigureHBridge(0, 0);
    // BDI-E: la búsqueda se desarmó a mitad del acercamiento (otro robot
    // anunció el objetivo): devolver el control a la navegación en curso.
    if (missionActive) BdieResumeAfterCandidate();
    state = READ_INSTRUCTION;
    return;
  }

  if (millis() - search.approachStart > SEARCH_APPROACH_TIMEOUT) {
    ConfigureHBridge(0, 0);
    SearchTrace("agotada %lu ms pasos=%d mejor=%u", SEARCH_APPROACH_TIMEOUT, search.steps, search.bestProx);
    SearchEvadeAndResume();
    return;
  }

  // ¿Ya se puede leer? Ver la nota de SEARCH_PROX_NEAR en AttaBot.ino.
  const uint8_t prox = frontSensor.readProximity();
  const bool leer =
      (prox >= SEARCH_PROX_NEAR) ||
      (prox >= SEARCH_PROX_MIN_READ &&
       (search.stalls >= 2 || search.steps >= SEARCH_MAX_STEPS));
  if (!leer) {
    /* 24-oct-2026 (corrida 15:06): el Atta_2 apuntaba al cubo a 136 mm, la
     * proximidad se quedó en 3 dos pasos seguidos (todavía fuera del alcance del
     * APDS9960, que ve a pocos cm) y se descartó como «tope sin proximidad»: el
     * cubo quedó marcado 30 s y lo esquivó. «Sin subir» no es «tocando»: ahora
     * se descarta así solo si las RUEDAS no avanzaron dos pasos seguidos (tope
     * físico contra algo que no refleja, p. ej. un robot oscuro). Si avanza y no
     * hay nada, el corte es SEARCH_MAX_STEPS (8 × 20 mm = 160 mm). */
    if (search.blocked >= 2 && prox < SEARCH_PROX_MIN_READ) {   // tocando algo que no refleja
      ConfigureHBridge(0, 0);
      SearchTrace("tope sin proximidad prox=%u pasos=%d -> descartado", prox, search.steps);
      SearchEvadeAndResume();
      return;
    }
    if (search.steps >= SEARCH_MAX_STEPS) {      // avanzó todo y no hay nada cerca
      ConfigureHBridge(0, 0);
      SearchTrace("%d pasos sin alcance prox=%u -> descartado", search.steps, prox);
      SearchEvadeAndResume();
      return;
    }
    if (!search.stepActive) {                    // paso nuevo: contadores a cero
      ResetPID();
      search.stepActive = true;
      search.stepStart = millis();
    }
    bool fin = MoveDistanceByWheel(SEARCH_STEP_MM, SEARCH_STEP_MM);
    if (!fin && millis() - search.stepStart < SEARCH_STEP_MAX_MS) return;
    ConfigureHBridge(0, 0);
    search.stepActive = false;
    search.steps++;
    if (fin) search.blocked = 0; else search.blocked++;   // !fin = se agotó el tiempo sin completar el paso
    if (prox > search.bestProx + 2) { search.bestProx = prox; search.stalls = 0; }
    else                            { search.stalls++; }
    SearchTrace("paso %d prox=%u mejor=%u sin_subir=%d trabado=%d", search.steps, prox, search.bestProx, search.stalls, search.blocked);
    return;
  }

  // Al alcance: detenerse y esperar una lectura de color válida (~100ms)
  ConfigureHBridge(0, 0);
  if (!frontSensor.colorDataReady()) {
    return;
  }
  uint16_t r, g, b, c;
  frontSensor.getColorData(&r, &g, &b, &c);
  bool match = MatchColor(search.targetColor, r, g, b, c);
  {  // 17-oct: el resultado de cada candidato va SIEMPRE a la base, no solo en DEBUG
    char cb[110];
    snprintf(cb, sizeof(cb), "SEARCH_COLOR: prox=%u pasos=%d R=%u G=%u B=%u C=%u -> %s",
             prox, search.steps, r, g, b, c, match ? search.targetColor : "no coincide");
    SendMessage(robots["Base"], cb);
  }
  MessageDebugf("DEBUG: -1, ID: %s, SEARCH: RGBC=%u,%u,%u,%u → %s",
                robotID.c_str(), r, g, b, c,
                match ? search.targetColor : "no coincide");

  if (match) {
    // El objeto está ~100mm frente al robot
    float ox = robotPose.x + 100.0f * cos(DegToRad(robotPose.angle));
    float oy = robotPose.y + 100.0f * sin(DegToRad(robotPose.angle));
    char buf[80];
    snprintf(buf, sizeof(buf), "OBJECT_FOUND|%s|%.0f|%.0f|%s",
             robotID.c_str(), ox, oy, search.targetColor);
    SendMessage(robots["Base"], buf);
    MessageDebugf("DEBUG: -1, ID: %s, OBJETO %s ENCONTRADO en (%.0f,%.0f)",
                  robotID.c_str(), search.targetColor, ox, oy);
    search.Reset();
    frontSensor.enableColor(false);
    instructionList.clear();
    /* 07-nov-2026: el descubridor tiene el objetivo PEGADO al frente (prox 255).
     * Con movementReady en false, StateStop lo tomaba por obstáculo y entraba en
     * bucle: CHECK_OBSTACLE → IDENTIFY_OBSTACLE → ACTIVE_EVASION con un
     * evasionStartTime viejo (> 1,5 s, «datos obsoletos») → STOP → … sin volver
     * nunca a DELIBERATE. El agente no se marcaba llegado y los demás esperaban
     * el temporizador de 5 min (07-nov 13:22: Atta_1 > 2 min sin salir; 24-oct
     * 16:21: Atta_3, 34 s). El movimiento termina limpio: STOP → WAIT → DELIBERATE,
     * y ahí el agente (iFoundIt) cuenta como llegado y espera junto al objetivo. */
    movementReady = true;
    isEvading = false;
    resumeScheduled = false;
    intContext.Clear();
    obstacles.Clear();
    if (missionActive) BdieTargetConfirmed();   // BDI-E: hallazgo → agente → TGT
    ledCtrl.setSolid(0, 255, 0, maxBrightness);
    state = STOP;
  } else {
    SearchEvadeAndResume();
  }
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

// Housekeeping de cada ciclo y despacho del estado en curso.
void loop() {
  // BDI-E: período real de loop(), para GET_STATUS
  {
    static uint32_t tLoopPrev = 0;
    const uint32_t  tLoopNow  = micros();
    if (tLoopPrev) loopPeriodUs = tLoopNow - tLoopPrev;
    tLoopPrev = tLoopNow;
  }

  ledCtrl.update();
  WiFiStatus();
  if (WiFi.status() != WL_CONNECTED)
    return;
  ReadUdpPackets();
  ReadSensors();
  SetupFrontSensor();

  // Lectura IMU no bloqueante — se ejecuta solo si la IMU está disponible
  // y han pasado al menos imuReadInterval ms desde la última lectura.
  if (imuAvailable && (millis() - lastImuRead >= imuReadInterval)) {
    lastImuRead = millis();
    LeerYaw();
    EkfTick();
  }

  // Telemetría del EKF (2Hz). Es solo salida: no toca el control, corra o no
  // EKF_NAV. Sirve para responder con datos "¿cuánto deriva el EKF?" durante las
  // corridas que ya se hacen, en vez de tener que confiar en él sin medirlo.
  if (ekf.initialized && millis() - lastEkfReport >= ekfReportInterval &&
      robots.find("Base") != robots.end() &&
      robots["Base"] != IPAddress(0, 0, 0, 0)) {
    lastEkfReport = millis();
    char ekfBuf[64];
    snprintf(ekfBuf, sizeof(ekfBuf), "EKF_POSE|%.1f|%.1f|%.1f", ekf.x, ekf.y,
             ekf.AngleDeg());
    SendMessage(robots["Base"], ekfBuf);
  }

#ifdef DebugSerial
  ReadSerialCommands();
#endif

  BdieLoopHook();   // BDI-E: guardián de nav, latido y búsqueda por color
  LatTick();        // OE3: ping robot↔robot si NAV_CONFIG|LAT lo encendió

  switch (state) {
  case WAIT:
    StateWait();
    break;
  case MOVE:
    StateMove();
    break;
  case TURN:
    StateTurn();
    break;
  case RANDOM_WALK:
    StateRandomWalk();
    break;
  case REVERSE:
    StateReverse();
    break;
  case STOP:
    StateStop();
    break;
  case READ_INSTRUCTION:
    StateReadInstruction();
    break;
  case MESSAGE_BASE:
    StateMessageBase();
    break;
  case IDENTIFY_OBSTACLE:
    StateIdentifyObstacle();
    break;
  case REQUEST_POSITION:
    StateRequestPosition();
    break;
  case ACTIVE_EVASION:
    StateActiveEvasion();
    break;
  case RESUME_AFTER_EVASION:
    StateResumeAfterEvasion();
    break;
  case SEARCH_APPROACH:
    StateSearchApproach();
    break;
  case DELIBERATE:          // BDI-E (punto 2): el ciclo de razonamiento
    StateDeliberate();
    break;
  }
}

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

// Sortea el proximo tramo del random walk: girar a un lado, al otro o avanzar,
// con las probabilidades y las tablas de angulos y distancias de arriba. Si hay
// un obstaculo detectado, la probabilidad de avanzar se pone en cero.
void SelectMovementRW() {
  int probabilityTurnPos = 15;
  int probabilityMove = 70 * (obstacleDetected ? 0 : 1);
  int probabilityTurnNeg = 15;
  int totalProbabilities =
      probabilityTurnPos + probabilityMove + probabilityTurnNeg;
  std::array<int, 3> cumulativeProbabilities = {
      probabilityTurnPos, probabilityTurnPos + probabilityMove,
      totalProbabilities};
  obstacleDetected = false;

  int randomSelection = random(totalProbabilities);
  int directionRW;
  if (randomSelection < cumulativeProbabilities[0]) {
    directionRW = TURN_POS;
  } else if (randomSelection < cumulativeProbabilities[1]) {
    directionRW = MOVE_FORWARD;
  } else {
    directionRW = TURN_NEG;
  }

  int angle = possibleAngles[random(possibleAngles.size() * 10) %
                             possibleAngles.size()];
  int distance = possibleAdvances[random(possibleAdvances.size() * 10) %
                                  possibleAdvances.size()];
  switch (directionRW) {
  case TURN_POS: {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = DegToArc(angle);
    break;
  }

  case MOVE_FORWARD: {
    fsmInstruction[0] = MOVE;
    fsmInstruction[1] = distance;
    break;
  }

  case TURN_NEG: {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = DegToArc(-angle);
    break;
  }
  }

  instructionList.push_front(fsmInstruction);
}

#ifdef DebugSerial
// Consola serie de diagnostico, solo con DebugSerial activo.
void ReadSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();

    int separatorIndex = command.indexOf('|');
    String cmd = command.substring(0, separatorIndex);
    String valueStr = command.substring(separatorIndex + 1);
    int value = valueStr.toInt();

    if (cmd == "MOVE") {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = value;
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando MOVE %d mm agregado\n", value);

    } else if (cmd == "TURN") {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = DegToArc(value);
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando TURN %d grados agregado — arco=%.2fmm\n", value,
                    fsmInstruction[1]);

    } else if (cmd == "STOP") {
      instructionList.clear();
      ConfigureHBridge(0, 0);
      state = STOP;
      Serial.println("Robot detenido");

    } else {
      Serial.println("Comandos: MOVE|valor, TURN|valor, STOP");
    }
  }
}
#endif


// ============================================================================
// VIGILANTE DE MOVIMIENTO (17-oct-2026)
// ============================================================================
// MoveDistanceByWheel solo termina si las ruedas llegan o si, HABIENDO girado,
// quedan quietas 800 ms. Una rueda trabada desde el arranque (robot contra la
// fila de robots estacionados, Atta_3 a las 12:51 y 13:03) no cumple ninguna de
// las dos: MOVE o TURN no terminaban nunca y el agente no volvía a deliberar.
// Cada MOVE/REVERSE/TURN tiene ahora un presupuesto de tiempo proporcional a lo
// que pide; si se agota, se corta, se avisa a la base y se suelta el cuerpo.
bool MotionTimedOut() {
  return motionBudgetMs != 0 && (millis() - motionStartMs) > motionBudgetMs;
}

void AbortMotion(const char *que) {
  ConfigureHBridge(0, 0);
  const unsigned long dt = millis() - motionStartMs;
  motionBudgetMs = 0;
  imuTurnActive = false;
  imuTurnIsCorrection = false;
  imuTurnCorrCount = 0;
  imuTurnSettleUntil = 0;
  instructionList.clear();
  intContext.Clear();
  isEvading = false;
  resumeScheduled = false;
  obstacles.Clear();
  char buf[90];
  snprintf(buf, sizeof(buf), "WARNING: %s sin terminar en %lu ms (rueda trabada?) - se corta",
           que, dt);
  SendMessage(robots["Base"], buf);
  if (missionActive) BdieReleaseBody();
  movementReady = true;
  state = WAIT;
  instructionValue = instructionCompletedDelay;
}
