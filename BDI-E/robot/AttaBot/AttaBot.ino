// Firmware del Atta-Bot (GIRoM, TEC) con la arquitectura BDI-E integrada.
// Las líneas que agrega BDI-E están marcadas «BDI-E»; el acople completo está
// en bdie.ino.
//
// Versiones de las librerías con que se compila, sobre ESP32 Arduino core
// 3.3.11. Hay que actualizarlas al cambiar una librería, para que un cambio de
// librería no se confunda con una falla de hardware.
//
// No se usa ESP32Servo: el PWM de un servo (50 Hz, 10 bits) ocupaba el mux de
// reloj del LEDC y dejaba sin PWM a los motores. Para volver a usar un servo,
// ver Controller/Readme.md del repositorio del laboratorio: necesita 16 bits
// de ancho de temporizador, y ESP32Servo 3.2.1 no permite fijarlos.
#include "utils.h"
#include <Adafruit_APDS9960.h> // v1.3.1
#include <ArduinoOTA.h>
#include <FastLED.h>    // v3.10.5
#include <ICM_20948.h>  // v1.3.2 — con ICM_20948_USE_DMP activo en ICM_20948_C.h
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ============================================================================
// CONFIGURACIÓN DE DEBUG
// ============================================================================

// Descomentar solo para debug
#define DebugSerial
#ifdef DebugSerial
#define DebugSerialPrint(x) Serial.print(x)
#define DebugSerialPrintln(x) Serial.println(x)
#define DebugSerialPrintf(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
#define DebugSerialPrint(x)
#define DebugSerialPrintln(x)
#define DebugSerialPrintf(x, ...)
#endif

// ============================================================================
// DEFINICIÓN DE PINES
// ============================================================================

#define leftMotorForward 12
#define leftMotorBackward 14
#define rightMotorForward 13
#define rightMotorBackward 15

#define leftEncoderC1 32
#define leftEncoderC2 35
#define rightEncoderC1 23
#define rightEncoderC2 25

#define enableLeftInfraredSensor 5
#define leftInfraredSensor 33
// GPIO 26 libre (no hay servo frontal).
#define enableRightInfraredSensor 18
#define rightInfraredSensor 27

#define batteryStatus 19
#define ledPin 2
#define AD0_VAL 1
#define NUM_LEDS 1

#define pwm_freq 1000
#define pwm_resolution 14

// ============================================================================
// CONSTANTES DEL SISTEMA
// ============================================================================

// WiFi
const char *ssid = "Atta-Bot";
const char *password = "attabot1234";
const unsigned int localPort = 6060;
char receivedPacket[255];

// Constantes del robot
float pulsesPerRev = 574;
const float wheelCircumference = PI * 44.5;
float millimetersPerPulse = wheelCircumference / pulsesPerRev;
// Los dos se ajustan en vivo desde la Base y se persisten: la media distancia
// entre ruedas con NAV_CONFIG|WHEEL_DIST y la escala del gyro con
// NAV_CONFIG|YAW_SCALE|x|SAVE. yawScale es la razon entre el giro fisico y el
// que reporta la IMU, calibrada contra ArUco; se midio hasta ±2.4% por robot.
float centerToWheelDistance = 41.5;
float yawScale = 1.0f;

// No usar radians() en este sketch: usar DegToRad/DegToArc.
//
// FastLED hace `#undef radians` (fl/stl/undef.h) y después `using fl::radians`
// (FastLED.h), así que radians() no es la macro de Arduino sino una plantilla
// que convierte la constante al tipo del argumento. Con un ángulo entero,
// static_cast<int>(0.01745…) es 0 y radians(180) devuelve 0: el giro queda en
// un no-op silencioso. Estos auxiliares usan la constante literal en float y
// toman float, así que fuerzan la promoción en la llamada y no dependen ni de
// FastLED ni de DEG_TO_RAD. Ver Controller/Readme.md del laboratorio.
inline float DegToRad(float degrees) {
  return degrees * 0.017453292519943295f;
}

// Grados de giro en el lugar → arco (mm) que recorre cada rueda. Es la única
// vía para esa conversión en todo el sketch.
inline float DegToArc(float degrees) {
  return DegToRad(degrees) * centerToWheelDistance;
}

// Muestreo y velocidad
const unsigned int samplingTime = 10;
const float samplingTimeS = samplingTime * 0.001;
const unsigned int SteadyStateTime = 800;
const float distanceOffset = 1 * millimetersPerPulse;
const float baseSpeed = millimetersPerPulse / samplingTimeS;
const float maxSpeed = baseSpeed * 5;
const float minSpeed = 12;
const int speedReductionThreshold = 16;

// Control PID
const int maxPWMValue = (1 << pwm_resolution) - 1;
const int minPWMValue = maxPWMValue * 0.20;
pidConstants pidSpeed(110, 375, 2);
kalmanFilter kfPID(6.0, 1.0, 1.0);

// Sensores de obstáculos
const int observationPeriod = 28800;
const int observationTime = 1600;
const int numberOfCycles = observationPeriod / observationTime;
// BDI-E · La ranura de emisión IR se deriva del ID que asigna la base
// (BdieOnConfigId en bdie.ino), en vez de sortearse con random() al inicializar
// las globales, antes de randomSeed() en setup(): con el generador sin sembrar,
// todos los robots obtenían la misma ranura, emitían a la vez y se leían
// mutuamente como obstáculo. Sortearla por robot tampoco basta: con 5 robots y
// 18 ranuras, la probabilidad de colisión es de un 46 %.
int lateralCycle = 0;
int centralCycle = numberOfCycles / 2;
const unsigned minObstacleTime = 1350;

// Detección de robots
const float robotDistanceMargin = 260;
const float maxRobotAngleMargin = 80;
const int obstacleWaitTime = 600;
const int reverseDistance = -40;

// Escape de bloqueo (ACTIVE_EVASION tras N evasiones): giro de rodeo de 95°
// hacia el interior de la arena, en vez de un giro ciego de 180°, con un tramo
// de 300 mm. El lado se elige proyectando el tramo por cada lado y tomando el
// que queda más adentro, con DISP_ARENA como límites.
const float escapeTurnDeg = 95.0f;
const float escapeMoveMm  = 300.0f;

// Arena del escenario en curso (marco de la cámara). No es constante: la base
// la envía con NAV_CONFIG|ARENA|w|h al registrar cada robot.
float arenaWidthMm  = 2400.0f;
float arenaHeightMm = 1750.0f;

// Límites de los saltos de dispersión: la arena en curso menos un margen,
// para que ningún salto apunte a la pared.
#define DISP_INSET 350.0f
#define DISP_ARENA_XMIN DISP_INSET
#define DISP_ARENA_XMAX (arenaWidthMm - DISP_INSET)
#define DISP_ARENA_YMIN DISP_INSET
#define DISP_ARENA_YMAX (arenaHeightMm - DISP_INSET)

// Debug
int debugUdp = 0;
int debugCounter = 0;
char direction = '+';
const char *debugMessage =
    "DEBUG: %d, ID: %s, Direccion: %c, val: Izq|Der, Encoder: %d|%d, Vel: "
    "%.2f|%.2f, Pwm: %d|%d, ErrorP: %.2f|%.2f, ErrorI: %.2f|%.2f, Dis: "
    "%.2f|%.2f, Tiempo: %d";

// Random Walk
const std::array<int, 7> possibleAngles = {30, 45, 60, 75, 90, 135, 180};
const std::array<int, 4> possibleAdvances = {200, 250, 300, 350};
enum possibleDirections { TURN_POS = 0, MOVE_FORWARD, TURN_NEG };

// Filtro de saltos bruscos al actualizar la pose: un salto de más de 500 mm o
// de más de 179° entre lecturas se descarta como error de la cámara. Tras tres
// rechazos seguidos se acepta igual, porque entonces el que está mal es el
// modelo interno y hay que volver a sincronizarse con la cámara.
const float max_pose_jump = 500;
const float max_angle_jump = 179;
const int max_pose_jump_rejections = 3;
int poseJumpRejections = 0;

// IMU
const float conversionFactor = 8192.0;
float yaw;
float imuGravity;
// Solo pasa a true si setupIMU() completo sin errores.
bool imuAvailable = false;

// LEDs
int maxBrightness = 140;

// Batería
volatile unsigned long lowBatteryTime = 0;
int minLowBatteryTime = 200;

// Contador de mensajes
int countMessages = 0;

// Sensor frontal (APDS9960). Si no arranca al principio se reintenta cada 5 s,
// porque el bus I2C a veces no está listo en el primer intento.
bool frontSensorInitialized = false;
unsigned long lastFrontSensorAttempt = 0;
const unsigned long frontSensorRetryInterval = 5000;
volatile bool lateralSensorsEnabled = false;

// ============================================================================
// VARIABLES GLOBALES (estructuras definidas en utils.h)
// ============================================================================

InterruptionContext intContext;
EvasionTracker evasionTracker;
CongregationState congregation;
// Throttle del broadcast LEADER_POSITION que emite el lider.
unsigned long lastLeaderCast = 0;

// Ventana móvil sobre la pose que difunde el líder, para no propagar el ruido
// de la cámara al goal de los seguidores. La usa SmoothLeaderPose().
const int   LEADER_SMOOTH_N     = 4;
const float LEADER_SMOOTH_RESET = 100.0f;
float leaderSmX[LEADER_SMOOTH_N], leaderSmY[LEADER_SMOOTH_N];
float leaderSmS[LEADER_SMOOTH_N], leaderSmC[LEADER_SMOOTH_N];
int   leaderSmCount = 0, leaderSmIdx = 0;

// Estado de los tres subsistemas de enjambre. Mientras la cámara conteste, la
// navegación corre sobre robotPose; el EKF es el respaldo que la mantiene viva
// cuando deja de contestar, y pasa a ser la fuente primaria con EKF_NAV.
DisperseState disperse;
EKFState ekf;
SearchState search;

// SEARCH_OBJECT: acercamiento al candidato. El robot avanza en pasos cortos
// con el PID de las ruedas (el mismo MoveDistanceByWheel de MOVE) y lee el
// color cuando llega a SEARCH_PROX_NEAR, cuando la proximidad deja de subir
// dos pasos seguidos (está tocando) o cuando agota los pasos, siempre que la
// proximidad sea al menos SEARCH_PROX_MIN_READ.
const int SEARCH_CREEP_PWM = 70;             // sin uso: el acercamiento es por pasos con el PID
const uint8_t SEARCH_PROX_NEAR = 180;
// Con el robot pegado al objetivo rojo la proximidad satura en 255 y el color
// es inequívoco (R÷G de 7,0 a 8,3); a pocos centímetros la proximidad baja a
// 33–41 y el color queda en el límite (R÷G de 1,05 a 1,77), con falsos
// negativos. Por eso solo se lee desde un 60 % del valor de contacto.
const uint8_t SEARCH_PROX_MIN_READ = 150;
const float SEARCH_STEP_MM = 20.0f;
const int SEARCH_MAX_STEPS = 8;              // 160 mm como máximo
const unsigned long SEARCH_STEP_MAX_MS = 1500;   // paso que no termina = está tocando
const unsigned long SEARCH_APPROACH_TIMEOUT = 12000;
// Memoria de candidatos descartados (ver StateActiveEvasion).
const unsigned long SEARCH_REJECT_MEMORY_MS = 30000;
const float SEARCH_REJECT_RADIUS_MM = 200.0f;
float searchRejectX = -1e6f, searchRejectY = -1e6f;
unsigned long searchRejectMs = 0;
// Sondeo lateral en búsqueda (ver StateActiveEvasion): ángulo del giro hacia
// el lado del contacto, tramo de avance para que el APDS9960 central llegue a
// ver el objeto y tiempo mínimo entre sondeos.
const float LATERAL_PROBE_DEG = 40.0f;
const float LATERAL_PROBE_MM  = 80.0f;
const unsigned long LATERAL_PROBE_COOLDOWN_MS = 4000;
unsigned long lastLateralProbeMs = 0;
/* Sondeo lateral fino: tras un contacto lateral, el robot retrocede poco y
 * barre hacia ese lado en pasos de 10° con una pausa corta; si en cualquier
 * paso el sensor central ve algo, el giro se interrumpe y el obstáculo pasa a
 * ser candidato (como en el barrido de celda). Si en 60° no aparece nada,
 * vuelve al centro del barrido (±30°) y avanza un tramo corto. Un giro grande
 * de una sola vez se pasa del objetivo, y un retroceso largo lo saca del
 * alcance del sensor. */
const int   LATERAL_PROBE_REVERSE_MM = -20;
const float LATERAL_PROBE_STEP_DEG   = 10.0f;
const int   LATERAL_PROBE_STEPS      = 6;
const unsigned long LATERAL_PROBE_PAUSE_MS = 120;
/* Si un contacto ocurre a menos de LATERAL_PROBE_SAME_MM del anterior, se
 * permite un sondeo más sin esperar el tiempo mínimo (que existe para no
 * rebotar indefinidamente contra lo mismo). */
const float LATERAL_PROBE_SAME_MM    = 120.0f;
float lastLateralProbeX = -9999, lastLateralProbeY = -9999;
int   lateralProbeSeguidos = 0;
/* Viaje cortado junto a la meta: en BUSCAR, si el tope de viaje corta un
 * viaje a menos de BDIE_REVISAR_META_MM de la meta, lo más probable es que lo
 * que impide llegar esté en esa celda. Antes de soltar, el robot gira hacia la
 * meta y hace un acercamiento de candidato hacia ella. */
const float BDIE_REVISAR_META_MM = 300.0f;

// Exposición del canal de color del APDS9960: ganancia (1/4/16/64x) y tiempo
// de integración en ms. El begin() de la librería arranca en 4x y 10 ms, y con
// eso la arena da C≈30 sobre una escala de 65535: una cuenta es el 10 % del
// valor y las razones entre canales con que decide MatchColor son ruido de
// cuantización. Con 64x y 100 ms, C≈6200: cerca de 1 % de precisión en las
// razones y todavía 10x de margen antes de saturar (el sensor es lineal en la
// exposición a un 3 %). Se ajustan en vivo con COLOR_READ|<ganancia>|<ms>, y
// no afectan a la proximidad, que usa otro registro (PGAIN).
const uint8_t COLOR_GAIN_DEFAULT = 64;
const uint16_t COLOR_INTEGRATION_MS_DEFAULT = 100;
// El tope de integración no es el del chip (~709 ms) sino lo que
// HandleColorRead puede esperar sin dejar de atender la red.
const uint16_t COLOR_INTEGRATION_MS_MAX = 200;
uint8_t colorGain = COLOR_GAIN_DEFAULT;
uint16_t colorIntegrationMs = COLOR_INTEGRATION_MS_DEFAULT;

// Balance de blancos del canal de color, por robot (COLOR_WB, guardado en
// NVS). Cada canal se divide por lo que ese sensor lee sobre el patrón neutro
// (el obstáculo beige de la arena), con lo que el patrón da (1,1,1) en todos y
// un mismo umbral sirve para toda la flota. Entre sensores del mismo modelo, la
// fracción de azul sobre el mismo patrón llega a variar un 37 %, suficiente
// para que un robot sin calibrar confunda el beige con rojo.
//
// (0,0,0) = sin calibrar: MatchColor compara los canales crudos.
uint16_t colorWbR = 0, colorWbG = 0, colorWbB = 0;
// Cuánto tiene que destacar un canal sobre los otros dos para dar el color por
// bueno. Con el balance aplicado, el neutro queda en 1,0 en los tres canales,
// así que este número es margen limpio sobre el fondo.
const float COLOR_MATCH_K = 1.5f;

ObstacleState obstacles;
MovementMetrics movement;
LedController ledCtrl;

// Lectura de la IMU a 50 Hz, por debajo de la tasa de salida del DMP (~112 Hz).
unsigned long lastImuRead = 0;
const unsigned long imuReadInterval = 20;

// Reporte pasivo del EKF a la base, para validarlo sin que controle nada. El
// EKF corre siempre como observador (EKF_NAV arranca apagado); la base escribe
// su pose en la misma fila del PositionLog que la pose de la cámara, y la
// deriva se calcula como una resta de columnas. 2 Hz basta para medirla.
unsigned long lastEkfReport = 0;
const unsigned long ekfReportInterval = 500;

// Navegación a ciegas: cuando la cámara no contesta el REQUEST_POSITION, el
// robot sigue con la pose del EKF en vez de abandonar la navegación. Se limita
// por dos lados porque la odometría sola se degrada rápido: un tope de pasos
// seguidos sin cámara y un tope de incertidumbre del filtro. Al superar
// cualquiera de los dos, el robot se detiene y avisa.
int blindNavSteps = 0;
const int BLIND_NAV_MAX_STEPS = 8;
const float BLIND_NAV_MAX_SIGMA = 250.0f;

// Variables de control de movimiento
unsigned long currentMillis = millis();
unsigned long previousMillisRW = 0;
int millisDifference;
int pastLeftEncoder = 0;
int pastRightEncoder = 0;

// Variables de sensores
unsigned long currentMicros = micros();
unsigned long previousMicros = micros();
bool isLateralCycleActive = false;
bool isCentralCycleActive = false;
// Máscaras de sensores IR — true = ignorar ese sensor (SENSOR_MASK|L/R/C|1)
bool maskLeftIR = false;
bool maskRightIR = false;
bool maskCentralIR = false;
// Umbral de proximidad del APDS9960 central (0..255, mayor = más cerca).
// Configurable en vivo con SENSOR_THRESHOLD|C|<n>[|SAVE] y persistido en NVS.
int centralIRThreshold = 2;
int cycleCounter = 0;
volatile unsigned long leftObsStartTime = 0;
volatile unsigned long rightObsStartTime = 0;
unsigned long centralObsStartTime = 0;
int centralDistance;

// Estado del robot
String robotID = "-1";
std::map<String, IPAddress> robots;
RobotState state = WAIT;
float instructionValue = 100;
bool movementReady = true;

// Navegación reactiva unificada (ir a un punto y congregación). EKF_NAV|1 pasa
// la navegación a la pose del EKF en vez del ArUco crudo; arranca apagado.
ReactiveNav nav;
bool ekfNavEnabled = false;

// Giro cerrado en guiñada: el arco restante se reorienta con la IMU en cada
// ciclo y al final se corrige el residuo. imuTurnPrevYaw sirve para desenvolver
// el ángulo de forma incremental e imuTurnAccumDeg guarda el giro medido sin
// envolver, de modo que admite arcos de más de 180°. imuTurnTargetDeg lleva
// signo (+ = antihorario).
bool  imuTurnActive = false;
bool  imuTurnIsCorrection = false;
int   imuTurnCorrCount = 0;
float imuTurnStartYaw = 0.0f;
float imuTurnPrevYaw = 0.0f;
float imuTurnAccumDeg = 0.0f;
float imuTurnTargetDeg = 0.0f;
unsigned long imuTurnSettleUntil = 0;

// Constantes del giro asistido, medidas contra el ArUco:
//   BrakeLead      cortar motores 3° antes: la inercia (2–12°) completa el giro;
//   SettleMs       ventana para que esa inercia termine antes de verificar;
//   Tolerance      residuo bajo el cual el giro se da por bueno;
//   MaxCorrections tope de correcciones por giro, para no perseguir el ruido
//                  del giroscopio indefinidamente.
// La corrección del residuo se hace sin BrakeLead (imuTurnIsCorrection): en
// arcos pequeños la inercia reservada se comería la corrección entera.
const float imuTurnBrakeLead = 3.0f;
const unsigned long imuTurnSettleMs = 400;
const float imuTurnTolerance = 3.0f;
const int imuTurnMaxCorrections = 4;

const int instructionCompletedDelay = 400;

// Pausas del paso de navegación (ReactiveNavStep), ajustables en vivo:
//    NAV_CONFIG|WAIT_TURN|80    NAV_CONFIG|WAIT_MOVE|150
//  - tras el giro: el giro asistido ya termina con su propia espera
//    (imuTurnSettleMs);
//  - tras el avance: la base responde con la mediana de las últimas tres
//    detecciones ArUco (~150 ms a 20 fps); hay que esperar a que sean del robot
//    ya detenido.
unsigned long navWaitAfterTurnMs = 300;
unsigned long navWaitAfterMoveMs = 300;

// Vigilante de movimiento y esperas a otros robots (ver estados.ino).
// MOVE de 150 mm → 3 s + 3,75 s; TURN de 180° (arco de 130 mm) → 5 s + 5,2 s.
const unsigned long MOTION_MOVE_BASE_MS = 3000;
const float         MOTION_MOVE_MS_PER_MM = 25.0f;
const unsigned long MOTION_TURN_BASE_MS = 5000;
const float         MOTION_TURN_MS_PER_MM = 40.0f;
unsigned long motionStartMs = 0;
unsigned long motionBudgetMs = 0;
const int           IDENT_ROBOT_MAX = 6;
const unsigned long IDENT_ROBOT_WINDOW_MS = 4000;
std::array<float, 2> fsmInstruction;
std::deque<std::array<float, 2>> instructionList;

pose robotPose(0, 0, 0);

// Evasión
bool isEvading = false;
unsigned long evasionStartTime = 0;
const unsigned long evasionCooldown = 2000;
bool resumeScheduled = false;
bool obstacleDetected = false;

// Controladores PID
pidController leftControl(kfPID, pidSpeed, samplingTimeS, minPWMValue,
                          maxPWMValue);
pidController rightControl(kfPID, pidSpeed, samplingTimeS, minPWMValue,
                           maxPWMValue);

// Hardware
Adafruit_APDS9960 frontSensor;
CRGB leds[NUM_LEDS];
WiFiUDP udp;
ICM_20948_I2C imu;
Preferences preferences;

// ============================================================================
// DECLARACIONES ADELANTADAS DE FUNCIONES
// ============================================================================

// Setup y configuración
void SetupFrontSensor();
void WiFiStatus();
void updateMillimetersPerPulse();
void InitializePPR();
void SavePPR(float newPPR);
void InitializePID();
void SavePID(float kp, float ki, float kd);

// Comunicación
void ReadUdpPackets();
void SendMessage(IPAddress host, const char *message);
void SendPose();
void MessageDebugf(const char *format, ...);

// Sensores y control
void ReadSensors();
void SetColorExposure(uint8_t gain, uint16_t ms);
void ReadColorRaw(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c);
void SaveColorWhiteBalance(uint16_t r, uint16_t g, uint16_t b);
void ResetPID();
void ConfigureHBridge(int leftWheelPWM, int rightWheelPWM);

// Movimiento
bool MoveDistanceByWheel(float leftDistance, float rightDistance);
float DesiredSpeed(float distance, float wheelDistance);
bool IsStationary(float currentLeftSpeed, float currentRightSpeed,
                  float leftWheelDistance, float rightWheelDistance);
void SelectMovementRW();

// Navegación y congregación
float SafeRingSlotAngle(float lx, float ly, int idx, int n, float nominalR,
                        float bearing, bool useBearing, float *outR);
void SmoothLeaderPose(float xIn, float yIn, float angIn,
                      float *xOut, float *yOut, float *angOut);
void UpdateCongregationGoal(float leaderX, float leaderY, float leaderAngle);
void ReactiveNavStep();
bool RequestPositionQueued();

// Auxiliares
std::array<String, 6> SeparateCommand(const String &command, char delimiter);
void MaybeDisperseHop();
int  MeetSlotIndex(float tx, float ty, float ring, int n);
bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id);
char DecidirPaso(int yo, int otro, bool meMuevo);   // negociación de paso, navegacion.ino
const char *NombrePaso(char p);
void ReadSerialCommands();
// CalculateDistance, NormalizeAngle e InRange están definidas inline en utils.h;
// se vuelven a declarar aquí para que sean visibles desde esta unidad.
inline float CalculateDistance(float x1, float y1, float x2, float y2);
inline float CalculateAngleToTarget(float x1, float y1, float x2, float y2);
inline float NormalizeAngle(float angle);

// LED
void setLedColor(uint8_t red, uint8_t green, uint8_t blue);
void setLedBrightness(uint8_t brightness);
void setLedBlink(uint8_t red, uint8_t green, uint8_t blue,
                 unsigned long intervalMs);

// IMU
void setupIMU();
void SaveIMUBias(biasStore* store);
void LeerYaw();
void EkfTick();
bool MatchColor(const char *target, uint16_t r, uint16_t g, uint16_t b,
                uint16_t c);
void SearchEvadeAndResume();
void SearchTrace(const char *fmt, ...);
bool MotionTimedOut();
void AbortMotion(const char *que);

// ═══ BDI-E ═══════════════════════════════════════════════════════════════════
// Va aquí y no arriba: la parte de Arduino de mission.h usa nav,
// instructionList, fsmInstruction, SendMessage y robots, que deben estar
// declarados antes. Las funciones del acople están en bdie.ino.
#include "mission.h"        // protocolo y acople; incluye bdi.h y los demás encabezados
#include "telemetry.h"      // telemetría de razonamiento hacia la base (solo observación)
bdie::Agent agent;
// Telemetría a la base: 0 apagada, 1 solo THINK, 2 THINK + BMAP (por omisión).
// BDIE_TELEM|n la cambia en vivo. Ningún robot la lee: la misión no depende de ella.
uint8_t            bdieTelemMode = 2;
bdie::TelemTicker  bdieTelem;
bool        missionActive  = false;
uint32_t    deliberationUs = 0;   // costo del ciclo BDI, reportado en GET_STATUS
uint32_t    loopPeriodUs   = 0;   // período real de loop(), reportado en GET_STATUS
// Color del objetivo de la misión; TARGET_COLOR|<color> lo cambia en vivo.
char        bdieTargetColor[12] = "rojo";
bool        bdieOwnsSearch = false;   // la búsqueda la armó BDI-E, no la base
bool        bdieOwnsMeet   = false;   // el MEET en curso lo pidió BDI-E, no la base
// Barrido al llegar a cada celda en BUSCAR (ver StateDeliberate en bdie.ino):
// gira +bdieScanHalfDeg y después −2·bdieScanHalfDeg, así el sensor frontal
// mira el frente y los dos costados (±45°) en 1 a 2 s. Termina a −45° del
// rumbo; el paso siguiente de la navegación lo vuelve a alinear.
//   NAV_CONFIG|SCAN|0     apagado
//   NAV_CONFIG|SCAN|1     encendido (con el ángulo vigente)
//   NAV_CONFIG|SCAN|<g>   semiángulo de g grados (10–179); 180 = giro completo
bool        bdieScanEnabled  = true;
bool        bdieScanPending  = false;
float       bdieScanHalfDeg  = 45.0f;
// Tope de un viaje de exploración en misión (bdie.ino, punto 1b del enganche
// de loop()). Un viaje con evasión recorre unos 900 mm en 25 s; 30 ms/mm deja
// un margen de 1,8 veces.
/* Candidato aunque parezca robot. Cuando otro robot está a menos de 260 mm y
 * dentro de ±80° del frente (robotDistanceMargin / maxRobotAngleMargin),
 * contesta OBSTACLE_DETECTED «soy yo», y el que ve el obstáculo espera y lo
 * esquiva sin acercarse a leerlo; con los robots agrupados cerca de la última
 * celda sin visitar, que suele ser la del objetivo, eso pasa casi siempre. En
 * búsqueda, un obstáculo central se revisa igual: si era un robot, el
 * acercamiento por pasos topa sin proximidad (las carcasas no reflejan) y queda
 * descartado 30 s. NAV_CONFIG|CAND_ROBOT|0 vuelve al comportamiento del
 * laboratorio. */
bool searchCandAunqueRobot = true;
/* Negociación de paso entre robots. El que pregunta CHECK_OBSTACLE y recibe
 * «soy yo» esperaría a que el otro se aparte aunque el otro esté quieto o lo
 * esté esperando a su vez. Por eso el que contesta dice también qué va a
 * hacer, OBSTACLE_DETECTED|id|QUIETO|SIGO|CEDO, y la simetría se rompe con la
 * misma regla del reparto de zonas: el menor ID tiene el paso.
 *   QUIETO  no está navegando y no se va a mover → el que pregunta evade ya.
 *   SIGO    se mueve y tiene el paso (su ID es menor) → el que pregunta evade ya.
 *   CEDO    se mueve y cede el paso (su ID es mayor) → el que pregunta espera
 *           como en el laboratorio; el otro, al preguntar a su vez, recibe SIGO
 *           y evade. Si aun así no se aparta, rige IDENT_ROBOT_MAX.
 * NAV_CONFIG|PASO|0 vuelve al protocolo del laboratorio (respuesta sin tercer
 * campo y espera siempre). Es retrocompatible: un robot sin este cambio ignora
 * el tercer campo, y una respuesta sin él se trata como la del laboratorio. */
bool pasoNegociado = true;
/* Latencia entre robots (instrumentación, no la usa la misión). Con
 * NAV_CONFIG|LAT|1 (o |<ms>) el robot difunde LAT_PING|seq|t_us|id por la misma
 * difusión que usan HB, TGT, ZC y ZA; cada compañero lo devuelve directo
 * (LAT_PONG|seq|t_us|id_B) y el que preguntó reporta a la base
 * LAT|id_B|rtt_ms|enviados|recibidos. La ida y vuelta es una cota superior de
 * la latencia de difusión entre robots y no necesita relojes sincronizados;
 * incluye el paso por loop() de los dos robots, es decir, la latencia que vive
 * el agente. Los contadores son acumulados, así que la pérdida se calcula
 * aunque se pierda algún LAT. Apagado por omisión. Ver latencia.ino. */
uint32_t latPeriodoMs = 0;                 // 0 = apagado
uint32_t latSeq = 0, latEnviados = 0, latUltimoMs = 0;
bool     latArrancado = false;
std::map<int, uint32_t> latRecibidos;      // pongs recibidos por compañero
void LatTick();
void LatConfigurar(long n);
void HandleLatPing(const std::array<String, 6> &arguments);
void HandleLatPong(const std::array<String, 6> &arguments);
const unsigned long BDIE_NAV_BASE_MS   = 15000;
const float         BDIE_NAV_MS_POR_MM = 30.0f;
// BDIE_SILENCIO|s (prueba): hasta este millis() el robot no envía ni escucha
// mensajes del enjambre. Reproduce una desconexión sin tocar el router.
uint32_t    bdieSilencioHasta = 0;
// Reintentos acotados de REQUEST_POSITION cuando no hay EKF para seguir a
// ciegas: una sola respuesta perdida no debe detener el movimiento en curso.
static const int kMaxNavRequestRetries = 3;
int         navRequestRetries = 0;

void BdieInitGrid();
void BdieOnConfigId();
void BdieLoopHook();
void BdieReleaseBody();
void BdieTargetConfirmed();
void BdieResumeAfterCandidate();
void BdieStartMeet(float x, float y);
void BdieDropMeet();
void BdieOnNavArrived();
void BdieMarcarBarrido();
bool BdieFaseActiva();
void StateDeliberate();
void HandleBdieMessage();
void HandleTargetColor(const std::array<String, 6> &arguments);
void HandleBdieTelem(const std::array<String, 6> &arguments);
void HandleBdieSilencio(const std::array<String, 6> &arguments);
bool BdieEnSilencio();
void BdiePump();
void BdieTelemetry();
// ═════════════════════════════════════════════════════════════════════════════

// ============================================================================
// INTERRUPCIONES (ISR)
// ============================================================================

// Barre el bus I2C y lista lo que responde. Diagnostico de banco: si la IMU o
// el APDS9960 no arrancan, esto dice si el problema es el cable o la libreria.
void i2cScan() {
  Serial.println("\n=== I2C SCAN ===");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  Dispositivo en 0x%02X", addr);
      if (addr == 0x68) Serial.print("  ← IMU (AD0=GND)");
      if (addr == 0x69) Serial.print("  ← IMU (AD0=VCC)");
      if (addr == 0x39) Serial.print("  ← APDS9960");
      Serial.println();
      found++;
    }
  }
  Serial.printf("  Total: %d dispositivo(s)\n", found);
  Serial.println("================\n");
}

// Encoder en cuadratura de la rueda izquierda. Compara la lectura anterior con
// la nueva y suma o resta según la transición, de modo que el conteo lleva
// signo y tolera los rebotes.
void IRAM_ATTR LeftWheelPulses() {
  int MSB = digitalRead(leftEncoderC2);
  int LSB = digitalRead(leftEncoderC1);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastLeftEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    movement.leftPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    movement.leftPulseCount--;
  }
  pastLeftEncoder = encoder;
}

// Encoder en cuadratura de la rueda derecha, espejo del izquierdo.
void IRAM_ATTR RightWheelPulses() {
  int MSB = digitalRead(rightEncoderC1);
  int LSB = digitalRead(rightEncoderC2);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastRightEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    movement.rightPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    movement.rightPulseCount--;
  }
  pastRightEncoder = encoder;
}

// Flanco del infrarrojo izquierdo. Solo anota el instante: el filtro por
// duración mínima corre en ReadSensors, fuera de la interrupción.
void IRAM_ATTR DetectLeftObstacle() {
  if (lateralSensorsEnabled && digitalRead(leftInfraredSensor) == LOW) {
    leftObsStartTime = micros();
  }
}

// Flanco del infrarrojo derecho, espejo del izquierdo.
void IRAM_ATTR DetectRightObstacle() {
  if (lateralSensorsEnabled && digitalRead(rightInfraredSensor) == LOW) {
    rightObsStartTime = micros();
  }
}

// Aviso de batería baja. Anota el instante para que el filtro por duración
// descarte las caídas momentáneas que produce el arranque de los motores.
void LowBattery() {
  if (digitalRead(batteryStatus) == LOW) {
    lowBatteryTime = millis();
  }
}

// ============================================================================
// FUNCIONES DE SETUP Y CONFIGURACIÓN
// ============================================================================

// Carga los pulsos por revolución desde NVS, o guarda el valor por defecto la
// primera vez. Cada robot tiene el suyo, calibrado contra ArUco (difieren
// alrededor de un 2 % entre unidades).
void InitializePPR() {
  preferences.begin("attabot-config", false);

  float storedPPR = preferences.getFloat("ppr", 0);

  if (storedPPR == 0) {
    preferences.putFloat("ppr", pulsesPerRev);
    DebugSerialPrintf("PPR inicial guardado: %.2f\n", pulsesPerRev);
  } else {
    pulsesPerRev = storedPPR;
    DebugSerialPrintf("PPR cargado desde memoria: %.2f\n", pulsesPerRev);
  }

  updateMillimetersPerPulse();

  yawScale = preferences.getFloat("yaw_scale", 1.0f);
  DebugSerialPrintf("Yaw scale: %.4f\n", yawScale);

  centralIRThreshold = preferences.getInt("ir_cen_thr", 2);
  DebugSerialPrintf("Umbral IR central: %d\n", centralIRThreshold);

  colorWbR = preferences.getUShort("cwb_r", 0);
  colorWbG = preferences.getUShort("cwb_g", 0);
  colorWbB = preferences.getUShort("cwb_b", 0);
  DebugSerialPrintf("Balance de blancos del color: %u/%u/%u%s\n", colorWbR,
                    colorWbG, colorWbB,
                    (colorWbR && colorWbG && colorWbB) ? "" : " (SIN CALIBRAR)");

  preferences.end();

  uint64_t chipid = ESP.getEfuseMac();
  DebugSerialPrintf("Robot Chip ID: %04X%08X\n", (uint16_t)(chipid >> 32),
                    (uint32_t)chipid);
}

// Persiste el vector del blanco del sensor de color. (0,0,0) lo desactiva.
void SaveColorWhiteBalance(uint16_t r, uint16_t g, uint16_t b) {
  colorWbR = r;
  colorWbG = g;
  colorWbB = b;
  preferences.begin("attabot-config", false);
  preferences.putUShort("cwb_r", r);
  preferences.putUShort("cwb_g", g);
  preferences.putUShort("cwb_b", b);
  preferences.end();
  DebugSerialPrintf("Balance de blancos guardado: %u/%u/%u\n", r, g, b);
}

// Guarda un PPR nuevo y recalcula lo que depende de él.
void SavePPR(float newPPR) {
  preferences.begin("attabot-config", false);
  preferences.putFloat("ppr", newPPR);
  preferences.end();
  DebugSerialPrintf("PPR guardado permanentemente: %.2f\n", newPPR);
}

// Carga las constantes del PID desde NVS, o guarda las de fábrica la primera
// vez.
void InitializePID() {
  preferences.begin("attabot-config", false);
  int   savedRes = preferences.getInt  ("pid_res", -1);
  float kp       = preferences.getFloat("pid_kp",  -1.0f);
  float ki       = preferences.getFloat("pid_ki",  -1.0f);
  float kd       = preferences.getFloat("pid_kd",  -1.0f);
  preferences.end();

  if (kp > 0.0f && savedRes == pwm_resolution) {
    leftControl.pidConst.kp  = kp;
    leftControl.pidConst.ki  = ki;
    leftControl.pidConst.kd  = kd;
    rightControl.pidConst.kp = kp;
    rightControl.pidConst.ki = ki;
    rightControl.pidConst.kd = kd;
    DebugSerialPrintf("PID cargado desde flash: Kp=%.2f Ki=%.2f Kd=%.3f\n", kp, ki, kd);
  } else {
    if (kp > 0.0f && savedRes != pwm_resolution) {
      DebugSerialPrintf("PID en flash descartado: guardado con %d-bit, actual %d-bit\n",
                        savedRes, pwm_resolution);
    }
    DebugSerialPrintf("PID usando defaults: Kp=%.2f Ki=%.2f Kd=%.3f\n",
                      leftControl.pidConst.kp, leftControl.pidConst.ki, leftControl.pidConst.kd);
  }
}

// Guarda constantes de PID nuevas.
void SavePID(float kp, float ki, float kd) {
  preferences.begin("attabot-config", false);
  preferences.putInt  ("pid_res", pwm_resolution);
  preferences.putFloat("pid_kp",  kp);
  preferences.putFloat("pid_ki",  ki);
  preferences.putFloat("pid_kd",  kd);
  preferences.end();
  DebugSerialPrintf("PID guardado (%d-bit): Kp=%.2f Ki=%.2f Kd=%.3f\n",
                    pwm_resolution, kp, ki, kd);
}

// Guarda los sesgos de giroscopio y acelerómetro que dejó la calibración del
// DMP, para no repetirla en cada arranque.
void SaveIMUBias(biasStore* store) {
  preferences.begin("attabot-config", false);
  preferences.putInt("bias_gx", store->biasGyroX);
  preferences.putInt("bias_gy", store->biasGyroY);
  preferences.putInt("bias_gz", store->biasGyroZ);
  preferences.putInt("bias_ax", store->biasAccelX);
  preferences.putInt("bias_ay", store->biasAccelY);
  preferences.putInt("bias_az", store->biasAccelZ);
  preferences.putInt("bias_cx", store->biasCPassX);
  preferences.putInt("bias_cy", store->biasCPassY);
  preferences.putInt("bias_cz", store->biasCPassZ);
  preferences.end();
  DebugSerialPrintln("Bias IMU guardados en Preferences");
}

// Recalcula la constante de conversión pulso → mm. Hay que llamarla después de
// cambiar el PPR o la circunferencia de la rueda.
void updateMillimetersPerPulse() {
  millimetersPerPulse = wheelCircumference / pulsesPerRev;
}

// Arranque: pines, PWM de motores, interrupciones de encoders e infrarrojos,
// configuración guardada, LED, IMU, sensor frontal, WiFi y OTA.
//
// El arranque se escalona con un retardo aleatorio: si varios robots encienden
// a la vez, todos piden IP en el mismo instante y el punto de acceso deja fuera
// a alguno.
void setup() {
#ifdef DebugSerial
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== INICIO DE SETUP ===");
#endif
  // BDI-E: datos del chip en una línea (cómputo disponible)
  DebugSerialPrintf("[0] %s rev%d, %d nucleos a %u MHz, flash %u MB, "
                    "PSRAM %u B, SDK %s, MAC %012llX\n",
                    ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                    ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / 1048576,
                    ESP.getPsramSize(), ESP.getSdkVersion(), ESP.getEfuseMac());

  // Retardo aleatorio para evitar colisiones de DHCP cuando varios ESP32
  // arrancan juntos; la MAC es la semilla, así cada robot tiene un retardo propio.
  randomSeed(ESP.getEfuseMac());
  unsigned long startupDelay = random(100, 2000);
  DebugSerialPrintf("Esperando %lu ms antes de iniciar WiFi...\n",
                    startupDelay);
  delay(startupDelay);

  DebugSerialPrintln("[1] Inicializando PPR desde flash...");
  InitializePPR();
  DebugSerialPrintln("[1] PPR OK");

  DebugSerialPrintln("[1b] Inicializando PID desde flash...");
  InitializePID();
  DebugSerialPrintln("[1b] PID OK");

  // Reloj del LEDC fijado en APB (80 MHz) antes de crear cualquier canal (con
  // canales ya tomados la llamada falla a propósito). Por defecto el driver usa
  // LEDC_AUTO_CLK y el reloj lo elige el primer periférico que se conecta; los
  // motores a 1 kHz y 14 bits necesitan una fuente de 16,384 MHz, que solo da
  // APB, y los cuatro temporizadores de baja velocidad comparten un único mux de
  // reloj. Fijarlo aquí deja el reloj explícito en vez de heredado.
  DebugSerialPrintln("[2] Fijando reloj del LEDC en APB...");
  if (!ledcSetClockSource(LEDC_USE_APB_CLK))
    DebugSerialPrintln("[2] AVISO: no se pudo fijar el reloj del LEDC en APB");

  DebugSerialPrintln("[3] Inicializando motores PWM...");
  bool pwmOk = ledcAttach(leftMotorForward,  pwm_freq, pwm_resolution)
             & ledcAttach(leftMotorBackward,  pwm_freq, pwm_resolution)
             & ledcAttach(rightMotorForward,  pwm_freq, pwm_resolution)
             & ledcAttach(rightMotorBackward, pwm_freq, pwm_resolution);
  if (!pwmOk) DebugSerialPrintf("[3] ERROR: ledcAttach falló — freq=%d res=%d "
                                "incompatibles, o algo más se quedó con el "
                                "reloj del LEDC (ver nota en [2])\n",
                                 pwm_freq, pwm_resolution);
  // Frecuencia real del temporizador, no la pedida: si el LEDC tuvo que ajustar
  // (u otro periférico le movió el reloj), aquí se ve. Debe leer 1000 Hz.
  // ledcReadFreq() mide el canal y devuelve 0 si el ciclo de trabajo es 0, y en
  // el arranque las ruedas están quietas: se le da un ciclo mínimo para poder
  // leerla. 1/16384 son 61 ns de pulso, muy por debajo del 20 % que el puente H
  // necesita para mover el motor.
  ledcWrite(leftMotorForward, 1);
  uint32_t realPwmFreq = ledcReadFreq(leftMotorForward);
  ledcWrite(leftMotorForward, 0);
  DebugSerialPrintf("[3] Motores PWM: %dHz %d-bit (max=%d) %s — real=%uHz\n",
                    pwm_freq, pwm_resolution, maxPWMValue, pwmOk ? "OK" : "FALLO",
                    realPwmFreq);

  DebugSerialPrintln("[4] Inicializando I2C...");
  Wire.begin();
  Wire.setClock(400000);
  i2cScan();
  DebugSerialPrintln("[4] I2C OK");

  // setHostname debe ir antes de WiFi.begin().
  DebugSerialPrintln("[5] Inicializando WiFi...");
  WiFi.mode(WIFI_STA);
  String hostname = "AttaBot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.setHostname(hostname.c_str());

  DebugSerialPrintf("Iniciando WiFi con hostname: %s\n", hostname.c_str());
  WiFi.begin(ssid, password);

  // BDI-E: sin esto la radio duerme entre balizas y el tiempo de ida y vuelta
  // sube a unos 110 ms, más de la mitad del límite de 200 ms entre robots.
  WiFi.setSleep(false);

  DebugSerialPrintln("[5] WiFi iniciado (sin ahorro de energia)");

  DebugSerialPrintln("[6] Inicializando LEDs...");
  FastLED.addLeds<WS2812, ledPin, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(maxBrightness);
  FastLED.setMaxRefreshRate(120);
  ledCtrl.setOff();
  DebugSerialPrintln("[6] LEDs OK");

  DebugSerialPrintln("[7] Inicializando OTA y UDP...");
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.begin();

  udp.begin(localPort);
  DebugSerialPrintf("El servidor UDP se inició en el puerto: %u\n", localPort);
  DebugSerialPrintln("[7] OTA y UDP OK");

  DebugSerialPrintln("[8] Configurando encoders...");
  pinMode(leftEncoderC1, INPUT_PULLUP);
  // GPIO 35 es input-only y no admite pull-up interno.
  pinMode(leftEncoderC2, INPUT);
                                 // (compatible con ESP32 Core 3.x)
  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), LeftWheelPulses,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), LeftWheelPulses,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), RightWheelPulses,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), RightWheelPulses,
                  CHANGE);
  DebugSerialPrintln("[8] Encoders OK");

  DebugSerialPrintln("[9] Configurando sensores infrarrojos...");
  pinMode(enableLeftInfraredSensor, OUTPUT);
  pinMode(enableRightInfraredSensor, OUTPUT);

  digitalWrite(enableLeftInfraredSensor, LOW);
  digitalWrite(enableRightInfraredSensor, LOW);

  pinMode(batteryStatus, INPUT);
  attachInterrupt(digitalPinToInterrupt(batteryStatus), LowBattery, FALLING);
  pinMode(leftInfraredSensor, INPUT);
  pinMode(rightInfraredSensor, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftInfraredSensor), DetectLeftObstacle,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(rightInfraredSensor),
                  DetectRightObstacle, FALLING);
  DebugSerialPrintln("[9] Sensores OK");

  // La IMU debe inicializarse antes que el APDS9960: invertir el orden rompe
  // sin aviso la inicialización del ICM-20948 en el bus I2C.
  DebugSerialPrintln("[10] Inicializando IMU ICM-20948...");
  setupIMU();
  if (imuAvailable) {
    DebugSerialPrintln("[10] IMU OK");
  } else {
    DebugSerialPrintln("[10] IMU no disponible — continuando sin IMU");
  }

  DebugSerialPrintln("[11] Inicializando sensor frontal APDS9960...");
  SetupFrontSensor();
  delay(200);

  BdieInitGrid();   // BDI-E: cuadrícula y agente con ID provisional

  DebugSerialPrintln("\n=== SETUP COMPLETO ===\n");
}
