#include "utils.h"
// Versiones con las que se compila hoy (2026-07-29), sobre ESP32 Arduino core
// 3.3.11. Actualizarlas al subir una librería: cuando el comentario miente, un
// cambio de librería se confunde con una falla de hardware.
// Ya no está ESP32Servo — el servo frontal se retiró (2026-07-29, no se usaba)
// porque su PWM a 50Hz/10-bit se llevaba el mux de reloj del LEDC y dejaba a
// los motores sin PWM: el robot no giraba ni avanzaba. Si algún día vuelve un
// servo, leer primero Controller/Readme.md — necesita 16 bits de ancho de
// timer y ESP32Servo 3.2.1 no deja ponérselos (attach() pisa el default).
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
// GPIO 26 libre: era el servo frontal, retirado el 2026-07-29.
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

// NO USAR radians() EN ESTE SKETCH — usar DegToRad/DegToArc.
//
// FastLED hace `#undef radians` (fl/stl/undef.h) y después `using fl::radians`
// (FastLED.h:226), así que radians() no es la macro de Arduino sino esto:
//     template<typename T> constexpr T radians(T deg) {
//         return deg * static_cast<T>(0.017453292519943295);
//     }
// La constante se castea al TIPO DEL ARGUMENTO. Con un ángulo entero,
// static_cast<int>(0.01745…) es 0, así que radians(180) devuelve 0 y el giro
// queda en un no-op silencioso. Con float anda bien, y de ahí el cuadro que
// costó semanas: el robot navegaba hacia el objetivo (nav usa floats) pero no
// esquivaba obstáculos (`int avoidanceAngle`), el RANDOM_WALK no giraba y
// TURN|grados no hacía nada en ningún robot. Llegó con FastLED 3.10.5
// (2026-07-27); antes la macro de Arduino promovía todo a double.
// Estos helpers usan la constante literal en float y toman float a propósito:
// fuerzan la promoción en el call site y no dependen ni de FastLED ni de
// DEG_TO_RAD. Diagnosticado el 2026-07-29, ver Controller/Readme.md.
inline float DegToRad(float degrees) {
  return degrees * 0.017453292519943295f;
}

// Grados de giro en el lugar → arco (mm) que recorre cada rueda. Única vía
// permitida para esa conversión, que antes vivía repetida en 9 lugares.
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
// BDI-E · LA RANURA IR NO SE SORTEA (punto 10).
// Eran `const int` inicializados con random() durante el arranque de las
// globales — ANTES de randomSeed(ESP.getEfuseMac()) en setup(). Con el
// generador sin sembrar todos los robots obtenían LA MISMA ranura, emitían a la
// vez y se leían mutuamente como obstáculo. Ahora se derivan del ID que asigna
// la base (BdieOnConfigId, bdie.ino). Sortearlas por robot tampoco serviría:
// con 5 robots y 18 ranuras la colisión es ~46 %.
int lateralCycle = 0;
int centralCycle = numberOfCycles / 2;
const unsigned minObstacleTime = 1350;

// Detección de robots
const float robotDistanceMargin = 260;
const float maxRobotAngleMargin = 80;
const int obstacleWaitTime = 600;
const int reverseDistance = -40;

// Escape de deadlock (ACTIVE_EVASION tras N evasiones): giro de rodeo de 95°
// hacia el interior de la arena, en vez del giro 180° ciego, con un tramo de
// 300mm. Espeja escape_turn() de la réplica de sim (attabot_firmware.py). El
// lado se elige proyectando el tramo por cada lado y tomando el que queda más
// adentro, con DISP_ARENA como límites.
const float escapeTurnDeg = 95.0f;
const float escapeMoveMm  = 300.0f;

// Arena del escenario en curso (marco cámara). NO es constante: la base la
// envía con NAV_CONFIG|ARENA|w|h al registrar cada robot, porque cambia por
// escenario (4 robots = 2400×1750, 10 robots = 3800×2800).
float arenaWidthMm  = 2400.0f;
float arenaHeightMm = 1750.0f;

// Límites de los saltos de dispersión: la arena en curso menos un inset, para
// que ningún salto apunte a la pared. Antes eran cuatro #define fijos al montaje
// de 2400×1550 del lab, así que en cualquier otro escenario los robots se
// dispersaban contra un borde imaginario.
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

// Filtro de saltos bruscos al actualizar la pose: un salto de mas de 500mm o de
// mas de 179° entre lecturas se descarta por misread de la camara. Tras tres
// rechazos seguidos se acepta igual, porque a esa altura el que esta mal es el
// modelo interno y hay que re-sincronizar con la camara.
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

// Sensor frontal (APDS9960). Si no arranca al principio se reintenta cada 5s,
// porque el bus I2C a veces no esta listo en el primer intento.
bool frontSensorInitialized = false;
unsigned long lastFrontSensorAttempt = 0;
const unsigned long frontSensorRetryInterval = 5000;
volatile bool lateralSensorsEnabled = false;

// ============================================================================
// VARIABLES GLOBALES REFACTORIZADAS (usando estructuras de utils.h)
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

// SEARCH_OBJECT: PWM de aproximación lenta, umbral de readProximity() a partir
// del cual el objeto está al alcance, y tope de la fase de acercamiento.
// Acercamiento al candidato (17-oct-2026). Antes: PWM fijo de 70 a lazo
// abierto hasta proximidad 180. En el laboratorio ese PWM NO movía al robot
// (Atta_2 pasó >60 s en SEARCH_APPROACH con la pose clavada a proximidad 10),
// así que nunca llegaba a leer el color. Ahora avanza en pasos cortos con el
// PID de las ruedas (el mismo MoveDistanceByWheel de MOVE; R2: el PID no se
// toca, solo se usa) y lee el color cuando: llega a SEARCH_PROX_NEAR, o la
// proximidad deja de subir dos pasos seguidos (está tocando), o agotó los pasos
// — siempre que la proximidad sea al menos SEARCH_PROX_MIN_READ.
const int SEARCH_CREEP_PWM = 70;             // ya no se usa para avanzar
const uint8_t SEARCH_PROX_NEAR = 180;
// Medido el 17-oct con los tres robots PEGADOS al cubo rojo: proximidad 255
// (saturada) y color inequívoco (R÷G 7,0–8,3; R÷B 2,9–5,3). A unos pocos cm la
// proximidad era 33–41 y el color quedaba en el límite (R÷G 1,05–1,77), así que
// leer ahí da falsos negativos. Se lee solo desde ~60 % del valor de contacto.
const uint8_t SEARCH_PROX_MIN_READ = 150;
const float SEARCH_STEP_MM = 20.0f;
const int SEARCH_MAX_STEPS = 8;              // 160 mm como máximo
const unsigned long SEARCH_STEP_MAX_MS = 1500;   // paso que no termina = está tocando
const unsigned long SEARCH_APPROACH_TIMEOUT = 12000;
// Memoria de candidatos descartados (17-oct-2026, ver StateActiveEvasion)
const unsigned long SEARCH_REJECT_MEMORY_MS = 30000;
const float SEARCH_REJECT_RADIUS_MM = 200.0f;
float searchRejectX = -1e6f, searchRejectY = -1e6f;
unsigned long searchRejectMs = 0;
// Sondeo lateral en búsqueda (12-oct-2026, ver StateActiveEvasion): ángulo del
// giro hacia el lado del contacto, tramo de avance para que el APDS9960 central
// llegue a ver el objeto, y tiempo mínimo entre sondeos.
const float LATERAL_PROBE_DEG = 40.0f;
const float LATERAL_PROBE_MM  = 80.0f;
const unsigned long LATERAL_PROBE_COOLDOWN_MS = 4000;
unsigned long lastLateralProbeMs = 0;
/* SONDEO LATERAL FINO (24-oct-2026). Corrida 15:06: el Atta_3 rozó el cubo a
 * 31 mm con el cubo 13° a la izquierda; el sondeo de entonces retrocedía 40 mm,
 * giraba 40° de una vez y avanzaba 80 mm: el giro se pasaba del cubo (quedaba
 * 27° hacia el otro lado) y el retroceso lo sacaba del alcance del APDS9960.
 * Ahora retrocede poco y barre hacia ese lado en pasos de 10° con una pausa
 * corta: si en cualquier paso el sensor central lo ve, el giro se interrumpe y
 * el obstáculo pasa a ser candidato (como en el barrido de celda). Si en 60° no
 * aparece nada, vuelve al centro del barrido (±30°) y avanza un tramo corto. */
const int   LATERAL_PROBE_REVERSE_MM = -20;
const float LATERAL_PROBE_STEP_DEG   = 10.0f;
const int   LATERAL_PROBE_STEPS      = 6;
const unsigned long LATERAL_PROBE_PAUSE_MS = 120;
/* Espera entre sondeos (24-oct): en la misma corrida, el segundo roce del cubo
 * cayó dentro de los 4 s de espera y se esquivó sin mirar. Si el contacto está
 * a menos de LATERAL_PROBE_SAME_MM del anterior se permite UN sondeo más sin
 * esperar (la espera existe para no rebotar indefinidamente en lo mismo). */
const float LATERAL_PROBE_SAME_MM    = 120.0f;
float lastLateralProbeX = -9999, lastLateralProbeY = -9999;
int   lateralProbeSeguidos = 0;
/* VIAJE CORTADO JUNTO A LA META (24-oct): en BUSCAR, si el tope de viaje corta
 * un viaje a menos de BDIE_REVISAR_META_MM de la meta, lo más probable es que lo
 * que no lo deja llegar esté EN esa celda —en la corrida 15:06, 6 viajes del
 * Atta_2 a celdas junto al cubo se cortaron así—. Antes de soltar, gira hacia la
 * meta y hace un acercamiento de candidato hacia ella. */
const float BDIE_REVISAR_META_MM = 300.0f;

// Exposicion del canal de COLOR del APDS9960: ganancia (1/4/16/64x) y tiempo de
// integracion en ms. El begin() de la libreria arranca en 4x y 10ms, y con eso
// la arena da C≈30 sobre un fondo de escala de 65535: a ese nivel una cuenta es
// el 10% del valor y las razones entre canales con las que decide MatchColor
// son ruido de cuantizacion. Se barren en vivo con COLOR_READ|<ganancia>|<ms>.
// NO tocan la proximidad, que corre por otro registro (PGAIN).
// Medido en la arena el 2026-08-13 sobre el obstaculo beige: 4x/10ms daba C≈30
// (ruido puro), 16x/50ms da C≈760 y 64x/100ms da C≈6200 sobre 65535 — ~1% de
// precision en las razones entre canales y todavia 10x de margen antes de
// saturar. El sensor resulto lineal en la exposicion a un 3%.
const uint8_t COLOR_GAIN_DEFAULT = 64;
const uint16_t COLOR_INTEGRATION_MS_DEFAULT = 100;
// El tope de integracion no es el del chip (~709ms) sino el que HandleColorRead
// puede esperar sin dejar de atender la red.
const uint16_t COLOR_INTEGRATION_MS_MAX = 200;
uint8_t colorGain = COLOR_GAIN_DEFAULT;
uint16_t colorIntegrationMs = COLOR_INTEGRATION_MS_DEFAULT;

// Balance de blancos del canal de color, POR ROBOT (COLOR_WB, persistido en
// NVS). Cada canal se divide por lo que ESE sensor lee sobre el patron neutro
// —el obstaculo beige de la arena—, con lo que el patron da (1,1,1) en todos y
// un solo umbral vuelve a valer para la flota.
//
// Medido el 2026-08-13 con los 4 robots mirando el MISMO obstaculo desde casi
// el MISMO punto (dispersion de luz 8.9%): la fraccion de azul iba de 0.239
// (Atta_3) a 0.328 (Atta_2) — 37% entre piezas del mismo modelo y distinto
// lote. Con eso Atta_3 daba el beige por ROJO (razon 1.518 contra el 1.5 de
// umbral) de forma repetible, en dos exposiciones y dos posiciones.
//
// (0,0,0) = sin calibrar: MatchColor compara los canales crudos, como antes.
uint16_t colorWbR = 0, colorWbG = 0, colorWbB = 0;
// Cuanto tiene que destacar un canal sobre los otros dos para dar el color por
// bueno. Con el balance puesto, el neutro queda en 1.0 en los tres canales, asi
// que este numero pasa a ser margen limpio sobre el fondo.
const float COLOR_MATCH_K = 1.5f;

ObstacleState obstacles;
MovementMetrics movement;
LedController ledCtrl;

// Lectura del IMU a 50Hz, por debajo del ODR del DMP (~112Hz).
unsigned long lastImuRead = 0;
const unsigned long imuReadInterval = 20;

// Reporte pasivo del EKF a la base, para VALIDARLO sin que controle nada.
// El EKF corre siempre como observador (EKF_NAV arranca apagado), así que
// mandando su pose se puede medir cuánto deriva contra el ArUco durante las
// corridas normales: la base lo escribe en la misma fila del PositionLog que la
// pose de cámara, y el error queda como una resta de columnas. Sin esto la única
// forma de verlo era polear GET_STATUS a mano. 2Hz alcanza para medir deriva.
unsigned long lastEkfReport = 0;
const unsigned long ekfReportInterval = 500;

// Navegación a ciegas: cuando la cámara no contesta el REQUEST_POSITION, el
// robot sigue con la pose del EKF en vez de abandonar la navegación. Se limita
// por dos lados porque la odometría sola se degrada rápido: un tope de pasos
// seguidos sin ver la cámara, y un tope de incertidumbre del propio filtro. Al
// pasarse cualquiera de los dos el robot se detiene y avisa, que es lo honesto;
// lo que no puede pasar es lo de antes, quedarse quieto al primer timeout.
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

// Navegación reactiva unificada (GT + congregación). EKF_NAV|1 pasa la nav a la
// pose del EKF en vez del ArUco crudo; arranca apagado para poder A/B-testearlo
// contra el comportamiento conocido.
ReactiveNav nav;
bool ekfNavEnabled = false;

// Giro cerrado en yaw: el arco restante se re-apunta con el IMU en cada ciclo y
// al final se corrige el residuo. imuTurnPrevYaw sirve para el unwrap
// incremental y imuTurnAccumDeg guarda el giro medido sin wrap, de modo que
// soporta arcos de más de 180°. imuTurnTargetDeg lleva signo (+ = CCW).
bool  imuTurnActive = false;
bool  imuTurnIsCorrection = false;
int   imuTurnCorrCount = 0;
float imuTurnStartYaw = 0.0f;
float imuTurnPrevYaw = 0.0f;
float imuTurnAccumDeg = 0.0f;
float imuTurnTargetDeg = 0.0f;
unsigned long imuTurnSettleUntil = 0;

// Constantes del giro asistido, todas medidas contra el ArUco:
//   BrakeLead      cortar motores 3° antes, que la inercia (coast de 2-12°)
//                  completa el giro;
//   SettleMs       ventana para que ese coast termine antes de verificar;
//   Tolerance      residuo bajo el cual el giro se da por bueno;
//   MaxCorrections tope de correcciones por giro, para no perseguir el ruido
//                  del gyro indefinidamente.
// La corrección del residuo se hace SIN brake-lead (imuTurnIsCorrection): en
// arcos chicos el coast reservado se comía la corrección entera, y un comando
// de -5.3° terminaba girando 0.8°.
const float imuTurnBrakeLead = 3.0f;
const unsigned long imuTurnSettleMs = 400;
const float imuTurnTolerance = 3.0f;
const int imuTurnMaxCorrections = 4;

const int instructionCompletedDelay = 400;

// Pausas del paso de navegación (ReactiveNavStep), antes fijas en 300 ms
// (17-oct-2026). Arrancan en 300/300 = comportamiento idéntico al anterior; se
// bajan EN VIVO para comparar A/B sin reflashear:
//    NAV_CONFIG|WAIT_TURN|80    NAV_CONFIG|WAIT_MOVE|150
//  - tras el GIRO: el giro asistido ya termina con su espera (imuTurnSettleMs).
//  - tras el AVANCE: la base responde la mediana de las últimas 3 detecciones
//    ArUco (~150 ms a 20 fps); hay que esperar a que sean del robot detenido.
unsigned long navWaitAfterTurnMs = 300;
unsigned long navWaitAfterMoveMs = 300;

// Vigilante de movimiento y esperas a robots (17-oct-2026, ver estados.ino).
// MOVE de 150 mm → 3 s + 3,75 s; TURN de 180° (arco 130 mm) → 5 s + 5,2 s.
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
// DECLARACIONES FORWARD DE FUNCIONES
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
// Nota: CalculateDistance, NormalizeAngle e InRange están definidas inline en utils.h
// Se redeclaran aquí para garantizar visibilidad desde este translation unit
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
// Va ACÁ y no arriba: mission.h §2 usa nav, instructionList, fsmInstruction,
// SendMessage y robots, y todos tienen que estar declarados antes. Las
// funciones viven en bdie.ino.
#include "mission.h"        // protocolo + pegamento; arrastra bdi.h y los demás
#include "telemetry.h"      // qué piensa el agente → base (solo observación)
bdie::Agent agent;
// Telemetría a la base: 0 apagada, 1 solo THINK, 2 THINK + BMAP (por omisión).
// BDIE_TELEM|n la cambia en vivo. No la lee ningún robot: la misión no depende.
uint8_t            bdieTelemMode = 2;
bdie::TelemTicker  bdieTelem;
bool        missionActive  = false;
uint32_t    deliberationUs = 0;   // costo del ciclo BDI → GET_STATUS
uint32_t    loopPeriodUs   = 0;   // período real de loop() → GET_STATUS
// Color del objetivo de la misión; TARGET_COLOR|<color> lo cambia en vivo.
char        bdieTargetColor[12] = "rojo";
bool        bdieOwnsSearch = false;   // la búsqueda la armó BDI-E, no la base
bool        bdieOwnsMeet   = false;   // el MEET en curso lo pidió BDI-E, no la base
// Barrido al llegar a cada celda en BUSCAR (17-oct-2026, ver StateDeliberate).
// Barrido CORTO: gira +bdieScanHalfDeg y después −2·bdieScanHalfDeg, así el
// sensor frontal mira el frente y los dos costados (±45°) en ~1–2 s. El giro de
// 360° de la primera versión costaba ~4 s por celda y en la arena se veía de más.
// Queda a −45° del rumbo: el paso siguiente de la navegación lo re-alinea igual.
//   NAV_CONFIG|SCAN|0     apagado
//   NAV_CONFIG|SCAN|1     encendido (con el ángulo vigente)
//   NAV_CONFIG|SCAN|<g>   semiángulo de g grados (10–179); 180 = giro completo
bool        bdieScanEnabled  = true;
bool        bdieScanPending  = false;
float       bdieScanHalfDeg  = 45.0f;
// Tope de un viaje de exploración en misión (bdie.ino, BdieLoopHook 1b).
// Medido el 17-oct: ~900 mm en 25 s con evasión → 30 ms/mm deja margen ×1,8.
/* CANDIDATO AUNQUE PAREZCA ROBOT (24-oct-2026). Corrida de las 13:52: el
 * enjambre cubrió el 100 % sin leer el cubo. De las 13 detecciones CENTRALES
 * junto al cubo, 11 tenían a otro robot a < 260 mm y ±80° del frente
 * (robotDistanceMargin / maxRobotAngleMargin), que contestó OBSTACLE_DETECTED
 * «soy yo»: el que veía el cubo esperaba y lo esquivaba, nunca se acercaba a
 * leerlo. Con los robots amontonados alrededor de la última celda sin visitar
 * —la del cubo— pasa casi siempre. En búsqueda, un obstáculo central se revisa
 * igual: si era el robot, el acercamiento por pasos topa sin proximidad (las
 * carcasas no reflejan) y queda descartado 30 s. NAV_CONFIG|CAND_ROBOT|0 vuelve
 * al comportamiento del laboratorio. */
bool searchCandAunqueRobot = true;
/* NEGOCIACIÓN DE PASO ENTRE ROBOTS (05-nov-2026). Corridas del 24-oct: el que
 * preguntaba CHECK_OBSTACLE y recibía «soy yo» esperaba a que el otro se
 * apartara, aunque el otro estuviera QUIETO (15:05, t = 1120 s: Atta_2 esperó
 * ~25 s a un Atta_1 sin navegación activa) o esperara a su vez al primero
 * (16:32, t = 67 s: los dos se esperaban mutuamente). Ahora el que contesta dice
 * también qué va a hacer — OBSTACLE_DETECTED|id|QUIETO|SIGO|CEDO — y la simetría
 * se rompe con la misma regla del reparto de zonas: el MENOR ID tiene el paso.
 *   QUIETO  no estoy navegando: no me voy a mover → el que pregunta evade ya.
 *   SIGO    me muevo y tengo el paso (mi ID es menor) → el que pregunta evade ya.
 *   CEDO    me muevo y te cedo el paso (mi ID es mayor) → el que pregunta espera
 *           como en el laboratorio; el otro, al preguntar a su vez, recibe SIGO
 *           y evade. Si aun así no se aparta, sigue valiendo IDENT_ROBOT_MAX.
 * NAV_CONFIG|PASO|0 vuelve al protocolo del laboratorio (respuesta sin tercer
 * campo y espera siempre). Es retrocompatible: un robot sin este cambio ignora
 * el tercer campo, y una respuesta sin él se trata como la del laboratorio. */
bool pasoNegociado = true;
/* LATENCIA ENTRE ROBOTS (05-nov-2026) — instrumento de OE3, no de la misión.
 * Con NAV_CONFIG|LAT|1 (o |<ms>) el robot difunde LAT_PING|seq|t_us|id por el
 * MISMO broadcast que usan HB, TGT y ZC/ZA; cada compañero lo devuelve directo
 * (LAT_PONG|seq|t_us|id_B) y el que preguntó le reporta a la base
 * LAT|id_B|rtt_ms|enviados|recibidos. La ida y vuelta es COTA SUPERIOR de la
 * latencia de difusión robot→robot (rtt = difusión + vuelta ≥ difusión) y no
 * necesita relojes sincronizados. Incluye el paso por loop() de los dos robots:
 * es la latencia que vive el agente, no la del aire. Los contadores son
 * acumulados, así que la pérdida se calcula aunque se pierda algún LAT a la
 * base. Apagado por omisión: la misión no lo necesita. Ver latencia.ino. */
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
// BDIE_SILENCIO|s (prueba, 14-oct-2026): hasta este millis() el robot no manda
// ni escucha mensajes del enjambre. Reproduce una desconexión sin tocar el router.
uint32_t    bdieSilencioHasta = 0;
// Reintentos ACOTADOS de REQUEST_POSITION cuando no hay EKF para seguir a
// ciegas: una sola respuesta perdida no puede matar el movimiento en curso.
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
// la nueva y suma o resta segun la transicion, de modo que el conteo lleva
// signo y sobrevive a los rebotes.
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
// duracion minima corre en ReadSensors, fuera de la interrupcion.
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

// Aviso de bateria baja. Anota el instante para que el filtro por duracion
// descarte los bajones momentaneos que produce el arranque de los motores.
void LowBattery() {
  if (digitalRead(batteryStatus) == LOW) {
    lowBatteryTime = millis();
  }
}

// ============================================================================
// FUNCIONES DE SETUP Y CONFIGURACIÓN
// ============================================================================

// Carga los pulsos por revolucion desde NVS, o guarda el valor por defecto la
// primera vez. Cada robot tiene el suyo: se calibran contra ArUco y se midieron
// diferencias de ~2% entre unidades.
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

// Persiste un PPR nuevo y recalcula lo que depende de el.
void SavePPR(float newPPR) {
  preferences.begin("attabot-config", false);
  preferences.putFloat("ppr", newPPR);
  preferences.end();
  DebugSerialPrintf("PPR guardado permanentemente: %.2f\n", newPPR);
}

// Carga las constantes del PID desde NVS, o guarda las de fabrica la primera
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

// Persiste constantes de PID nuevas.
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

// Persiste los sesgos de giroscopo y acelerometro que dejo la calibracion del
// DMP, para no tener que repetirla en cada arranque.
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

// Recalcula la constante de conversion pulso->mm. Hay que llamarla despues de
// cambiar el PPR o la circunferencia de rueda.
void updateMillimetersPerPulse() {
  millimetersPerPulse = wheelCircumference / pulsesPerRev;
}

// Arranque: pines, PWM de motores, interrupciones de encoder e infrarrojos,
// carga de la configuracion persistida, LED, IMU, sensor frontal, WiFi y OTA.
//
// El arranque se escalona con un retardo aleatorio: con varios robots
// encendiendo a la vez, todos pedian IP en el mismo instante y el AP dejaba
// afuera a alguno.
void setup() {
#ifdef DebugSerial
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== INICIO DE SETUP ===");
#endif
  // BDI-E: el chip en una línea, para el Cap. 4 (R9: cómputo cerca del límite)
  DebugSerialPrintf("[0] %s rev%d, %d nucleos a %u MHz, flash %u MB, "
                    "PSRAM %u B, SDK %s, MAC %012llX\n",
                    ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                    ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / 1048576,
                    ESP.getPsramSize(), ESP.getSdkVersion(), ESP.getEfuseMac());

  // Delay aleatorio para evitar colisiones DHCP cuando múltiples ESP32 arrancan
  // juntos Usa la MAC address como semilla para que cada robot tenga un delay
  // único
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

  // Reloj del LEDC clavado en APB (80MHz), antes de cualquier canal (con
  // canales ya tomados la llamada falla a propósito). Por defecto el driver va
  // en LEDC_AUTO_CLK y lo elige el PRIMER periférico que se attachea; los
  // motores a 1kHz/14-bit necesitan 16.384MHz de fuente, que solo sale de APB,
  // y los 4 timers low-speed comparten un único mux de reloj. Cuando el servo
  // frontal (50Hz/10-bit) se lo llevaba al reloj lento, los ledcAttach() de
  // los motores fallaban y el robot no giraba ni avanzaba (2026-07-29). El
  // servo ya no está, pero esto deja el reloj explícito en vez de heredado.
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
  // Frecuencia REAL del timer, no la pedida: si el LEDC tuvo que ajustar (o si
  // otro periférico le movió el reloj), acá se ve. Debe leer 1000Hz.
  // ledcReadFreq() mide el CANAL y devuelve 0 si el duty es 0 (core 3.3.11,
  // esp32-hal-ledc.c:411), y en el arranque las ruedas están quietas: hay que
  // darle un duty mínimo para que la lectura sea legible. 1/16384 son 61ns de
  // pulso, muy por debajo del 20% que el puente H necesita para mover el motor.
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

  // IMPORTANTE: setHostname DEBE estar ANTES de WiFi.begin()
  DebugSerialPrintln("[5] Inicializando WiFi...");
  WiFi.mode(WIFI_STA);
  String hostname = "AttaBot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.setHostname(hostname.c_str());

  DebugSerialPrintf("Iniciando WiFi con hostname: %s\n", hostname.c_str());
  WiFi.begin(ssid, password);

  // BDI-E: sin esto la radio duerme entre balizas y el ida y vuelta se va a
  // ~110 ms (medido el 28-sep-2026, p1_arranque.py, 117 muestras). El umbral
  // comprometido entre robots es 200 ms: el ahorro se comía la mitad.
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

  // IMPORTANTE: La IMU debe inicializarse ANTES que el APDS9960.
  // Invertir este orden rompe silenciosamente la init del ICM-20948 en el bus I2C.
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
