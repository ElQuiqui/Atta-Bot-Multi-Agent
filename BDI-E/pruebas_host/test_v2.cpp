// Prueba de integración en la PC: el sketch completo (todos los .ino del
// laboratorio, bdie.ino y los encabezados) compilado contra stubs de hardware,
// con reloj y red simulados. No prueba la física: prueba que los enganches se
// encadenan como se espera.
#include <cassert>
static int fails = 0;
#define CHECK(c, msg) do{ if(c) printf("  ok   %s\n", msg); else { printf("  FALLA %s\n", msg); fails++; } }while(0)
static void step(int n=1){ for(int i=0;i<n;i++){ g_ms += 5; loop(); } }
static bool sent(const char* pfx){ for(auto&s:g_out) if(s.rfind(pfx,0)==0) return true; return false; }
static bool sentHas(const char* pfx, const char* sub){ for(auto&s:g_out) if(s.rfind(pfx,0)==0 && s.find(sub)!=std::string::npos) return true; return false; }
static void poses(float x,float y,float a,unsigned ep){ char b[96]; snprintf(b,96,"POSES|%u|2:%.0f:%.0f:%.0f;",ep,x,y,a); g_in.push_back(b); }
// Cuerpo de juguete para el tramo de congregación: los TURN y MOVE se dan por
// terminados de inmediato (los stubs no tienen encoders ni IMU) y la base
// contesta cada REQUEST_POSITION con el robot ya en la meta vigente.
static unsigned g_ep = 100;
static float g_px = 500, g_py = 500;
static int drive(int maxIter, bool (*hasta)()){
  int k=0;
  for(; k<maxIter && !hasta(); k++){
    g_out.clear(); step(2);
    if(state==TURN || state==MOVE){ movementReady = true; state = STOP; }
    if(sent("REQUEST_POSITION")){
      if(nav.isActive){ g_px = nav.goalX; g_py = nav.goalY; }
      char b[96]; snprintf(b,96,"POSITION_RESPONSE|%.1f|%.1f|0",g_px,g_py); g_in.push_back(b);
      poses(g_px,g_py,0,g_ep++);
    }
  }
  return k;
}
static bool meetArmado(){ return bdieOwnsMeet && congregation.IsActive(); }
static bool llego(){ return agent.bel.arrivedAtPlace; }
static bool enVolver(){ return agent.bel.phase == bdie::PH_VOLVER; }
int main(){
  setup();
  g_in.push_back("CONFIG|START|192.168.1.255"); step(2);
  g_in.push_back("CONFIG|2"); step(2);
  CHECK(lateralCycle == 2 && centralCycle == (2+9)%18, "punto 10: ranura IR derivada del ID (L2 C11)");
  CHECK(agent.selfId == 2, "el agente toma el ID real de CONFIG");
  step(200);
  CHECK(!missionActive && state != DELIBERATE, "sin SESSION el robot no delibera (se comporta como el del lab)");
  unsigned ep = 1;
  for(int k=0;k<5;k++){ poses(500,500,0,ep++); step(4); }
  g_in.push_back("SESSION|START|1|1250|800"); step(2);
  CHECK(missionActive, "SESSION|START activa la misión");
  bool delib=false, reqpos=false;
  for(int k=0;k<400 && !(delib&&reqpos);k++){ poses(500,500,0,ep++); step(2);
    if(deliberationUs>0) delib=true; if(sent("REQUEST_POSITION")) reqpos=true; }
  CHECK(delib, "punto 2-3: WAIT -> DELIBERATE, el ciclo corre");
  CHECK(nav.pendingInit || nav.isActive, "ApplyAction arrancó la navegación del lab");
  CHECK(reqpos, "la navegación pregunta su pose a la base (lazo de ReactiveNav)");
  g_out.clear(); step(300);
  CHECK(sent("HB|2|"), "latido HB sale mientras navega (Comunicar en loop)");
  // ── Reparto por mensajes ──────────────────────────────────────────────────
  step(200);   // el reporte ZA se repite cada 2 s: la ventana tiene que cubrirlo
  CHECK(sentHas("ZA|", "|2|") && agent.rp.adopted && agent.bel.myZone == 0 && agent.assign.kz == 1,
        "solo en la arena: acuerda consigo mismo (k=1, franja 0) y lo reporta con ZA");
  g_in.push_back("ZC|1:6|1|0|3"); step(2);      // otro robot (ID 1) reclama la franja 0
  CHECK(agent.bel.peer[1].heard && agent.rp.peer[1].valid && agent.rp.peer[1].zone == 0,
        "ZC de un par entra por comandos.ino -> Feed -> OnZoneClaim, y cuenta como «lo oí»");
  bool zc=false;
  for(int k=0;k<400 && !(agent.rp.mask==0x6u && agent.rp.adopted);k++){
    if(k%40==0) g_in.push_back("HB|1|2|0|1");
    if(k%100==0) g_in.push_back("ZC|9:6|1|0|3");
    poses(500,500,0,ep++); step(2); if(sentHas("ZC|", "|2|")) zc=true; }
  CHECK(zc && agent.rp.rep >= 9, "un par nuevo abre ronda: el robot se pasa a la ronda mayor y reclama (ZC)");
  printf("       ronda %u, participantes 0x%x, franja %d de %d, acordada=%d\n",
         (unsigned)agent.rp.rep, (unsigned)agent.rp.mask, agent.bel.myZone, agent.assign.kz,
         (int)agent.rp.adopted);
  while(!g_in.empty()) step(1);
  g_in.push_back("BDIE_SILENCIO|2"); step(2); g_out.clear();
  for(int k=0;k<300;k++){ poses(500,500,0,ep++); step(1); }   // 1,5 s
  bool mudo=true; for(auto&s:g_out) if(s.rfind("HB|",0)==0||s.rfind("ZC|",0)==0||s.rfind("ZA|",0)==0) mudo=false;
  CHECK(mudo && BdieEnSilencio(), "BDIE_SILENCIO|2: no sale ni HB ni ZC/ZA");
  g_ms += 1000; g_out.clear(); step(250);
  CHECK(!BdieEnSilencio() && sent("HB|2|"), "pasado el silencio, vuelve a latir");
  CHECK(search.active && bdieOwnsSearch, "búsqueda por color del lab armada en exploración");
  CHECK(!strcmp(search.targetColor,"rojo"), "color objetivo por defecto: rojo");
  // La base difunde NEIGHBOR_POSITIONS a 1 Hz desde el arranque (lo usa el MEET)
  g_in.push_back("NEIGHBOR_POSITIONS|2,500,500;1,1500,900;3,300,1200"); step(2);
  // Hallazgo por la ruta del laboratorio: MatchColor positivo en SEARCH_APPROACH
  BdieTargetConfirmed(); state = STOP;   // como en StateSearchApproach
  CHECK(agent.bel.targetKnown && agent.bel.targetOwner == 2, "hallazgo -> agente conoce el objetivo, dueño 2");
  // Llega enseguida el hallazgo de un robot de menor ID, antes de que el 2, como
    // descubridor y ancla, cierre solo la congregación con quórum de 1.
  g_in.push_back("TGT|1|1|900|300");
  g_out.clear(); step(50);
  CHECK(sent("TGT|"), "TGT difundido");
  step(20);
  CHECK(!search.active, "búsqueda desarmada al conocer el objetivo");
  // El descubridor como ancla se prueba más abajo, en una sesión nueva: aquí
    // llega enseguida un TGT de menor ID y el robot deja de ser el descubridor.
    // Cambia el objetivo: gana el descubridor de menor ID.
  g_out.clear(); g_in.push_back("TGT|1|1|900|300"); step(4);
  drive(400, meetArmado);
  CHECK(agent.bel.targetOwner == 1 && meetArmado() &&
        congregation.globalTargetX == 900.0f && congregation.globalTargetY == 300.0f,
        "TGT de menor ID: deja de ser descubridor y corre el MEET del lab al punto nuevo (VIRTUAL)");
  CHECK(congregation.leaderID == "VIRTUAL" && congregation.hasGlobalTarget, "P_MEET_LAB dispara HandleMeet del lab (VIRTUAL, sin líder)");
  printf("       MEET: slot %d/%d, anillo %.0f mm\n", congregation.followerIndex,
         congregation.totalFollowers, congregation.parkingDist);
  CHECK(congregation.totalFollowers == 3, "n sale de NEIGHBOR_POSITIONS (MeetSlotIndex del lab)");
  int it = drive(2000, llego);
  printf("       llegó a la ranura en %d iteraciones; staging=%d\n", it, (int)congregation.stagingDone);
  CHECK(llego(), "ReactiveNavStep del lab declara la llegada -> agente.OnSlotReached()");
  drive(400, enVolver);
  CHECK(enVolver() && !congregation.IsActive() && !bdieOwnsMeet,
        "cierre (quórum de 1) -> VOLVER y el MEET del lab queda apagado");
  // Telemetría a la base (telemetry.h)
  g_out.clear(); step(250);   // 1,25 s
  size_t nThink=0, nMap=0, maxLen=0; std::string think;
  for(auto&s:g_out){ if(s.rfind("THINK|id=2|",0)==0){nThink++; think=s;} if(s.rfind("BMAP|id=2|",0)==0){nMap++; maxLen=std::max(maxLen,s.size());} }
  printf("       %s\n", think.c_str());
  CHECK(nThink>=2 && think.find("|fase=")!=std::string::npos && think.find("|plan=")!=std::string::npos, "telemetría: THINK a 2 Hz con fase/deseo/plan");
  CHECK(nMap>=1 && maxLen < 1024, "telemetría: BMAP a 1 Hz y cabe en un datagrama (<1024 B)");
  printf("       BMAP mide %zu B\n", maxLen);
  g_in.push_back("BDIE_TELEM|0"); step(2); g_out.clear(); step(300);
  bool calla=true; for(auto&s:g_out) if(s.rfind("THINK",0)==0||s.rfind("BMAP",0)==0) calla=false;
  CHECK(calla, "BDIE_TELEM|0 apaga la telemetría (experimento de latencia)");
  g_in.push_back("BDIE_TELEM|2"); step(2);
  g_in.push_back("GET_STATUS"); g_out.clear(); step(2);
  bool st=false; for(auto&s:g_out) if(s.rfind("STATUS|",0)==0){ st = s.find("|heap:")!=std::string::npos && s.find("|busca:")!=std::string::npos; printf("       %s\n", s.c_str()); }
  CHECK(st, "punto 9: GET_STATUS con heap/loop/bdi/cov/fase/zona/slot/busca");
  // Candidato que no era el objetivo, con navegación en curso
  nav.isActive = true; SearchEvadeAndResume();
  bool rw=false; for(auto&i:instructionList) if(i[0]==RANDOM_WALK) rw=true;
  CHECK(!rw && instructionList.back()[0]==REQUEST_POSITION, "en misión, candidato descartado vuelve a la nav (no RANDOM_WALK)");
  bool mA=missionActive; missionActive=false; SearchEvadeAndResume();
  CHECK(instructionList.back()[0]==RANDOM_WALK, "fuera de misión, SEARCH_OBJECT del lab intacto (RANDOM_WALK)");
  missionActive=mA;
  // Sondeo lateral en búsqueda: contacto solo izquierdo en mitad de la arena
  auto contactoIzq=[](float x,float y,float a){ instructionList.clear(); obstacles.Clear();
    obstacles.leftObstacle=true; obstacles.obstacleSensors=0b100; evasionStartTime=millis();
    robotPose.x=x; robotPose.y=y; robotPose.angle=a; search.active=true; StateActiveEvasion(); };
  auto tieneGiro=[](float signo){ for(auto&i:instructionList) if(i[0]==TURN) return (i[1]*signo)>0; return false; };
  lastLateralProbeMs = millis() - 10000;
  contactoIzq(1000,800,0);
  CHECK(tieneGiro(-1) && instructionList.size()>=3, "búsqueda + contacto lateral izquierdo: gira HACIA la izquierda a sondear");
  { int giros10=0; for(auto&i:instructionList) if(i[0]==TURN && fabs(i[1]-DegToArc(-10))<0.5f) giros10++;
    CHECK(giros10==6, "24-oct: el sondeo lateral barre en 6 pasos de 10° hacia ese lado"); }
  contactoIzq(1000,800,0);
  CHECK(tieneGiro(-1), "24-oct: segundo contacto en el MISMO lugar dentro de la espera: sondea una vez más");
  contactoIzq(1000,800,0);
  CHECK(tieneGiro(+1), "tercer contacto en el mismo lugar dentro del enfriamiento: evade normal (gira a la derecha)");
  g_ms += 5000; contactoIzq(1000,20,0);
  CHECK(tieneGiro(+1), "contacto contra la pared (punto fuera de la arena): evade normal");
  search.active=false; obstacles.Clear();
  nav.Reset(); instructionList.clear(); state=STOP; step(2);
  g_in.push_back("SESSION|ABORT"); step(4);
  printf("       fase tras ABORT = %d\n", (int)agent.bel.phase);
  CHECK(agent.bel.phase == bdie::PH_SEGURO && !nav.isActive && !nav.pendingInit && instructionList.empty(), "SESSION|ABORT: fase SEGURO y cuerpo suelto");
  { state=WAIT; g_in.push_back("MOVE|-100"); bool movio=false;
    for(int k=0;k<200 && !movio;k++){ step(1); if(state==MOVE) movio=true; }
    CHECK(!missionActive && movio, "tras ABORT (SEGURO) el robot vuelve a obedecer comandos manuales (MOVE|-100)");
    instructionList.clear(); state=STOP; step(2); }
  // Una sesión nueva empieza de cero (objetivo, llegada, feromona, cobertura)
  { CHECK(!agent.bel.targetKnown && agent.map.CoverageCells() == 0 && !agent.bel.congregationClosed,
          "SESSION|ABORT ya borra la memoria de la corrida (objetivo, cobertura, cierre)");
    g_in.push_back("SESSION|START|2|1250|800"); step(2);
    CHECK(!agent.bel.targetKnown && !agent.bel.arrivedAtPlace && !agent.bel.congregationClosed &&
          agent.map.CoverageCells() <= 1 && agent.bel.phase == bdie::PH_DESPLEGAR,
          "SESSION|START nuevo limpia objetivo, llegada, cierre, feromona y cobertura de la corrida anterior");
    // El descubridor se queda como ancla de la congregación
    for(int k=0;k<5;k++){ poses(500,500,0,ep++); step(4); }
    BdieTargetConfirmed(); state = STOP;
    for(int k=0;k<200 && !llego();k++){ poses(500,500,0,ep++); step(2); }
    CHECK(llego() && !meetArmado() && agent.bel.iFoundIt,
          "el descubridor no corre el MEET: cuenta como llegado y espera junto al objetivo");
    g_in.push_back("SESSION|ABORT"); step(4); instructionList.clear(); state=STOP; step(2);
    // Lo mismo por la ruta real (SEARCH_APPROACH + MatchColor) y con el objetivo
        // pegado al frente (proximidad 255): debe volver a deliberar, sin quedar en
        // bucle de CHECK_OBSTACLE.
    g_in.push_back("SESSION|START|2|1250|800"); step(2);
    for(int k=0;k<5;k++){ poses(500,500,0,ep++); step(4); }
    g_apdsProx=255; g_apdsR=900; g_apdsG=300; g_apdsB=400; g_apdsC=1200;
    instructionList.clear(); nav.Reset(); search.active=true; strcpy(search.targetColor,"rojo");
    movementReady=false; evasionStartTime=millis()-5000;   // como al final de un acercamiento por pasos
    state=SEARCH_APPROACH; instructionValue=0; search.BeginApproach(millis());
    int checks=0; g_out.clear();
    for(int k=0;k<300 && !llego();k++){ poses(500,500,0,ep++);
      if(state!=SEARCH_APPROACH){ obstacles.centralObstacle=true; obstacles.obstacleSensors=0b010; }  // el objetivo sigue ahí
      step(2); }
    for(auto&m:g_out) if(m.find("CHECK_OBSTACLE")!=std::string::npos) checks++;
    CHECK(agent.bel.iFoundIt && llego() && checks<=1,
          "descubridor con el cubo pegado (prox 255): delibera y cuenta como llegado, sin bucle de CHECK_OBSTACLE");
    g_apdsProx=0; g_apdsR=g_apdsG=g_apdsB=g_apdsC=0;
    g_in.push_back("SESSION|ABORT"); step(4); instructionList.clear(); state=STOP; step(2);
    // Volver a explorar tras «datos obsoletos» debe terminar en una evasión real
    g_apdsProx=255; search.active=false; missionActive=false; instructionList.clear(); g_out.clear();
    movementReady=false; isEvading=true; evasionStartTime=millis()-5000; state=STOP;
    bool evadio=false; checks=0;
    for(int k=0;k<400 && !evadio;k++){ obstacles.centralObstacle=true; obstacles.obstacleSensors=0b010;
      step(1); evadio = (state==REVERSE||state==TURN||state==MOVE); }
    for(auto&m:g_out) if(m.find("CHECK_OBSTACLE")!=std::string::npos) checks++;
    CHECK(evadio && checks<=2,
          "obstáculo central fijo con datos de evasión viejos: re-escanea UNA vez y evade (no bucle STOP↔IDENTIFY)");
    g_apdsProx=0; instructionList.clear(); obstacles.Clear(); isEvading=false; state=STOP; step(4); }
  g_in.push_back("TARGET_COLOR|azul"); step(2);
  CHECK(!strcmp(bdieTargetColor,"azul"), "TARGET_COLOR cambia el color en vivo");
  // Vigilante de movimiento: un TURN con las ruedas trabadas (los stubs no
    // cuentan pulsos) debe cortarse solo y avisar a la base.
  instructionList.clear(); fsmInstruction[0]=TURN; fsmInstruction[1]=DegToArc(90);
  instructionList.push_back(fsmInstruction); state=READ_INSTRUCTION; g_out.clear();
  step(2); bool girando = (state==TURN);
  for(int k=0;k<3000 && state==TURN;k++) step(1);      // hasta 15 s simulados
  CHECK(girando && state!=TURN && sent("WARNING: TURN sin terminar"), "TURN trabado se corta solo y avisa (vigilante)");
  // Bloqueo temporal de la meta sin progreso en BUSCAR
  { bdie::Agent ag; bdie::GridConfig gc; ag.Init(3, gc); ag.bel.phase = bdie::PH_BUSCAR;
    ag.TempBlockCell(40, 1000);
    bool b1 = ag.map.Blocked(40); ag.TempBlockExpire(1000 + ag.tempBlockMs + 1);
    CHECK(b1 && !ag.map.Blocked(40), "bloqueo temporal de celda: se bloquea y se libera solo"); }
  // Barrido al llegar a una celda buscando
  { bool mA2=missionActive; missionActive=true; bdieOwnsSearch=true; search.active=true;
    auto fA=agent.bel.phase; agent.bel.phase=bdie::PH_BUSCAR; instructionList.clear();
    BdieOnNavArrived(); StateDeliberate();
    bool giro = instructionList.size()==2 && instructionList[0][0]==TURN && instructionList[1][0]==TURN &&
                fabs(instructionList[0][1]-DegToArc(45))<0.5f && fabs(instructionList[1][1]-DegToArc(-90))<0.5f &&
                state==READ_INSTRUCTION;
    bdieScanEnabled=false; BdieOnNavArrived(); bool apagado=!bdieScanPending; bdieScanEnabled=true;
    CHECK(giro && apagado, "al llegar a una celda en BUSCAR barre +45° y -90° antes de deliberar (NAV_CONFIG|SCAN lo apaga)");
    agent.bel.phase=fA; missionActive=mA2; search.active=false; bdieOwnsSearch=false; instructionList.clear(); state=STOP; }
  // El retroceso no se corta por el obstáculo de enfrente
  { instructionList.clear(); fsmInstruction[0]=REVERSE; fsmInstruction[1]=-80; instructionList.push_back(fsmInstruction);
    state=READ_INSTRUCTION; isEvading=false; step(1);
    obstacles.centralObstacle=true; obstacles.obstacleSensors=0b010; step(3);
    CHECK(state==REVERSE, "el retroceso sigue aunque haya obstáculo adelante (no hay sensores atrás)");
    obstacles.Clear(); instructionList.clear(); state=STOP; step(2); }
  // Un candidato descartado no se vuelve a revisar en el mismo lugar
  { bool mA3=missionActive; missionActive=false; robotPose.x=900; robotPose.y=700; robotPose.angle=0;
    search.active=true; SearchEvadeAndResume(); instructionList.clear();
    obstacles.Clear(); obstacles.centralObstacle=true; obstacles.obstacleSensors=0b010; evasionStartTime=millis();
    search.active=true; StateActiveEvasion();
    CHECK(state!=SEARCH_APPROACH, "un candidato descartado hace poco en el mismo lugar no se vuelve a revisar");
    search.active=false; obstacles.Clear(); instructionList.clear(); state=STOP; missionActive=mA3; isEvading=false; step(2); }
  // FIN al volver a la base (sin otros robots vivos) y borrado de la memoria
  { bdie::Agent ag; bdie::GridConfig gc; ag.Init(5, gc);
    ag.StartMission(300, 300, 9, 1000); ag.bel.phase = bdie::PH_VOLVER; ag.bel.congregationClosed = true;
    ag.bel.targetKnown = true; ag.map.Deposit(30);
    ag.bel.x = 320; ag.bel.y = 310; ag.Cycle(2000);
    CHECK(ag.bel.phase == bdie::PH_FIN && !ag.bel.targetKnown && ag.map.CoverageCells() == 0,
          "al llegar a la base en VOLVER pasa a FIN y borra la memoria de la corrida"); }
  // Cada robot tiene su propio lugar en la base
  { bdie::GridConfig gc; float gxs[2], gys[2]; int ids[2]={2,3};
    for (int k=0;k<2;k++){ bdie::Agent ag; ag.Init(ids[k], gc); ag.StartMission(300,300,9,1000);
      ag.bel.phase=bdie::PH_VOLVER; ag.bel.x=900; ag.bel.y=700; ag.tClosed=0;
      bdie::Intention I; bdie::Action act; bdie::plans::bdVolverForm(ag, I, act); gxs[k]=I.goalX; gys[k]=I.goalY; }
    const float sep = std::hypot(gxs[0]-gxs[1], gys[0]-gys[1]);
    const float r0 = std::hypot(gxs[0]-300, gys[0]-300), r1 = std::hypot(gxs[1]-300, gys[1]-300);
    CHECK(sep > 150 && r0 < 200 && r1 < 200, "en el retorno cada robot va a su propio lugar alrededor de la base (dentro del radio de FIN)"); }
  // Fuera de sesión no se deposita feromona; la celda propia sigue a la pose
  { bdie::Agent ag; bdie::GridConfig gc; ag.Init(2, gc); ag.StartMission(300,300,9,1000); ag.AbortMission(2000);
    ag.OnPose(2, 520, 520, 0, 10); ag.OnPose(2, 820, 520, 0, 11);
    CHECK(ag.map.CoverageCells()==0 && ag.bel.myCell==bdie::MmToIndex(gc,820,520),
          "en SEGURO no se deposita feromona y la celda propia sigue a la pose"); }
  // Un robot apenas fuera de la cuadrícula no se queda sin meta
  { bdie::Agent ag; bdie::GridConfig gc; ag.Init(1, gc); ag.StartMission(300,300,9,1000);
    ag.bel.phase=bdie::PH_BUSCAR; ag.OnPose(1, -12, 90, 140, 12);
    bool celda = ag.bel.myCell == bdie::MmToIndexClamped(gc, -12, 90);
    bdie::Action a = ag.Cycle(5000); bool meta = false;
    for (int k=0;k<20 && !meta;k++){ a = ag.Cycle(5000 + 500*k); meta = (a.kind==bdie::Action::A_GOTO_CELL || ag.intention.goalCell>=0); }
    CHECK(celda && meta, "un robot 22 mm fuera de la cuadrícula toma la celda del borde y elige meta"); }
  // En búsqueda, el obstáculo central se revisa aunque otro robot diga «soy yo»
  { bool mA7=missionActive; missionActive=false; search.active=true; searchRejectMs=0;
    obstacles.Clear(); obstacles.centralObstacle=true; obstacles.obstacleSensors=0b010;
    obstacles.robotDetected=true; obstacles.fromRobotID="2"; g_out.clear();
    StateIdentifyObstacle(); bool aEvasion = (state==ACTIVE_EVASION);
    StateActiveEvasion();
    CHECK(aEvasion && state==SEARCH_APPROACH && sent("SEARCH|"),
          "en búsqueda el obstáculo central con un robot cerca se revisa igual (y deja traza SEARCH|)");
    searchCandAunqueRobot=false; search.active=true; obstacles.Clear(); obstacles.centralObstacle=true;
    obstacles.obstacleSensors=0b010; obstacles.robotDetected=true; state=IDENTIFY_OBSTACLE;
    StateIdentifyObstacle();
    CHECK(state==WAIT, "con NAV_CONFIG|CAND_ROBOT|0 espera al robot como el laboratorio");
    searchCandAunqueRobot=true; search.Reset(); search.active=false; obstacles.Clear(); instructionList.clear();
    state=STOP; missionActive=mA7; isEvading=false; step(2); }
  // Negociación de paso entre robots
  { CHECK(DecidirPaso(1,2,true)=='S' && DecidirPaso(2,1,true)=='C' && DecidirPaso(2,1,false)=='Q' &&
          DecidirPaso(1,2,false)=='Q', "regla de paso: quieto → QUIETO; en movimiento, el menor ID sigue y el mayor cede");
    // El que contesta: este robot es el 2. Pregunta el 1 desde 100 mm al frente, mirándolo.
    robots["1"] = IPAddress(192,168,1,10); bool pn=pasoNegociado;
    robotPose.x=500; robotPose.y=500; robotPose.angle=180; nav.Reset(); instructionList.clear();
    auto pregunta=[&](){ g_out.clear();
      std::array<String,6> a = {"CHECK_OBSTACLE","2","400","500","0",""}; HandleCheckObstacle(a); };
    pregunta(); bool q = sent("OBSTACLE_DETECTED|2|QUIETO");
    nav.Start(900,500); pregunta(); bool c = sent("OBSTACLE_DETECTED|2|CEDO");
    pasoNegociado=false; pregunta(); bool lab = sent("OBSTACLE_DETECTED|2") && !sentHas("OBSTACLE_DETECTED|2","|C") ;
    pasoNegociado=pn; nav.Reset();
    CHECK(q, "parado, contesta OBSTACLE_DETECTED|id|QUIETO");
    CHECK(c, "navegando y con ID mayor que el que pregunta, contesta CEDO");
    CHECK(lab, "con NAV_CONFIG|PASO|0 contesta como el laboratorio (sin tercer campo)");
    // El que pregunta: QUIETO o SIGO → evade ya; CEDO → espera como el laboratorio
    bool mA9=missionActive; missionActive=false; search.active=false;
    auto llega=[&](const char* paso){ obstacles.Clear(); obstacles.rightObstacle=true; obstacles.obstacleSensors=0b001;
      std::array<String,6> a = {"OBSTACLE_DETECTED","1",paso,"","",""}; HandleObstacleDetected(a);
      state=IDENTIFY_OBSTACLE; g_out.clear(); StateIdentifyObstacle(); return state; };
    auto dbg=debugUdp; debugUdp=1; bool evQ = (llega("QUIETO")==ACTIVE_EVASION); debugUdp=dbg;
    CHECK(evQ && sentHas("DEBUG","PASO: robot 1 QUIETO"),
          "si el otro está QUIETO evade sin esperar (y lo deja en el log)");
    CHECK(llega("SIGO")==ACTIVE_EVASION, "si el otro tiene el paso (SIGO) evade sin esperar");
    CHECK(llega("CEDO")==WAIT, "si el otro cede (CEDO) espera a que se aparte, como el laboratorio");
    CHECK(llega("")==WAIT, "una respuesta sin tercer campo (robot sin el cambio) espera como el laboratorio");
    // Dos respuestas: gana la más restrictiva, llegue en el orden que llegue
    obstacles.Clear(); obstacles.rightObstacle=true;
    { std::array<String,6> a={"OBSTACLE_DETECTED","3","SIGO","","",""}; HandleObstacleDetected(a);
      std::array<String,6> b={"OBSTACLE_DETECTED","4","CEDO","","",""}; HandleObstacleDetected(b); }
    bool r1 = obstacles.robotPaso=='S' && obstacles.fromRobotID=="3";
    obstacles.Clear();
    { std::array<String,6> b={"OBSTACLE_DETECTED","4","CEDO","","",""}; HandleObstacleDetected(b);
      std::array<String,6> a={"OBSTACLE_DETECTED","3","QUIETO","","",""}; HandleObstacleDetected(a); }
    bool r2 = obstacles.robotPaso=='Q' && obstacles.fromRobotID=="3";
    CHECK(r1 && r2, "con dos robots delante gana la respuesta más restrictiva (QUIETO/SIGO sobre CEDO)");
    pasoNegociado=false; CHECK(llega("QUIETO")==WAIT, "con NAV_CONFIG|PASO|0 espera siempre, como el laboratorio");
    pasoNegociado=pn;
    g_out.clear(); { std::array<String,6> a={"NAV_CONFIG","PASO","0","","",""}; HandleNavConfig(a); }
    bool apagado = !pasoNegociado && sent("NAV_CONFIG: paso_negociado=no");
    { std::array<String,6> a={"NAV_CONFIG","PASO","1","","",""}; HandleNavConfig(a); }
    CHECK(apagado && pasoNegociado, "NAV_CONFIG|PASO|0/1 lo apaga y lo enciende en vivo");
    obstacles.Clear(); instructionList.clear(); state=STOP; missionActive=mA9; isEvading=false; step(2); }
  // Medición de latencia entre robots. Este robot es el 2.
  { auto nav_cfg=[&](const char* n){ std::array<String,6> a={"NAV_CONFIG","LAT",n,"","",""}; HandleNavConfig(a); };
    g_out.clear(); LatTick(); bool apagado = !sent("LAT_PING");
    g_out.clear(); nav_cfg("1"); bool cfg = latPeriodoMs==1000 && sent("NAV_CONFIG: latencia=cada 1000 ms");
    g_out.clear(); LatTick(); bool p1 = sent("LAT_PING|") && sentHas("LAT_PING|", "|2") ;
    uint32_t s1 = latSeq;
    g_out.clear(); g_ms += 300; LatTick(); bool espera = !sent("LAT_PING");
    g_out.clear(); g_ms += 700; LatTick(); bool p2 = sent("LAT_PING|") && latSeq==s1+1 && latEnviados==2;
    CHECK(apagado && cfg, "NAV_CONFIG|LAT|1 enciende el ping cada 1000 ms (apagado por omisión)");
    CHECK(p1 && espera && p2, "un LAT_PING por período, por broadcast, con seq y el ID propio");
    // El que contesta: devuelve el ping de otro, no el propio ni en silencio
    g_out.clear(); { std::array<String,6> a={"LAT_PING","7","123456","3","",""}; HandleLatPing(a); }
    bool pong = sent("LAT_PONG|7|123456|2");
    g_out.clear(); { std::array<String,6> a={"LAT_PING","7","123456","2","",""}; HandleLatPing(a); }
    bool eco = !sent("LAT_PONG");
    uint32_t sil=bdieSilencioHasta; bdieSilencioHasta = g_ms + 5000;
    g_out.clear(); { std::array<String,6> a={"LAT_PING","8","1","3","",""}; HandleLatPing(a); }
    bool mudo = !sent("LAT_PONG"); bdieSilencioHasta = sil;
    CHECK(pong && eco && mudo, "LAT_PING de un compañero se devuelve (LAT_PONG con mi ID); el propio eco y el silencio no");
    // El que preguntó: rtt con su propio reloj y reporte con contadores acumulados
    char t0[24]; snprintf(t0, sizeof t0, "%lu", (unsigned long)(uint32_t)micros());
    char sq[12]; snprintf(sq, sizeof sq, "%lu", (unsigned long)latSeq);
    g_ms += 25; g_out.clear(); { std::array<String,6> a={"LAT_PONG",sq,t0,"3","",""}; HandleLatPong(a); }
    bool rep = sentHas("LAT|3|25.0|", "|2|1");
    g_out.clear(); { std::array<String,6> a={"LAT_PONG",sq,t0,"4","",""}; HandleLatPong(a); }
    bool otro = sentHas("LAT|4|", "|2|1");
    g_out.clear(); { std::array<String,6> a={"LAT_PONG","999",t0,"3","",""}; HandleLatPong(a); }
    bool ajeno = !sent("LAT|");
    CHECK(rep, "LAT_PONG → LAT|B|rtt_ms|enviados|recibidos a la base (rtt 25 ms medido con el reloj propio)");
    CHECK(otro && ajeno, "cada compañero lleva su contador; un pong con seq que no mandé se descarta");
    g_out.clear(); nav_cfg("0"); g_ms += 5000; LatTick();
    CHECK(latPeriodoMs==0 && !sent("LAT_PING") && sent("NAV_CONFIG: latencia=apagada"), "NAV_CONFIG|LAT|0 lo apaga");
    nav_cfg("50"); bool minimo = latPeriodoMs==200; nav_cfg("0");
    CHECK(minimo, "el período tiene un piso de 200 ms (no saturar el canal del enjambre)"); }
  // Tope de viaje en misión
  { bool mA4=missionActive; auto f4=agent.bel.phase; missionActive=true; agent.bel.phase=bdie::PH_BUSCAR;
    bdieOwnsMeet=false; robotPose.x=500; robotPose.y=500; nav.Start(500,1500);     // 1000 mm → 45 s
    BdieLoopHook(); g_out.clear(); g_ms += 30000; BdieLoopHook(); bool sigue = nav.isActive;
    g_ms += 16000; BdieLoopHook();
    CHECK(sigue && !nav.isActive && sent("WARNING: viaje a (500,1500)"),
          "un viaje de exploración que no llega en 15 s + 30 ms/mm se suelta y avisa");
    agent.bel.phase=f4; missionActive=mA4; nav.Reset(); instructionList.clear(); state=STOP; step(2); }
  // Viaje cortado junto a la meta en BUSCAR: gira hacia la meta y la revisa como candidato
  { bool mA8=missionActive; auto f8=agent.bel.phase; bool fs8=frontSensorInitialized;
    missionActive=true; agent.bel.phase=bdie::PH_BUSCAR; bdieOwnsMeet=false; bdieOwnsSearch=true;
    search.active=true; frontSensorInitialized=true; instructionList.clear();
    robotPose.x=1000; robotPose.y=300; robotPose.angle=0; nav.Start(1000,500);   // meta a 200 mm, a +90°
    BdieLoopHook(); g_out.clear(); g_ms += 22000; BdieLoopHook();
    bool cola = instructionList.size()==3 && instructionList[0][0]==TURN &&
                fabs(instructionList[0][1]-DegToArc(90))<0.5f && instructionList[2][0]==SEARCH_APPROACH &&
                instructionList[2][1] < 0 && state==READ_INSTRUCTION;
    CHECK(cola && sentHas("SEARCH|", "viaje cortado a 200 mm"), "un viaje cortado a < 300 mm de la meta gira hacia ella y la revisa (candidato)");
    instructionList.clear(); state=SEARCH_APPROACH; instructionValue=-1; search.steps=5; StateSearchApproach();
    CHECK(search.steps==0 || search.steps==1, "el acercamiento pedido desde la cola arranca con contadores en cero");
    search.Reset(); search.active=false; bdieOwnsSearch=false; frontSensorInitialized=fs8;
    agent.bel.phase=f8; missionActive=mA8; nav.Reset(); instructionList.clear(); state=STOP; step(2); }
  // El TGT que llega a mitad de un viaje suelta el cuerpo
  { auto f5=agent.bel.phase; bool tk=agent.bel.targetKnown; int ow=agent.bel.targetOwner;
    agent.bel.phase=bdie::PH_BUSCAR; agent.bel.targetKnown=false; agent.bel.targetOwner=-1;
    robotPose.x=500; robotPose.y=500; nav.Start(560,50);
    strcpy(receivedPacket, "TGT|7|3|976|371"); HandleBdieMessage();
    CHECK(agent.bel.phase==bdie::PH_CONGREGAR && !nav.isActive && state==STOP,
          "al enterarse del objetivo por mensaje deja el viaje de exploración y delibera");
    agent.bel.phase=f5; agent.bel.targetKnown=tk; agent.bel.targetOwner=ow; nav.Reset(); instructionList.clear(); step(2); }
  // En VOLVER, ya junto a la base, se suelta el viaje para poder cerrar
  { bool mA6=missionActive; auto f6=agent.bel.phase; bool se6=agent.bel.session;
    missionActive=true; agent.bel.phase=bdie::PH_VOLVER; agent.bel.session=true;
    agent.bel.baseX=260; agent.bel.baseY=350; robotPose.x=155; robotPose.y=405;
    nav.Start(260,350); g_ms += 4000; BdieLoopHook();
    CHECK(!nav.isActive && state==STOP, "en VOLVER a menos de 250 mm de la base suelta el viaje y delibera (FIN)");
    agent.bel.phase=f6; agent.bel.session=se6; missionActive=mA6; nav.Reset(); instructionList.clear(); step(2); }
  printf("\n%s (%d fallas)\n", fails? "FALLA":"TODO OK", fails);
  return fails?1:0;
}
