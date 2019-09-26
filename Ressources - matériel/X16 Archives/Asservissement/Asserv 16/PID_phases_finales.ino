#include <Servo.h>

// les servomoteurs
Servo servoAmont;
Servo servoAval;
Servo servoBras;

// PIN couleur
const int pinCouleur = 34;

// PIN départ
const int pinDepart = 36;

// PINS servo
const int pinServoAmont = 4;
const int pinServoAval = 12;
const int pinServoBras = 7;
// variables de calcul globales
int compteurZeroDistance = 0; // compte depuis combie n d'itÃ©rations on a la bonne distance (+- 5 incrÃ©ments)
int compteurZeroAngle = 0; // compte depuis combien d'itÃ©rations on est au bon angle (+- 5 incrÃ©ments)

// pins d'entrÃ©e des codeurs incrÃ©mentaux des moteurs (inputs)
const int pin_G_A = 3; // fil jaune du moteur gauche (encodeur)
const int pin_G_B = 13;  // blanc gauche
const int pin_D_A = 2; // jaune droit
const int pin_D_B = 8; // blanc droit

// pin communication avec ST
const int ST = 10;

// pins de sortie pour rÃ©gler les moteurs (outputs)
#define DIR_R A2 // dÃ©finit la direction du moteur droit : Ã©crire 1 (HIGH) pour avancer, 0 (LOW) pour reculer
#define PWM_R 5 // dÃ©finit la vitesse du moteur droit : entre 0 et 255
#define DIR_L A1 // idem pour le gauche
#define PWM_L 6 // idem pour le gauche
#define PWM_lanceur 9 // PWM de la roue

// motor control constants (Ã  rÃ¨gler, peut Ãªtre inversÃ©es ?)
#define FORWARD 0
#define REVERSE 1
#define STOP 2
#define LEFT 0
#define RIGHT 1

// other constants
#define m_pi 3.14159

// robot parameters variables. InitialisÃ©es dans le setup, dÃ©pendent des roues et moteurs
int trackWidth, wheelDiameter, cpr; // trackWidth est l'espacement des roues

// current location variables
volatile long leftClicks, rightClicks; // la valeur actuelle du nombre d'incrÃ©ments sur chaque roue
long lastLeftClicks, lastRightClicks; // la valeur de leftClicks et rightClicks Ã  la derniÃ¨re itÃ©ration de loop
bool hasArrived; // sommes nous arrivÃ©s ?
bool collision;
float x,y, angle, angleinit, lastAngle; // la position globale du robot à chaque instant
long delta, lastDistParc; // utilisés pour mettre à jour la position en x et y


// erreur et position relative Ã  la cible
long erreurDistance, erreurAngle; // e rreurs relatives (distance - position)
long distanceTarget, angleTarget; // la distance et l'angle (en incrÃ©ments) jusqu'Ã  la cible depuis le point de dÃ©part
long distanceDer, angleDer; // les "dÃ©rivÃ©es" des erreurs de distance et d'angle
long diffAbsClicks, lastdiffAbsClicks, diffClicksDer, diffClicksInt; // pour le PID quand on tourne
long lastErreurAngle, lastErreurDistance; // les valeurs prÃ©cÃ©dentes
long distanceInt, angleInt; // les "intÃ©grales" des erreurs de distance de d'angle
long distanceParcourue;
long consigneVitesse ;
long vMaxActuelle;
float clicksTommAvancer;
float Krotation;

// coordinate PID.
const int kPcoord = 1500; // WARNING: the value is divided by 1000   anciennes valeurs: 1100, 4000, 0.65
const int kDcoord = 300; // idem
const int kIcoord = 10;    // idem

// Variables pour la consigne
float aMax;
float vMax, vMin, vRot, vFin; // valeur pour la croisiÃ¨re

float maxPWM; // valeur Ã  ne pas dÃ©passer
long timer; //en millisecondes
long startTime, totalTime, delayTime; // dÃ©but et temps total, delayTime est le temps entre 2 actions
long tempsMinTrajet;
long finAcceleration;
long distAcceleration;
int speedLanceur;

//variables pour l'ajustement de dernière minute
bool calage;
long calageTime;
long deltaV;
bool homologation;


// variables de comportement lobal du robot
long leftPWM, rightPWM; // vitesses transmises aux moteurs

// variables temporaires de calcul
int sign,signe;

// Tableau des distances par capteurs [centre,droit,gauche]
int dist[3];
int distMilieu, distDroite, distGauche;

void parcoursDroit();
void actualisePos(float angleInit);
bool timeOver();
void stopAll();
void stopMotors();
void leftMotor(long Pwm);
void rightMotor(long Pwm);
void sortirBras();
void rentrerBras();
void parcoursHomologationGauche();
void parcoursHomologationDroite();
void envoyerBalles();
void envoyerBallesSales();









/////////////////////////////////////////////// Incréments //////////////////////////////////////////////////////////////






void incr_right() {
  if (digitalRead(pin_D_B)) {
    // B is high so clockwise
    rightClicks ++;
  }

  else {
    // B is low so counter-clockwise
    rightClicks --;
  }
}

void incr_left() {
  if (digitalRead(pin_G_B)) {
    // -B is high so cclockwise
    leftClicks --;
  }

  else {
    // -B is low so clockwise
    leftClicks ++;
  }
}
 




///////////////////////////////////////////////// Setup /////////////////////////////////////////////////




void setup(){

  // initialisation des variables du robot:
  cpr = 293 ; // number of counts per revolution of the encoder // dÃ©pend de la charge !! 304 en théorie
  wheelDiameter = 73; // ENCODER wheel diameter in milimeters
  trackWidth = 278 ; // la constante qu'on rÃ¨gle pour la rotation

  // on initialise le temps
  delayTime = 200;

  // les servomoteurs
  servoAmont.attach(pinServoAmont);
  servoAval.attach(pinServoAval);
  servoBras.attach(pinServoBras);

  // intitialisation d'autres variables
  homologation = false;
  
  ;
  calage = false;
  calageTime = 4000;
  collision = false;
  leftPWM = 0;
  rightPWM = 0;
  leftClicks = 0;
  rightClicks = 0;
  lastErreurAngle = 0;
  lastErreurDistance = 0;
  deltaV = 4; // entre 0 et 4 maximum quand elle est chargée !!!!!!!!!!!!
  // on a mis 4 au bout de 4 runs. Resultat :
  vMax = 55 - deltaV;
  vMin = 40 - deltaV;
  vRot = 35 - int(0.6*deltaV);
  vFin = 30 - int(0.6*deltaV);
  maxPWM = vMax + 40;
  aMax = 0.1;
  finAcceleration = (long) vMax/aMax; // en ms
  distAcceleration = aMax*finAcceleration*finAcceleration/2;
  consigneVitesse = 0;
  hasArrived = false;
  clicksTommAvancer = (float)(wheelDiameter*m_pi/cpr); // expÃ©rimental, garder des .0 pour ne pas avoir de problÃ¨me
  Krotation = 0.91;
  speedLanceur = 52  - int(1.7*deltaV); // reglage de base : 51
  distMilieu = 6;
  distGauche = 6;
  distDroite = 6;

  //dÃ©lcaration des pins INPUT/OUTPUT
  pinMode(PWM_R,OUTPUT);
  pinMode(DIR_R,OUTPUT);
  pinMode(PWM_L,OUTPUT);
  pinMode(DIR_L,OUTPUT);
  pinMode(PWM_lanceur,OUTPUT);

  pinMode(pin_G_A, INPUT);
  pinMode(pin_G_B, INPUT);
  pinMode(pin_D_A, INPUT);
  pinMode(pin_D_B, INPUT);

  Serial.begin(9600); //I am using Serial Monitor instead of LCD display
  attachInterrupt(digitalPinToInterrupt(pin_D_A), incr_right, FALLING); // pour actualiser rightClicks en permanence
  attachInterrupt(digitalPinToInterrupt(pin_G_A), incr_left, FALLING); // idem

  // Initialisation des distances
  dist[0] = 15;
  dist[1] = 15;
  dist[2] = 15;
  Serial1.begin(9600);

  Serial.setTimeout(1000); // apparamment nécessaire

  // seros fermés
  servoAmont.write(180);
  servoBras.write(15);

  delay(200);

  if (homologation){
  }

  else {
    while(!depart()){
      delay(100);
    }

    totalTime = millis();
    
    if (couleur()){
      parcoursGauche();
    }
    else{
      parcoursDroit();
    }
  }
  
}

/////////////////////////////////////////////////// loop, non utilisée /////////////////////////////////////////////////////

void loop(){
}


//////////////////////////////////////////////////// Lecture capteurs //////////////////////////////////////////////////////

void lecture(){
  while (Serial1.available()) {
    int id;
    int check;
    int range;
    int inByte = Serial1.read();
    
    id = inByte % 4;
    inByte = inByte / 4;
    check = inByte % 4;
    range = inByte /4;

    /*
    Serial.print(id);
    Serial.print(" ");
    Serial.print(check);
    Serial.print(" ");
    Serial.print(range);
    Serial.print(" ");
    Serial.print(range % 4 == check);
    Serial.write("\n");*/

    if (range % 4 == check){
      dist[id]=range;
    }
  }
}


///////////////////////////////////////////////// Couleur ///////////////////////////////////////////////////////////////

bool couleur(){
  // 0 pour vert et 1 pour orange
  return digitalRead(pinCouleur);
}

/////////////////////////////////////////////// Départ //////////////////////////////////////////////////////////////////

bool depart(){
  return !digitalRead(pinDepart);
}

// ********************************************************************************************************************** //
/////////////////////////////////////////////////////// Asservissement /////////////////////////////////////////////////////
// ********************************************************************************************************************** //





void aller(float distance) {

  hasArrived = false;
  leftClicks = 0;
  rightClicks = 0; // rÃ©initialiser les compteurs
  startTime = millis();
  erreurDistance = 0;
  angleinit = angle;
  lastErreurAngle = 0;
  angleInt = 0;

  // distance peut Ãªtre positif (vers l'avant) ou nÃ©gatif (vers l'arriÃ¨re).
  signe = -1;     // les moteurs sont montés à l'envers
  if (distance<0) {
    signe = 1;
  }
  distanceTarget = abs(distance/clicksTommAvancer); // distanceTarget est en Clicks et positif

  while (!hasArrived){

    timer = millis()-startTime;
    distanceParcourue = abs((leftClicks+rightClicks)/2); //positif
    actualisePos(angleinit);

    if (timeOver()){
        stopAll();
    }

    if(millis()-startTime>10000){ // s'il est bloqué, il passe à l'action suivante
      hasArrived = true;
    }

    if ((calage)&&(millis()-startTime>calageTime)){
      hasArrived = true;
    }

    ////////////Test !!!! //////////

    lecture();
    collision = dist[0] < distMilieu || dist[1] < distDroite || dist[2] <distGauche;

    Serial.print(dist[0]);
    Serial.write(" ");
    Serial.print(dist[1]);
    Serial.write(" ");
    Serial.print(dist[2]);
    Serial.write("\n");

    ////////// Fin test ////////////


    
    if (collision){
      stopMotors();
      while (collision){
        delay(100);

        lecture();
        collision = dist[0] < distMilieu || dist[1] < distDroite || dist[2] <distGauche;
        Serial.print(dist[0]);
        Serial.write(" ");
        Serial.print(dist[1]);
        Serial.write(" ");
        Serial.print(dist[2]);
        Serial.write("\n");
      }
 
      
      distanceTarget = distanceTarget-distanceParcourue;
      leftClicks = leftClicks-signe*distanceParcourue;
      rightClicks = rightClicks-signe*distanceParcourue;
      distanceParcourue = 0;
    }
    

    ///////// Calcul de la consigne en vitesse //////////////////////////////

    // la consigne (rampe vitesse up puis plateau puis down) en vitesse
    if (distanceParcourue<0.3*distanceTarget){
      consigneVitesse = vMin + min(aMax*timer,vMax-vMin);
      vMaxActuelle = consigneVitesse;
    }
    else if (distanceParcourue<0.85*distanceTarget){
      consigneVitesse = vMaxActuelle;
    }
    else if (distanceParcourue>0.85*distanceTarget){
      consigneVitesse = vMaxActuelle*(distanceTarget-distanceParcourue)/(0.15*distanceTarget);
    }

    // la consigne en Vitesse quand on est très proches (ou si on a dépassé).
    if (abs(consigneVitesse)<vMin){
      consigneVitesse = vMin;
      if ((distanceTarget-distanceParcourue)<0){
        consigneVitesse = -consigneVitesse;
      }
    }

    /////////// Calcul de l'erreur en angle /////////////////////

    // Intégrale de l'erreur
    if(abs(angleInt+erreurAngle)<2000){ // pourquoi cette condition ???
      angleInt += erreurAngle; // "l'intÃ©grale"
    }

    // Dérivée de l'erreur
    angleDer = erreurAngle - lastErreurAngle;

    /////////// Condition d'arrivée  /////////////////////////

    if((distanceTarget-distanceParcourue<10)&&(abs(erreurAngle)<5)){
      hasArrived = true;
    }

    lastErreurAngle = erreurAngle;
    erreurAngle = signe*(rightClicks-leftClicks);
    if (abs(erreurAngle)>30){
      erreurAngle = 30 * erreurAngle/abs(erreurAngle);
    }
    /////// le PID ///////////////////////////////////


    leftPWM = signe*(consigneVitesse + (kPcoord * erreurAngle + kDcoord * angleDer + kIcoord*angleInt) / 1000 );
    rightPWM = signe*(consigneVitesse + (-kPcoord *erreurAngle - kDcoord * angleDer - kIcoord * angleInt) / 1000) ;

    leftMotor(leftPWM);
    rightMotor(rightPWM);

    // les prints ///
    //Serial.print("   Erreur Angle : ");
    //Serial.print(erreurAngle);
    //Serial.print("   DistClicks : ");
    //Serial.print(distanceParcourue);
    //Serial.print("   Distance : ");
    //Serial.println(distanceParcourue*clicksTommAvancer);


  }

  //Serial.print("Distance th en clicks : ");
  //Serial.println(distanceTarget);
  
  stopMotors();
  delay(delayTime);
}


void tourner(float angleTarget) { //angle en degrÃ©s, sens trigo positif
  hasArrived = false;
  leftClicks = 0;
  rightClicks = 0; // rÃ©initialiser les compteurs
  diffClicksInt = 0;
  lastdiffAbsClicks = 0;
  compteurZeroAngle = 0;
  angleinit = angle;
  startTime = millis();

  angleTarget = angleTarget*m_pi/180*trackWidth/clicksTommAvancer*Krotation; // en incréments
  signe = 1;
  if (angleTarget<0){
    signe = -1;
  }

  while (!hasArrived){

    if (timeOver()){
       stopAll();
    }

    if(millis()-startTime>10000){ // s'il est bloqué, il passe à l'action suivante
      hasArrived = true;
    }

    actualisePos(angleinit);
    erreurAngle = angleTarget-(rightClicks-leftClicks); // possiblement négatif
    consigneVitesse = vRot*signe;
    if (abs(erreurAngle)< 0.7*abs(angleTarget)){
      consigneVitesse = (vFin +(vRot-vFin)*erreurAngle/(0.7*angleTarget)) * signe;
    }
    diffAbsClicks = signe*(abs(rightClicks)-abs(leftClicks));

    diffClicksDer = diffAbsClicks-lastdiffAbsClicks;
    if (diffClicksInt+diffAbsClicks<2000){
      diffClicksInt +=diffAbsClicks;
    }
    lastdiffAbsClicks = diffAbsClicks;

    if (signe*erreurAngle<1){
      hasArrived=true;
    }
    if (compteurZeroAngle>20) {
      hasArrived = true;
    }

    // correction proportionnelle
    leftPWM = -(consigneVitesse + diffAbsClicks + 0.6*diffClicksDer + 0.005*diffClicksInt);
    rightPWM = consigneVitesse - diffAbsClicks - 0.6*diffClicksDer - 0.005*diffClicksInt;

    // setting the speed and direction parameters for the motors
    leftMotor(leftPWM);
    rightMotor(rightPWM);
  }

  stopMotors();
  delay(delayTime);
}

void tournerBalles(float angleTarget){
  int temp = vRot;
  int tempbis = vFin;
  vRot = 50 - deltaV;
  vFin = 80 - 2*deltaV;
  
  tourner(angleTarget);

  vRot = temp;
  vFin = tempbis; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Actions spéciales ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// envoie les 8 balles in the basket
void envoyerBalles(){
  analogWrite(PWM_lanceur,speedLanceur);
  delay(900);
  servoAmont.write(120);
  long delayCopy = delayTime;
  delayTime = 1;  
  tourner(3);
  tourner(-3);
  tourner(3);
  tourner(-3);
  delay(250);
  speedLanceur += 1;
  delay(250);
  speedLanceur += 1;
  delay(250);
  speedLanceur += 1;
  delay(250);
  speedLanceur += 2;
  delay(300);
  speedLanceur += 1;
  delay(250);
  speedLanceur += 1;
  delay(250);
  speedLanceur += 1;
  delay(250);
  speedLanceur += 2;
  delay(300);
  speedLanceur += 1;
  delay(300);
  servoAmont.write(180);
  speedLanceur -= 11;
  tourner(3);
  tourner(-3);
  tourner(3);
  tourner(-3);
  servoAmont.write(120);
  speedLanceur -= 8;
  delay(500);
  speedLanceur += 2;
  delay(500);
  speedLanceur += 2;
  delay(500);
  speedLanceur += 2;
  delay(500);
  speedLanceur += 2;
  delay(500);
  analogWrite(PWM_lanceur,0);
  delayTime = delayCopy;
  servoAmont.write(180);
  delay(delayTime);
}

void envoyerBallesSales(){
  speedLanceur = 25;
  analogWrite(PWM_lanceur,speedLanceur);
  delay(1200);
  servoAmont.write(120);
  long delayCopy = delayTime;
  delayTime = 1;  
  tourner(3);
  tourner(-3);
  tourner(3);
  tourner(-3);
  delay(100);
  speedLanceur += 1;
  delay(100);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 2;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 2;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 2;
  delay(200);
  speedLanceur += 2;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 2;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 2;
  delay(200);
  speedLanceur += 1;
  delay(200);
  speedLanceur += 1;
  delay(500);
  analogWrite(PWM_lanceur,0);
  delayTime = delayCopy;
  servoAmont.write(180);
  delay(delayTime);
}


void sortirBras(){
  servoBras.write(86); // avant 90
}

void rentrerBras(){
  servoBras.write(15);
}

///////////////////////////////////////// Gestion des moteurs ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void leftMotor(long speed){
  // speed can be negative, above maximal values etc...
  sign = speed/abs(speed);
  if (abs(speed)>maxPWM){
    speed = sign*maxPWM;
  }
  if (speed < 0) setMotor(LEFT, REVERSE, -speed);
  if (speed>=0) setMotor(LEFT, FORWARD, speed);
}

void rightMotor(long speed){
  // speed can be negative, above maximal values etc...
  sign = speed/abs(speed);
  if (abs(speed)>maxPWM){
    speed = sign*maxPWM;
  }
  if (speed < 0) setMotor(RIGHT, REVERSE, -speed);
  if (speed>=0) setMotor(RIGHT, FORWARD, speed);
}

void setMotorDir(int motor, int dir){
  // rÃ¨gle DIR_L ou DIR_R qui dÃ©cide du sens de rotation du moteur. Fonction annexe de setMotor.
  // motor vaut RIGHT(1) ou LEfT (0)
  // dir vaut 0 (ou FORWARD) vers l'avant, 1 (ou REVERSE) vers l'arriÃ¨re, 2 (ou STOP) pour stop
  if (motor == LEFT)
  {
    if (dir == 0) digitalWrite(DIR_L, HIGH);//      FORWARD
    else if (dir == 1) digitalWrite(DIR_L, LOW);//REVERSE
    else if (dir == 2) digitalWrite(DIR_L, LOW);//STOP
  }
  else if (motor == RIGHT)
  {
    if (dir == 0) digitalWrite(DIR_R, HIGH);
    else if (dir == 1) digitalWrite(DIR_R, LOW);
    else if (dir == 2) digitalWrite(DIR_R, LOW);
  }
}

void setMotor(int motor, int dir, int speed){
  // rÃ¨gle la vitesse et la direction du moteur (motor vaut LEFT ou RIGHT)
  // dir vaut 0 vers l'avant, 1 vers l'arriÃ¨re, 2 pour stop
  setMotorDir(motor, dir);
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  if (motor == LEFT) analogWrite(PWM_L, speed);
  else if (motor == RIGHT) analogWrite(PWM_R, speed);
}

void stopMotors(){
  analogWrite(PWM_R,0); //stopMotor
  analogWrite(PWM_L,0); //idem
}

// Quand on a fait plus de 100 s
void stopAll(){
    stopMotors();
    analogWrite(PWM_lanceur,0);

    while(true){
        delay(1000);
    }
}

/////////////////////////////////////////// Calcul de position //////////////////////////////////////////////////

// Calcule la position du robot en coordonnées (x,y)
// On ne l'utilise pas
void actualisePos( float angleinit){
  angle = angleinit + clicksTommAvancer * ( rightClicks - leftClicks) / trackWidth *180 / m_pi/Krotation; // en degrÃ©s
  //Serial.println(angle);
  delta = (leftClicks + rightClicks)/2 - lastDistParc;
  lastDistParc = (leftClicks + rightClicks)/2;
  x = x + clicksTommAvancer*delta * sin(angle/180*m_pi); // en mm
  y = y + clicksTommAvancer*delta * cos(angle/180*m_pi); // en mm
  /*Serial.print("x  ");
  Serial.println(x);
  Serial.print("y  ");
  Serial.println(y);*/
}

//////////////////////////////////////////////// Les 2 parcours possibles /////////////////////////////////////////////

// le parcours si on part de droite (coté vert ?)
void parcoursDroit(){

  speedLanceur += 1;

  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;

  aller(480);
  tourner(6);
  aller(84);
  tourner(84);
  delay(2000);
  long delayCopy = delayTime;
  delayTime = 1;
  tourner(3);
  tourner(-3);
  tourner(3);
  tourner(-3);
  delayTime = delayCopy;
  delay(1500);
  tourner(6);
  delay(2000);

  distMilieu = 2;
  distDroite = 3;
  distGauche = 0;
  
  aller(240);
  tourner(-55); // on se tourne vers la cible
  envoyerBalles();
  delay(2000);
  tourner(57);
  aller(190);
  tourner(-83);// vers l'abeille

  distMilieu = 4;
  distDroite = 4;
  distGauche = 4;
  
  aller(700); //
  tourner(-84); // virage à droite vers la zone de calage

  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;
  
  calage = true;
  calageTime = 4000;
  deltaV = 9;
  aller(800);
  aller(-45);
  tourner(93);
  calageTime = 3000;
  aller(500);
  aller(-15); 
  calage = false;
  deltaV = 4;

  // taper l'abeille
  tourner(-135);
  sortirBras();
  delay(500);
  tourner(72);
  tourner(-20);
  aller(-100); 
  rentrerBras();

  // chemin vers l'interrupteur
  distMilieu = 4;
  distDroite = 4;
  distGauche = 4;
  
  tourner(-133);
  aller(1100);
  tourner(46);

  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;

  // interrupteur
  calage = true;
  calageTime = 6500;
  aller(1000);
  aller(-40);

  // on reessaye à gauche
  calageTime = 2000;
  tourner(90);
  aller(125);
  tourner(-88);
  aller(100);
  aller(-40);
  
  calage = false;
  
  distMilieu = 4;
  distDroite = 5;
  distGauche = 5;

  // on va vers le recuperateur adverse
  tourner(-88);
  aller(125);
  tourner(-41);
  aller(1870);
  tourner(-38);

  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;
  
  calage = true; // on se cale contre le mur en face d'eau sale orange
  calageTime = 3000;
  aller(1000);
  aller(-30);
  tourner(-89);
  aller(500);
  aller(-30);
  tourner(-72); // on tourner pour ouvrir le loquet
  delay(1500);
  delayTime = 1;
  tourner(-3);
  tourner(3);
  tourner(-3);
  tourner(3);
  delayTime = delayCopy;
  delay(1000);
  tourner(-76); // on se place vers la station d'épuration
  aller(-60);
  envoyerBallesSales(); // on nique RCVA
  delay(1000);
  sortirBras();
  rentrerBras();
  sortirBras();
  rentrerBras(); 
}


// le parcours si on part de gauche
void parcoursGauche(){
  
  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;
  
  aller(550);
  tourner(-4);
  aller(24);
  tourner(-82);
  delay(2000);
  long delayCopy = delayTime;
  delayTime = 1;
  tourner(-3);
  tourner(3);
  tourner(-3);
  tourner(3);
  delayTime = delayCopy;
  delay(1500);
  tourner(-3);
  delay(2000);

  distMilieu = 2;
  distDroite = 3;
  distGauche = 0;
  
  aller(240);
  tourner(88); // on se tourne vers la cible
  envoyerBalles();
  delay(2000);
  tourner(-76);
  aller(190);
  tourner(94);// vers l'abeille

  distMilieu = 4;
  distDroite = 4;
  distGauche = 4;
  
  aller(700); //
  tourner(94); // virage à droite vers la zone de calage

  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;
  
  calage = true;
  calageTime = 5000;
  deltaV = 9;
  aller(800);
  aller(-35);
  tourner(-85);
  calageTime = 4000;
  aller(500);
  aller(-15); 
  calage = false;
  deltaV = 4;

  // taper l'abeille
  tourner(-45); // à adapter en testant
  sortirBras();
  delay(1000);
  tourner(-65);
  tourner(18);
  delay(200);
  aller(100); 
  rentrerBras();
  delay(1000);

  // chemin vers l'interrupteur
  distMilieu = 4;
  distDroite = 4;
  distGauche = 4;
  
  tourner(-36);
  aller(1280);
  tourner(-41);
 
  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;

  // interrupteur
  calage = true;
  calageTime = 6500;
  aller(850);
  aller(-40);

  // on essaie plus à droite
  calageTime = 2000;
  tourner(-89);
  aller(125);
  tourner(90);
  aller(100);
  aller(-40);
  
  calage = false;

  // on va vers le recuperateur adverse
  tourner(91);
  aller(125);
  tourner(48);

  distMilieu = 4;
  distDroite = 5;
  distGauche = 5;
  
  aller(1670);
  tourner(41);

  distMilieu = 0;
  distDroite = 0;
  distGauche = 0;
  
  calage = true; // on se cale contre le mur en face d'eau sale orange
  calageTime = 3000;
  aller(1000); // calage numero 1
  aller(-40);
  tourner(91);
  aller(500);
  aller(-55); // on a fait les deux calages
  tourner(78); // on tourner pour ouvrir le loquet
  delay(1500);
  delayTime = 1;
  tourner(3);
  tourner(-3);
  tourner(3);
  tourner(-3);
  delayTime = delayCopy;
  delay(1000);
  tourner(91); // on se place vers la station d'épuration
  aller(-70);
  envoyerBallesSales(); // on nique RCVA
  delay(1000);
  sortirBras();
  rentrerBras();
  sortirBras();
  rentrerBras();
}



// pour savoir si on a fait plus de 100s
bool timeOver(){
    if (millis()-totalTime>99500) {
        return true;
    }
    else {
        return false;
    }
}

