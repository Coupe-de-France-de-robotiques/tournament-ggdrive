//#include <SimpleTimer.h>
#include <digitalWriteFast.h>

#define m_pi  3.14159265359

//Constante pour les fonctions des moteurs
#define LEFT 0
#define RIGHT 1
#define REVERSE 1
#define FORWARD 0
#define IRsensor0 A0
#define IRsensor1 A1
#define IRsensor2 A2
#define IRsensor3 A3

// robot parameters variables. Initialisées dans le setup, dépendent des roues et moteurs
int trackWidth, wheelDiameter, cpr; // trackWidth est l'espacement des roues, cpr = counts per revolution

// current location variables
volatile long leftTicks, rightTicks; // la valeur actuelle du nombre d'incrÃ©ments sur chaque roue
long lastLeftTicks, lastRightTicks; // la valeur de leftTicks et rightTicks Ã  la derniÃ¨re itÃ©ration de loop
bool hasArrived; // sommes-nous arrivés ?

float x_Pos,y_Pos, theta; // la position globale du robot à chaque instant
long delta, lastDistParc; // utilisés pour mettre à jour la position en x et y


// erreur et position relative Ã  la cible
long erreurDistance, erreurAngle; // erreurs relatives (distance - position)
long distanceTarget, angleTarget; // la distance et l'angle (en incréments) jusqu'à la cible depuis le point de départ
long distanceDer, angleDer; // les "dérivées" des erreurs de distance et d'angle
//long diffAbsClicks, lastdiffAbsClicks, diffClicksDer, diffClicksInt; // pour le PID quand on tourne
long lastErreurAngle, lastErreurDistance; // les valeurs précédentes
long distanceInt, angleInt; // les "intégrales" des erreurs de distance de d'angle
long distanceParcourue;
long consigneVitesse ;
long vMaxActuelle;
float TickTomm;

// constant PID.
const float Kp = 1; 
const float Kd = 0; 
const float Ki = 0;    

const float Kap = 0.1;
const float Kad = 1;
const float Kai = 0.00001;

// Variables pour la consigne
float aMax;
float vMax, vMin, vRot, vFin; // valeur pour la croisière

float maxPWM; // valeur à  ne pas dépasser
long timer; //en millisecondes
long startTime, totalTime, delayTime; // début et temps total, delayTime est le temps entre 2 actions
long tempsMinTrajet;
long finAcceleration;
long distAcceleration;
int speedLanceur;

long deltaV;

// variables de comportement global du robot
long leftPWM, rightPWM; // vitesses transmises aux moteurs

//Pins
//For encoders
#define pin_D_B  24 // B fil BLANC de l'encodeur
#define pin_D_A  18 // A fil VERT  de l'encodeur
#define pin_G_B  25
#define pin_G_A  19


//Pour le moteur
int PWM_R = 5;
int PWM_L = 7;
int DIR_R = 6; 
int DIR_L = 8;


int sign;
int distanceIR;
///////////////// Des méthodes pour incrémenter les ticks de l'encodeur
void incr_right() {
  if(digitalReadFast(pin_D_B)) {
    // B is high so clockwise
    rightTicks++;
  }
  else {
    // B is low so counter-clockwise
    rightTicks--;
  }
  //Serial.println("right"); //Pour tester s'il affiche la bonne valeur
  //Serial.println(rightTicks);
}
void incr_left() {
  if (digitalReadFast(pin_G_B)) {
    // -B is high so cclockwise
    leftTicks--;
  }
  else {
    // -B is low so clockwise
    leftTicks ++;
  }
  //Serial.println("left");
  //Serial.println(leftTicks);
}
///////////////////////////////////////////////




void setup() {
  Serial.begin(9600);
  
  // initialisation des variables du robot:
  cpr = 600 ; // number of counts per revolution of the encoder // dépend de la charge !!
  wheelDiameter = 57; //ENCODER wheel diameter in milimeters
  trackWidth = 274  ; //la constante qu'on règle pour la rotation en milimètre

  // on initialise le temps
  delayTime = 500;

  leftPWM = 0;
  rightPWM = 0;
  leftTicks = 0;
  rightTicks = 0;
  lastRightTicks = 0;
  lastLeftTicks = 0;
  lastErreurAngle = 0;
  lastErreurDistance = 0;
  
  ///////////////
  vMax = 55 ;
  vMin = 10 ;
  aMax = 0.1;
  
  maxPWM = vMax + 40;
  finAcceleration = (long) vMax/aMax; // en ms
  distAcceleration = aMax*finAcceleration*finAcceleration/2;
  
  consigneVitesse = 0;
  hasArrived = false;
  TickTomm = (float)(wheelDiameter*m_pi/cpr); // expérimental, garder des .0 pour ne pas avoir de problème
  
  

  ////////////////////////
  pinMode(PWM_R,OUTPUT);
  pinMode(DIR_R,OUTPUT);
  pinMode(PWM_L,OUTPUT);
  pinMode(DIR_L,OUTPUT);
  

  pinMode(pin_G_A, INPUT);
  pinModeFast(pin_G_B, INPUT);
  pinMode(pin_D_A, INPUT);
  pinModeFast(pin_D_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(pin_D_A), incr_right, FALLING); // pour actualiser rightTicks en permanence
  attachInterrupt(digitalPinToInterrupt(pin_G_A), incr_left, FALLING); // idem

  //IR.setInterval(100, DistanceIR);
}


void loop() { 



 
  delay(1000);
  aller(500);
  tourner(180);
  aller(500);
  tourner(180);
  stopAll();
  
}


//Pour aller tout droit

void aller(float distance) {
  delay(500);
  hasArrived = false;
  leftTicks = 0;
  rightTicks = 0; // réinitialiser les compteurs
  startTime = millis();
  
  lastErreurAngle = 0;
  angleInt = 0;

  // 
  distanceTarget = abs(distance/TickTomm); // distanceTarget est en Ticks et positif

  while (!hasArrived)
  {

    DistanceIR_ArretUrgence();
    
    updateOdometry();
    
    Serial.print(erreurAngle);
    Serial.println(" ");
    timer = millis()-startTime;
   
    distanceParcourue = abs((leftTicks+rightTicks)/2); //positif
    

    ///////// Calcul de la consigne en vitesse //////////////////////////////

    // la consigne (rampe vitesse up puis plateau puis down) en vitesse

    consigneVitesse = min(vMax, 2*sqrt(distanceTarget-distanceParcourue));

    /*
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
    */

    // la consigne en Vitesse quand on est très proches (ou si on a dépassé).
    if (abs(consigneVitesse)<vMin){
      consigneVitesse = vMin;
      /////
      if ((distanceTarget-distanceParcourue)<0){
        consigneVitesse = -consigneVitesse;
      }
    }

    /////////// Calcul de l'erreur en angle /////////////////////
    //On n'a pas utilisé timer

    erreurAngle = (rightTicks-leftTicks); //L'orientation du robot ---- l'angle de destination par rapport à robot(=0)
/*
    if (abs(erreurAngle)>30){
      erreurAngle = 30 * erreurAngle/abs(erreurAngle);
*/
    // Intégrale de l'erreur
    angleInt += erreurAngle; 
    
    // Dérivée de l'erreur
    angleDer = erreurAngle - lastErreurAngle;

    /////////// Condition d'arrivée  /////////////////////////

    if((distanceTarget-distanceParcourue<50)&&(abs(erreurAngle)<50)){
      hasArrived = true;
    }

    lastErreurAngle = erreurAngle;
    /////
   
    /////// le PID ///////////////////////////////////


    leftPWM = consigneVitesse + (Kp * erreurAngle + Kd * angleDer + Ki*angleInt)  ;
    rightPWM = consigneVitesse - (Kp *erreurAngle + Kd * angleDer + Ki*angleInt)  ;
    //En fait Kp = Kp *C
    // Ki = Ki * C * SamplingTime
    // Kd = Kd *C/SamplingTime
    // C = mmToTick/2
    
    leftMotor(leftPWM);
    rightMotor(rightPWM);
    }  
    stopMotors();
}

void tourner(float angle){ //angle is in degree
  
  hasArrived = false;
  leftTicks = 0;
  rightTicks = 0; // réinitialiser les compteurs
  startTime = millis();
  
  lastErreurAngle = 0;
  angleInt = 0;

  angleTarget = angle*m_pi/180*trackWidth/TickTomm; //dimension en ticks
  
  while(!hasArrived){
    updateOdometry();
    Serial.print(erreurAngle);
    Serial.println(" ");
    erreurAngle = angleTarget - (rightTicks - leftTicks);

    angleInt += erreurAngle;

    angleDer = erreurAngle - lastErreurAngle;

    if(abs(erreurAngle) < 5){
        hasArrived = true;
      }
    lastErreurAngle = erreurAngle;


    
    leftPWM = -(Kap * erreurAngle + Kad * angleDer + Kai*angleInt);
    /*if(abs(leftPWM < vMin)){
      leftPWM = +- vMin;
    }*/
    rightPWM = + (Kap *erreurAngle + Kad * angleDer + Kai*angleInt);

    leftMotor(leftPWM);
    rightMotor(rightPWM);
    }
}

///////////////////////////////Gestion des moteurs
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
  // règle DIR_L ou DIR_R qui dÃ©cide du sens de rotation du moteur. Fonction annexe de setMotor.
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
  // règle la vitesse et la direction du moteur (motor vaut LEFT ou RIGHT)
  // dir vaut 0 vers l'avant, 1 vers l'arrière, 2 pour stop
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
    while(true){
        delay(1000);
    }
}
//////////////

void updateOdometry(){
  //theta est en radian, les positions sont en mm
  double D_r = TickTomm * (rightTicks - lastRightTicks);
  double D_l = TickTomm * (leftTicks - lastLeftTicks);
  double D_c = (D_r + D_l)/2;

  theta += (D_r - D_l)/trackWidth;
  x_Pos += D_c*cos(theta);
  y_Pos += D_c*sin(theta);

  lastRightTicks = rightTicks;
  lastLeftTicks = leftTicks;
}

void calculerDistanceMinSensors(){
  float volts = analogRead(IRsensor0)*0.0048828125;  // value from sensor * (5/1024)
  distanceIR = 13*pow(volts, -1); // worked out from datasheet graph
}

void DistanceIR_ArretUrgence(){

   calculerDistanceMinSensors();
  
  // Si un obstacle se trouve a une distance dangereuse :
  if(distanceIR < 10 && distanceIR >=4){
    stopMotors();
    
    while(distanceIR < 12 && distanceIR >= 4){ // La condition est distanceIR < 12 (et non pas 10) pour attendre que l'obstacle se soit un peu eloigne
    delay(50);
    // Et on recalcule la distance a l'obstacle pour savoir si on sort de la boucle :
    calculerDistanceMinSensors();
    }
  }
  
}

