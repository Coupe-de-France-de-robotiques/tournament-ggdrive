////////////////// Setup //////////////////
void setup(){
	//initialisation des variables : initialisation des distances/angles et declaration des pins out/in puis lancement du parcours droit ou gauche [selon le pin utilisé pour brancher un certain fil sur arduino]
}

////////////////// Lecture des capteurs //////////////////
void lecture(){}

////////////////// Asservissement //////////////////
void aller(){
	//distance algebrique
	//Calcul de la consigne en vitesse : vitesse au milieu du chemain / vitesse à la fin pour ne pas depasser le pt d'arrivé
	//PID sur la translation : PID means faire avancer le robot pour réduire l'erreure entre la distance parcourue/son integrale/sa dérivée et la distance théorique/son integrale et sa dérivée.
}
void tourner(){
	//angle algebrique
	//calcul de la consigne en vitesse angulaire
	//PID sur la rotation
}
void tournerBalles(){
	//pour tourner le moteur des balles :)
}
void envoyerBalles(){
	//code pour enlever le frein des balles et les lancer + faire tourner le robot vers les boxs correspondant 
}
void envoyerBallesSales(){
	//idem sauf que les etapes sont un peu differentes : peut etre pour une autre phase du jeu
}
void sortirBras(){
	//sortir le bras du coté
}
void rentrerBras(){
	//ca j'ai pas compris ca fait quoi xD
}

////////////////// Gestion des moteurs /////////////////////
void leftMotor(long speed){
	//faire tourner moteur gauche à la vitesse : speed.
}
void rightMotor(long speed){
	//idem pour le right motor
}
void setMotorDir(int motor, int dir){
	//fonction annexe de setMotor prend en paramettre une moteur : gauche ou droite (0 ou 1) et une direction (0 : forward ou 1 : back) et fait avancer le moteur
}
void setMotor(int motor, int dir , int speed){
	//idem en ajoutant le speed du moteur : utilise setMotorDir.
}
void stopMotors(){
	//arrete les moteurs left et right
}
void stopAll(){
	//arrete tout quand on depasse 100s : temps qu'on a pour faire ce qu'on a à faire xD
}

////////////////// Calcul de position : mais ca sert pas à grand chose dans ce modele //////////////////
void actualisePos(float angleinit){
	//actualise la position à chaque fois : on lui fait appel dans aller et tourner.
}

////////////////// les parcours //////////////////
void parcoursDroit(){
	//etapes d'un certain parcours : commenté si on part de la droite. : fait appel à aller / tourner / envoyerBalles / sortirBras ect...
}
void parcoursGauche(){
	//etapes d'un certain parcours : commenté si on part de la gauche. : fait appel à aller / tourner / envoyerBalles / sortirBras ect...
}
bool timeOver(){
	//pour tester à chaque fois si le temps est fini : on lui fait appel avec stopAll dans aller et tourner 
}