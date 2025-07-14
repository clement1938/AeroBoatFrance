#include <Servo.h>

// On créé les objets servo sur tt ce qui est en PWM
Servo moteur1;
Servo moteur2;
Servo moteur3;
Servo moteur4;
Servo derive5;
Servo switch6;
double moteur = 0;
double m1=0;
double m2=0;
double m3=0;
double m4=0;
double derive = 0;
double PB=0;

void setup() {
  
  /*
  // Pour le test sortie sur console
  Serial.begin(115200);
  Serial.println("Prêt à recevoir des données");
  */
  moteur1.attach(1);  // attaches the servo on pin 1 to the moteur1 object avant gauche 
  moteur2.attach(2);  // attaches the servo on pin 2 to the moteur2 object aile gauche
  moteur3.attach(3);  // attaches the servo on pin 3 to the moteur3 object avant droit
  moteur4.attach(4);  // attaches the servo on pin 4 to the moteur4 object aile droite
  derive5.attach(5);  // attaches the servo on pin 5 to the servo object derive
  switch6.attach(6);  // attaches the servo on pin 6 to the servo object switch

  pinMode(7,INPUT);  // attaches the  PWM  on pin 7, Intensité des gaz
  pinMode(8,INPUT);  // attaches the servo on pin 8, Dérive
  pinMode(9,INPUT);  // attaches the servo on pin 9, Switch
  delay (100);

}

void loop() {

  moteur=pulseIn(7,HIGH);
  moteur=((moteur-983)/1019)*180;

  derive=pulseIn(8,HIGH);
  derive=((derive-1498)/1019)*180;

  PB=pulseIn(9,HIGH);
  PB=((PB-1300)/702)*180;

  m1=moteur;
  m2=moteur;
  m3=moteur;
  m4=moteur;


  if(PB>50)
  {
  m1=moteur;
  m2=moteur+derive;
  m3=moteur;
  m4=moteur-derive;
  
  if (m2<0){
    m2=0;
  }

  if (m2>180){
    m2=180;
  }

  if (m4<0){
    m4=0;
  }

  if (m4>180){
    m4=180;
  }

  }

  moteur1.write(m1);
  moteur2.write(m2);
  moteur3.write(m3);
  moteur4.write(m4);
  derive5.write(90+derive);
  switch6.write(PB);
/*
  // Test sortie sur console
  Serial.print("m2 = ");
  Serial.print(m2);
  Serial.print(",");
  Serial.print("m1 = ");
  Serial.print(m1);
  Serial.print(",");
  Serial.print("m3 = ");
  Serial.print(m3);
  Serial.print(",");
  Serial.print("m4 = ");
  Serial.print(m4);
  Serial.print(",");

  Serial.print("derive = ");
  Serial.print(derive);
  Serial.print(",");

  Serial.print("PB = ");
  Serial.print(PB);

  Serial.print("\n");
  */
}


