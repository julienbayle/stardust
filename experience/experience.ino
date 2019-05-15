#include "ZSharpIR.h"
#include <Servo.h>

#define ir A0                                     // Broche de connection du capteur infra rouge
#define pinServo 9                                // Broche de pilotage du servo moteur
#define model ZSharpIR::GP2Y0A02YK0F              // Référence du capteur IR
#define pinLed1Gnd 2                              // Pin Led1 GND
#define pinLed2Gnd 4                              // Pin Led2 GND
#define pinLed1Pwm 3                              // Pin Led1 PMW
#define pinLed2Pwm 5                              // Pin Led2 PMW
#define SerialSpeed 115200


Servo myservo;                      // create servo object to control a servo
ZSharpIR theZSharpIR(ir, model);    // IR sensor

boolean animLedFinie;               // animation Led finie 0=non 1=oui
int intensite;                      // intensité des Led
long int tempo;
float angle, angleDeg, WVit;        // calcul angle de l'intensité
int count;                          // compteur déclenchement lancé au bout de 10

void setup() {
  Serial.begin(SerialSpeed);
  pinMode(pinLed1Gnd, OUTPUT);
  pinMode(pinLed2Gnd, OUTPUT);
  pinMode(pinLed1Pwm, OUTPUT);
  pinMode(pinLed2Pwm, OUTPUT);
  digitalWrite(pinLed1Gnd, LOW);
  digitalWrite(pinLed2Gnd, LOW);
  digitalWrite(pinLed1Pwm, LOW);  // Led éteinte au départ
  digitalWrite(pinLed2Pwm, LOW);  // Led éteinte au départ

  while ( count < 10)
  {
    delay(500);
    Serial.print("d=");
    // Mesure de la distance du capteur remplacer par 1000 pour test sans capteur
    long idistance = theZSharpIR.distance();  
    Serial.print(idistance);
    Serial.print(" mm");
    Serial.print("\t ");   
    Serial.print(count);
    Serial.println();     

    if (idistance > 900) {
      // Avant de declencher, on temporise 5 secondes (10 boucles de 500ms) à PWM 30
      analogWrite(pinLed2Pwm, 30);
      count++;
    } else {
      // Si le robot est présent, on reste sur PWM 4 (pas d'animation)
      count = 0;
      analogWrite(pinLed2Pwm, 4);
    }
  }

  Serial.println("START !!!!");

  //Anim Lampe
  tempo = -9001;                      // Pour démarrer Led éteinte (angle=0)
  WVit = 1.0 / 100;                   // vitesse angulaire initiale
  while (animLedFinie < 1) {
    tempo = tempo + 1;
    angle = WVit * float(tempo) * PI / 180 ; // angle en radian
    intensite = 127 * (1 + sin(angle));      // fonction intensité lumineuse

    // Calcul de l'évolution de la fréquence
    if (tempo % 100 == 0) {
      WVit = WVit + 1.0 / 5000;
      ///*
      // Affichage de l'animation
      angleDeg = angle * 180 / PI;
      Serial.print("tempo=");
      Serial.print(tempo);      Serial.print("\t");
      Serial.print("angle=");
      Serial.print(angleDeg, 0);       Serial.print("°");     Serial.print("\t");
      Serial.print("intensite=");
      Serial.print(intensite);
      Serial.print("\t");
      Serial.print("W=");
      Serial.print(WVit);
      Serial.println();
      //*/
    }

    analogWrite(pinLed1Pwm, intensite);
    analogWrite(pinLed2Pwm, intensite);
    if (WVit > 0.34 && intensite > 240) {
      animLedFinie = 1;
    }
  }

  // Eclairage max  sur les deux tubes !
  analogWrite(pinLed1Pwm, 255);                             
  analogWrite(pinLed2Pwm, 255);                         

  Serial.println("ELECTRON GO !!!!");
  
  myservo.attach(pinServo);  // attaches the servo on pin 9 to the servo object
  delay(1500);
  myservo.write(170 );
  delay(1500);
  myservo.detach();

  Serial.println("JOB DONE !!!!");
}


void loop() {
  ///*
    analogWrite(pinLed1Pwm, 5);
    analogWrite(pinLed2Pwm, 30);
    delay(500);
    analogWrite(pinLed1Pwm, 30);
    analogWrite(pinLed2Pwm, 5);
    delay(500);
  //*/
}
