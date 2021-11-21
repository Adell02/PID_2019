//// LIBRERIAS ////
#include<Servo.h>
#include<PID_v1.h>
#include <SharpIR.h>
#include <Adafruit_NeoPixel.h>

//// PINES ////
#define pinLedKp 5
#define pinLedKi 6
#define pinLedKd 12
#define pinKp 2
#define pinKi 3
#define pinKd 4
#define servoPin  11
#define sensorPin  A0
#define LEDs 13
#define numled 9


//// VARIABLES ////
int last_reading;
float Kp = 0.65;
float Ki = 0.2;
float Kd = 0.34;
double Setpoint, Input, Output, ServoOutput;
double KpA = 0, KiA = 0, KdA = 0;
float dist;
int tiempo =100;
int counter1 = 0;
int counter2 = 0;

PID myPID(&Input, &Output, &Setpoint, KpA, KiA, KdA, DIRECT);

SharpIR sensor( SharpIR::GP2Y0A21YK0F, sensorPin );
Servo myServo;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numled, LEDs, NEO_GRB + NEO_KHZ800);

///////////////
//// SETUP ////
///////////////

void setup() {
  last_reading = 0;
  Serial.begin(9600);

  //// PIN MODE ////
  pinMode(pinKp, INPUT_PULLUP);
  pinMode(pinKi, INPUT_PULLUP);
  pinMode(pinKd, INPUT_PULLUP);
  pinMode(pinLedKp, OUTPUT);
  pinMode(pinLedKi, OUTPUT);
  pinMode(pinLedKd, OUTPUT);
  

  digitalWrite(pinLedKp, LOW);
  digitalWrite(pinLedKi, LOW);
  digitalWrite(pinLedKd, LOW);
  
  myServo.attach(servoPin);

  //// PID ////
  Input = readPosition();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-80, 80);


  //// LEDS ////
  pixels.begin();
  pixels.show();
  int OK = counter1 + counter2;
  
}

//////////////
//// LOOP ////
//////////////

void loop() {

  BotonKp();
  BotonKi();
  BotonKd();
  Setpoint = 23;
  Input = readPosition();
  ControlPID();
  ConsignaOK();

}

//// BOTON KP ////
void BotonKp() {
  static boolean s = 0;
  static boolean ON = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinKp) == 0) {
      s = 1;
      mils = millis();
      ON = !ON;
      KpA = Kp * ON;
      myPID.SetTunings(KpA, KiA, KdA);
      digitalWrite(pinLedKp, ON);
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinKp) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }
}


//// BOTON KI ////
void BotonKi() {
  static boolean s = 0;
  static boolean ON = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinKi) == 0) {
      s = 1;
      mils = millis();
      ON = !ON;
      KiA = Ki * ON;
      myPID.SetTunings(KpA, KiA, KdA);
      digitalWrite(pinLedKi, ON);

    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinKi) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }
}


//// BOTON KD ////
void BotonKd() {
  static boolean s = 0;
  static boolean ON = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinKd) == 0) {
      s = 1;
      mils = millis();
      ON = !ON;
      KdA = Kd * ON;
      myPID.SetTunings(KdA, KiA, KdA);
      digitalWrite(pinLedKd, ON);
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinKi) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }
}



//// CONTROL PID ////

void ControlPID() {

  myPID.Compute();
  last_reading = Input;
  ServoOutput = 120.5 + Output;
  myServo.write(ServoOutput);


}


//// PLOTTER POSICIÓN ////
float readPosition() {
  delay(50);

  dist = sensor.getDistance();
  if (dist > 55)
  {
    dist = 55;
  }

  Serial.println(dist);
  return dist;
}

////OK ////
void ConsignaOK() {
if ( abs(dist) >= 20 && (abs(dist)<= 25)) {
    pixels.setPixelColor (4,0,255,0); pixels.show();
  }
else {pixels.setPixelColor (4,0,0,0); pixels.show();}}
