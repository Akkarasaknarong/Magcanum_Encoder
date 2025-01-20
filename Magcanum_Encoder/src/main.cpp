#include <Arduino.h>
#define LED_BUILTIN 25

#include <Motor.h>
#include <pio_encoder.h>
Motor FL(0, 1, 2, 12, 20, 0.02, 0.5);
Motor BL(5, 3, 4, 14, 20, 0.02, 0.5);
Motor FR(6, 7, 8, 16, 20, 0.02, 0.5);
Motor BR(11, 9, 10, 18, 20, 0.02, 0.5);

#define en1 12
#define en2 13

long Timer0 = 0;
long Timer0_prevT = 0;
float eprev = 0;
float eintregral = 0 ;

// Velocity measurement
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float pervT_i = 0;
volatile float velocity_2 = 0;

// Velocity filter
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float v1 = 0;
float v2 = 0;

void pico_begin()
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void Encoder_Interrupt()
{

  int b = digitalRead(en2);
  int increment = 0;
  if (b > 0)
  {
    increment = 1;
  }
  else
  {
    increment = -1;
  }
  pos_i += increment;

  // Read velocity mothod 2
  long currT = micros();
  float deltaT = (currT - pervT_i) / 1.0e6;
  velocity_2 = increment / deltaT;
  pervT_i = currT;
}

void setup()
{
  attachInterrupt(digitalPinToInterrupt(en1), Encoder_Interrupt, RISING);
  pico_begin();
  Serial.begin(115200);
  FL.encoder.begin();
  FR.encoder.begin();
  BL.encoder.begin();
  BR.encoder.begin();
}

void loop()
{

  // Read velocity mothod 1
  long currT = micros();
  float deltaT = (currT - prevT) / 1.0e6;
  float velocity_1 = (pos_i - posPrev) / deltaT;
  posPrev = pos_i;
  prevT = currT;

  // Convert velocity from    >>> encoder/second <<<    to     >>> RPM/second <<<
  v1 = (velocity_1 / 1000 * 60);
  v2 = (velocity_2 / 1000 * 60);

  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;

  // Compute PID
  float vt = 110 * (sin(currT / 1e6) > 0);
  // float vt = 100 ;
  float kp = 10;
  float ki = 0.1;

  float e = vt - v1Filt;
  eintregral = eintregral + (e * deltaT);

  float output = kp * e + ki * eintregral;

  FL.speed(output);

  Timer0 = millis();
  if (Timer0 - Timer0_prevT > 100)
  {
    Serial.print(">");
    Serial.print("vt:");
    Serial.print(vt);
    Serial.print(",");

    Serial.print("v1Filt:");
    Serial.print(v1Filt);
    Serial.print(",");
    Serial.println();
    Timer0_prevT = Timer0;
  }
}
