#include <Arduino.h>
#define LED_BUILTIN 25

#include <Motor.h>
#include <pio_encoder.h>
Motor FL(0, 1, 2, 12, 20, 0.02, 0.5);
Motor BL(5, 3, 4, 14, 20, 0.02, 0.5);
Motor FR(6, 7, 8, 16, 20, 0.02, 0.5);
Motor BR(11, 9, 10, 18, 20, 0.02, 0.5);

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

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
  delay(2500);
}

void setup()
{
  pico_begin();
  Serial.begin(115200);
  FL.encoder.begin();
  FR.encoder.begin();
  BL.encoder.begin();
  BR.encoder.begin();
}

void loop()
{
  // int target = 250 * sin(prevT / 1e6);
  // long currT = micros();
  // float deltaT = ((float)(currT - prevT)) / (1.0e6);
  // prevT = currT;
  // Serial.println(target);
  // delay(100);
  
  FL.compute_PID(80, 1500);
  Serial.print("\t");
  FR.compute_PID(80, 1500);
  Serial.print("\t");
  BL.compute_PID(80, 1500);
  Serial.print("\t");
  BR.compute_PID(80, 1500);
  Serial.println();
}
