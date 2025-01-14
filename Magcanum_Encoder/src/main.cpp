#include <Arduino.h>
#define LED_BUILTIN 25

#include <Motor.h>
#include <pio_encoder.h>
PioEncoder Encoder_FL(12);
PioEncoder Encoder_BL(14);
PioEncoder Encoder_FR(16);
PioEncoder Encoder_BR(18);

Motor FL(0, 1, 2, 12, 13);
Motor BL(5, 3, 4, 14, 15);
Motor FR(6, 7, 8, 16, 17);
Motor BR(11, 9, 10, 18, 19);

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

  Encoder_FL.begin();
}

void loop()
{
  // set target position
  int target = 1200;
  // int target = 250 * sin(prevT / 1e6);

  // PID constants
  float kp = 20;
  float kd = 0.5;
  float ki = 0.02;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position
  int pos = 0;
  pos = Encoder_FL.getCount();

  // error
  int e = target - pos;
  // derivative
  float dedt = (e - eprev) / (deltaT);
  // integral
  eintegral = eintegral + e * deltaT;
  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  int output_speed;
  if (u > 255)
  {
    output_speed = 255;
  }
  else if (u < -255)
  {
    output_speed = -255;
  }
  else if (u > -1 || u < 1)
  {
    output_speed = 0;
  }

  FL.speed(output_speed);

  eprev = e;
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}
