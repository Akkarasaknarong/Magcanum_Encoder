#include <Arduino.h>
#define LED_BUILTIN 25

#include <Motor.h>
#include <pio_encoder.h>
Motor Motor_FL(0, 1, 2, 12, 13);
Motor Motor_BL(5, 3, 4, 14, 15);
Motor Motor_FR(6, 7, 8, 16, 17);
Motor Motor_BR(11, 9, 10, 18, 19);

void setup()
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
  Serial.begin(115200);
  
}

void loop()
{

}
