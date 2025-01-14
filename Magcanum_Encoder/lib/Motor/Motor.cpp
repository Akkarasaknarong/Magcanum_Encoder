#include "Motor.h"

Motor::Motor(int ENpin, int IN1pin, int IN2pin, int encoder1pin, int encoder2pin)
{
    EN = ENpin;
    IN1 = IN1pin;
    IN2 = IN2pin;

    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
}

void Motor::speed(int speed)
{
    if (speed > 0)
    {

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    analogWrite(EN, abs(speed));
}

void Motor::stop()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(EN, 255);
}

