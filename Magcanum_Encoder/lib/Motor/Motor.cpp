#include "Motor.h"
#include "Arduino.h"

Motor::Motor(int _EN, int _IN1, int _IN2, int _encoder1, float _kp, float _ki, float _kd)
    : encoder(_encoder1), EN(_EN), IN1(_IN1), IN2(_IN2), kp(_kp), ki(_ki), kd(_kd)
{
    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
}

void Motor::speed(int _speed)
{
    if (_speed > 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else if (_speed < 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else if (_speed == 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, HIGH);
    }
    analogWrite(EN, abs(_speed));
}

void Motor::compute_PID(int _maxspeed, int _target_encoder)
{   
    max_speed = _maxspeed;

    // set target position
    int target = _target_encoder;
    // int target = 5000 * sin(prevT / 1e6);

    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    // Read the position
    int pos = 0;
    pos = encoder.getCount();

    // P = error
    int e = target - pos;
    // I = integral
    eintegral = eintegral + e * deltaT;
    // D = derivative
    float dedt = (e - eprev) / (deltaT);

    // output signal
    float u = kp * e + kd * dedt + ki * eintegral;
    int output_speed;
    if (u > max_speed)
    {
        output_speed = max_speed;
    }
    else if (u < (max_speed * -1))
    {
        output_speed = max_speed * -1;
    }
    else if (u > -1 || u < 1)
    {
        output_speed = 0;
    }

    speed(output_speed);
    eprev = e;

    Serial.print(pos);
}