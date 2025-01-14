#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <pio_encoder.h>

class Motor
{
private:
    int IN1;
    int IN2;
    int EN;

    int max_speed ;

    // PID construct
    volatile int posi = 0;
    long prevT = 0;
    float eprev = 0;
    float eintegral = 0;
    float kp, ki, kd;

public:
    Motor(int _EN, int _IN1, int _IN2, int _encoder1, float _kp, float _ki, float _kd);
    void speed(int _speed);
    void compute_PID(int _maxspeed, int _target_encoder);
    PioEncoder encoder;
};

#endif