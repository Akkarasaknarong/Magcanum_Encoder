#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
class Motor {
    private :
        int IN1;
        int IN2;
        int EN;
        int encoder1;
        int encoder2;
    public :
        Motor(int ENpin , int IN1pin ,int IN2pin,int encoder1pin , int encoder2pin);
        void speed(int speed);
        void stop();
};

#endif