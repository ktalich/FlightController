#include <Arduino.h>
#include <Servo.h>

#define MIN_PULSEWIDTH 1000
#define MAX_PULSEWIDTH 2000

class ESC
{
private:
    Servo _servo;
    int _pin;

public:
    ESC(int pin)
    {
        _pin = pin;
    }

    void begin()
    {
        _servo.attach(_pin);
        _servo.writeMicroseconds(MIN_PULSEWIDTH);
    }

    void setSpeed(float speed)
    {
        int pulseWidth = map(speed * 100, 0, 100, MIN_PULSEWIDTH, MAX_PULSEWIDTH);
        _servo.writeMicroseconds(pulseWidth);
    }

    // long map(long x, long in_min, long in_max, long out_min, long out_max) {
    //   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // }
};