#include <GlobalConstants.h>

class PIDController
{
private:
    float _kp = 0;
    float _ki = 0;
    float _kd = 0;
    float _lastError = 0;
    float _integral = 0;

public:
    PIDController() { }
    PIDController(float kp, float ki, float kd)
    {
        setPID(kp, ki, kd);
    }

    void setPID(float kp, float ki, float kd)
    {
        setKp(kp);
        setKi(ki);
        setKd(kd);
    }

    void setKp(float kp)
    {
        _kp = kp;
    }
    void setKi(float ki)
    {
        _ki = ki * REFRESH_RATE_S;
        _integral = 0;
    }
    void setKd(float kd)
    {
        _kd = kd / REFRESH_RATE_S;
        _lastError = 0;
    }

    float Update(float target, float current)
    {
        float error = target - current;
        float derivative = error - _lastError;
        _integral += _ki * error;
        _lastError = error;

        return _kp * error + _integral + _kd * derivative;
    }
};
