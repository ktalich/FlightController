class PIDController
{
private:
    float _kp;
    float _ki;
    float _kd;
    float _lastError;
    float _integral;
public:
    PIDController(float kp, float ki, float kd);
    float Update(float target, float current, float dt);
};

PIDController::PIDController(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _lastError = 0;
    _integral = 0;
}

float PIDController::Update(float target, float current, float dt)
{
    float error = target - current;
    float derivative = (error - _lastError) / dt;
    _integral += error * dt;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    _lastError = error;

    return output;
}
