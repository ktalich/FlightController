#include <Arduino.h>
#include <EEPROM.h>
#include <IMU.h>
#include <ESC.h>
#include <PIDController.h>

#define MOTOR_PIN_FL 2
#define MOTOR_PIN_FR 3
#define MOTOR_PIN_RL 4
#define MOTOR_PIN_RR 5

#define MIN_MOTOR_SPEED 0.0
#define MAX_MOTOR_SPEED 0.3
#define IDLE_MOTOR_SPEED 0.05

#define MAX_ANGLE 30.0
#define MAX_YAW_RATE 10.0

#define PITCH_ROLL_PID_KP 0.3
#define PITCH_ROLL_PID_KI 0.005
#define PITCH_ROLL_PID_KD 0.02

#define YAW_PID_KP 0.6
#define YAW_PID_KI 0.0
#define YAW_PID_KD 0.0

class FlightController
{
    IMU _imu;
    ESC _motorFL = ESC(MOTOR_PIN_FL);
    ESC _motorFR = ESC(MOTOR_PIN_FR);
    ESC _motorRL = ESC(MOTOR_PIN_RL);
    ESC _motorRR = ESC(MOTOR_PIN_RR);

    PIDController _pitchPID = PIDController(PITCH_ROLL_PID_KP, PITCH_ROLL_PID_KI, PITCH_ROLL_PID_KD);
    PIDController _rollPID = PIDController(PITCH_ROLL_PID_KP, PITCH_ROLL_PID_KI, PITCH_ROLL_PID_KD);
    PIDController _yawPID = PIDController(YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);

    float mapf(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    float clampf(float x, float min, float max)
    {
        if (x < min)
            return min;
        if (x > max)
            return max;
        return x;
    }

public:
    FlightController()
    {
        
    }

    void begin()
    {
        _imu.begin();
        EEPROM.begin(256);
        if (EEPROM.read(0) == 1)
            _imu.loadCalibration();
        else
            _imu.calibrateImu();
        EEPROM.end();
        delay(1000);

        Serial.println("IMU initialized.");
        delay(1000);

        _motorFL.begin();
        _motorFR.begin();
        _motorRL.begin();
        _motorRR.begin();

        _motorFL.setSpeed(0);
        _motorFR.setSpeed(0);
        _motorRL.setSpeed(0);
        _motorRR.setSpeed(0);
        Serial.println("Motors initialized.");
        delay(5000);
    }

    void update()
    {
        _imu.update();

        // float pitchTarget = remoteController.PitchInput * MAX_ANGLE;
        // float rollTarget = remoteController.RollInput * MAX_ANGLE;
        // float yawTarget = _imu.Yaw + remoteController.YawInput * MAX_YAW_RATE;
        float pitchTarget = 0.0 * MAX_ANGLE;
        float rollTarget = 0.0 * MAX_ANGLE;
        float yawTarget = 0.0 * MAX_YAW_RATE;

        // float dt = (millis() - _lastTime) / 1000.0;
        float dt = 0.005;

        float pitchOutput = _pitchPID.Update(pitchTarget, _imu.Pitch, dt);
        float rollOutput = _rollPID.Update(rollTarget, _imu.Roll, dt);
        // float yawOutput = yawPID.Update(yawTarget, _imu.Yaw, dt);
        float yawOutput = 0.0;

        float pitchSignal = mapf(pitchOutput, -MAX_ANGLE, MAX_ANGLE, -1.0, 1.0);
        float rollSignal = mapf(rollOutput, -MAX_ANGLE, MAX_ANGLE, -1.0, 1.0);
        float yawSignal = mapf(yawOutput, -MAX_YAW_RATE, MAX_YAW_RATE, -1.0, 1.0);

        float motorFLSpeed = clampf(0.0 + yawSignal + pitchSignal + rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);
        float motorFRSpeed = clampf(0.0 - yawSignal + pitchSignal - rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);
        float motorRLSpeed = clampf(0.0 - yawSignal - pitchSignal + rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);
        float motorRRSpeed = clampf(0.0 + yawSignal - pitchSignal - rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);

        _motorFL.setSpeed(motorFLSpeed);
        _motorFR.setSpeed(motorFRSpeed);
        _motorRL.setSpeed(motorRLSpeed);
        _motorRR.setSpeed(motorRRSpeed);

        // Serial.print("Motor FL:\t");
        // Serial.print(motorFLSpeed);
        // Serial.print("\t");

        // Serial.print("Motor FR:\t");
        // Serial.print(motorFRSpeed);
        // Serial.print("\t");

        // Serial.print("Motor RL:\t");
        // Serial.print(motorRLSpeed);
        // Serial.print("\t");

        // Serial.print("Motor RR:\t");
        // Serial.print(motorRRSpeed);
        // Serial.print("\t");

        // Serial.println();
    }
};
