#ifndef KT_FlightController_H
#define KT_FlightController_H

#include <Arduino.h>
#include <EEPROM.h>
#include <IMU.h>
#include <ESC.h>
#include <PIDController.h>
#include <GlobalConstants.h>

#define MOTOR_PIN_FL 2
#define MOTOR_PIN_FR 3
#define MOTOR_PIN_RL 4
#define MOTOR_PIN_RR 5

// #define MIN_MOTOR_SPEED 0.0
// #define MAX_MOTOR_SPEED 0.3
// #define IDLE_MOTOR_SPEED 0.05

// #define MAX_PITCH_ROLL_ANGLE 30.0
// #define MAX_YAW_RATE 10.0

// #define PITCH_ROLL_PID_KP 0.3
// #define PITCH_ROLL_PID_KI 0.005
// #define PITCH_ROLL_PID_KD 0.02

// #define YAW_PID_KP 0.6
// #define YAW_PID_KI 0.0
// #define YAW_PID_KD 0.0

class FlightController
{
    IMU _imu;
    ESC _motorFL = ESC(MOTOR_PIN_FL);
    ESC _motorFR = ESC(MOTOR_PIN_FR);
    ESC _motorRL = ESC(MOTOR_PIN_RL);
    ESC _motorRR = ESC(MOTOR_PIN_RR);

    PIDController _pitchPID;
    PIDController _rollPID;
    PIDController _yawPID;

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
    volatile float Pitch, Roll, Yaw;
    volatile float GyroX, GyroY, GyroZ;

    volatile float PitchInput, RollInput, YawInput;

    volatile float Throttle;

    float MinMotorSpeed = 0.0;
    float MaxMotorSpeed = 0.5;
    float IdleMotorSpeed = 0.05;

    FlightController() { }

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

        setPID_KP(1.0);
    }

    void update()
    {
        _imu.update();

        Pitch = _imu.Pitch;
        Roll = _imu.Roll;
        Yaw = _imu.Yaw;

        GyroX = _imu.GyroX;
        GyroY = _imu.GyroY;
        GyroZ = _imu.GyroZ;


        //float desiredPitchRate = PitchInput * 100.0;
        float desiredRollRate = RollInput * 100.0;
        //float desiredYawRate = YawInput * 100.0;

        //float pitchOutput = 0.0;
        float rollOutput = clampf(_rollPID.Update(desiredRollRate, GyroX), -100.0, 100.0);
        //float yawOutput = 0.0;

        float throttleSignal = Throttle;
        float rollSignal = rollOutput / 100.0;

        if(throttleSignal + rollSignal > 1.0)
            throttleSignal -= throttleSignal + rollSignal - 1.0;

        _motorFL.setSpeed(clampf(throttleSignal + rollSignal, IdleMotorSpeed, MaxMotorSpeed));
        _motorFR.setSpeed(clampf(throttleSignal - rollSignal, IdleMotorSpeed, MaxMotorSpeed));
        _motorRL.setSpeed(clampf(throttleSignal + rollSignal, IdleMotorSpeed, MaxMotorSpeed));
        _motorRR.setSpeed(clampf(throttleSignal - rollSignal, IdleMotorSpeed, MaxMotorSpeed));

        // Serial.print("Motor FL: ");
        // Serial.print(clampf(throttleSignal + rollSignal, IDLE_MOTOR_SPEED, 0.5));
        // Serial.print("Motor FR: ");
        // Serial.print(clampf(throttleSignal - rollSignal, IDLE_MOTOR_SPEED, 0.5));
        // Serial.print("Motor RL: ");
        // Serial.print(clampf(throttleSignal + rollSignal, IDLE_MOTOR_SPEED, 0.5));
        // Serial.print("Motor RR: ");
        // Serial.println(clampf(throttleSignal - rollSignal, IDLE_MOTOR_SPEED, 0.5));


        // float pitchTarget = 0.0 * MAX_PITCH_ROLL_ANGLE;
        // float rollTarget = 0.0 * MAX_PITCH_ROLL_ANGLE;
        // float yawTarget = 0.0 * MAX_YAW_RATE;

        // float pitchOutput = _pitchPID.Update(pitchTarget, Pitch);
        // float rollOutput = _rollPID.Update(rollTarget, Roll);
        // // float yawOutput = yawPID.Update(yawTarget, Yaw);
        // float yawOutput = 0.0;

        // float pitchSignal = mapf(pitchOutput, -MAX_ANGLE, MAX_ANGLE, -1.0, 1.0);
        // float rollSignal = mapf(rollOutput, -MAX_ANGLE, MAX_ANGLE, -1.0, 1.0);
        // float yawSignal = mapf(yawOutput, -MAX_YAW_RATE, MAX_YAW_RATE, -1.0, 1.0);

        // float motorFLSpeed = clampf(0.0 + yawSignal + pitchSignal + rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);
        // float motorFRSpeed = clampf(0.0 - yawSignal + pitchSignal - rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);
        // float motorRLSpeed = clampf(0.0 - yawSignal - pitchSignal + rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);
        // float motorRRSpeed = clampf(0.0 + yawSignal - pitchSignal - rollSignal, IDLE_MOTOR_SPEED, MAX_MOTOR_SPEED);

        // _motorFL.setSpeed(motorFLSpeed);
        // _motorFR.setSpeed(motorFRSpeed);
        // _motorRL.setSpeed(motorRLSpeed);
        // _motorRR.setSpeed(motorRRSpeed);
    }

    void setMinMotorSpeed(float minMotorSpeed)
    {
        MinMotorSpeed = minMotorSpeed;
    }
    void setMaxMotorSpeed(float maxMotorSpeed)
    {
        MaxMotorSpeed = maxMotorSpeed;
    }
    void setIdleMotorSpeed(float idleMotorSpeed)
    {
        IdleMotorSpeed = idleMotorSpeed;
    }
    void setMaxAngle(float maxAngle)
    {
    }
    void setPID_KP(float pid_kp)
    {
        _pitchPID.setKp(pid_kp);
        _rollPID.setKp(pid_kp);
    }
    void setPID_KI(float pid_ki)
    {
        //_pitchPID.setKi(pid_ki);
        //_rollPID.setKi(pid_ki);
    }
    void setPID_KD(float pid_kd)
    {
        //_pitchPID.setKd(pid_kd);
        //_rollPID.setKd(pid_kd);
    }
    void setThrottle(float throttle)
    {
        Throttle = throttle;
    }
    void setPitchInput(float pitchInput)
    {
        PitchInput = pitchInput;
    }
    void setRollInput(float rollInput)
    {
        RollInput = rollInput;
    }
    void setYawInput(float yawInput)
    {
        YawInput = yawInput;
    }
};

#endif