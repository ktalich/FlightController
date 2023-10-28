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
// #define MOTOR_PIN_RL 4
// #define MOTOR_PIN_RR 5

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
    // ESC _motorRL = ESC(MOTOR_PIN_RL);
    // ESC _motorRR = ESC(MOTOR_PIN_RR);

    PIDController _ratePID = PIDController(0.0012, 0.0, 0.0);
    PIDController _anglePID = PIDController(0.0, 0.0, 0.0);
    // PIDController _pitchPID;
    // PIDController _rollPID;
    // PIDController _yawPID;

    // float mapf(float x, float in_min, float in_max, float out_min, float out_max)
    // {
    //     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // }

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

    volatile float MinMotorSpeed = 0.0;
    volatile float MaxMotorSpeed = 0.18;
    volatile float IdleMotorSpeed = 0.09;

    volatile float MaxAngle = 30.0;

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
        // _motorRL.begin();
        // _motorRR.begin();

        _motorFL.setSpeed(0);
        _motorFR.setSpeed(0);
        // _motorRL.setSpeed(0);
        // _motorRR.setSpeed(0);
        Serial.println("Motors initialized.");
        delay(5000);
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

        float desiredAnglePitch = PitchInput * MaxAngle;
        float anglePitchOutput = _anglePID.Update(desiredAnglePitch, Pitch);

        float desiredPitchRate = -anglePitchOutput;

        float ratePitchOutput = _ratePID.Update(desiredPitchRate, GyroY);
        float pitchSignal = ratePitchOutput;

        float throttle = IdleMotorSpeed + (MaxMotorSpeed - IdleMotorSpeed) * Throttle;

        float motorLSpeed = clampf(throttle + pitchSignal, IdleMotorSpeed, MaxMotorSpeed);
        float motorRSpeed = clampf(throttle - pitchSignal, IdleMotorSpeed, MaxMotorSpeed);

        _motorFL.setSpeed(motorLSpeed);
        _motorFR.setSpeed(motorRSpeed);

        // if(throttleSignal + rollSignal > 1.0)
        //     throttleSignal -= throttleSignal + rollSignal - 1.0;

        //printInfo(motorLSpeed, motorRSpeed);
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
        //MaxAngleRate = maxAngle;
        MaxAngle = maxAngle;
    }
    void setRatePID_KP(float pid_kp)
    {
        _ratePID.setKp(pid_kp);
    }
    void setRatePID_KI(float pid_ki)
    {
        _ratePID.setKi(pid_ki);
    }
    void setRatePID_KD(float pid_kd)
    {
        _ratePID.setKd(pid_kd);
    }
    void setAnglePID_KP(float pid_kp)
    {
        _anglePID.setKp(pid_kp);
    }
    void setAnglePID_KI(float pid_ki)
    {
        _anglePID.setKi(pid_ki);
    }
    void setAnglePID_KD(float pid_kd)
    {
        _anglePID.setKd(pid_kd);
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

private:
    volatile unsigned long _lastPrint = 0;
    inline void printInfo(float motorRL, float motorRR)
    {
        if (millis() - _lastPrint < 50)
            return;
        _lastPrint = millis();

        Serial.println();
        Serial.print("Pitch:\t");
        Serial.print(Pitch);
        Serial.print("\t");
        Serial.print("Roll:\t");
        Serial.print(Roll);
        Serial.print("\t");
        Serial.print("Yaw:\t");
        Serial.print(Yaw);
        Serial.print("\t");
        Serial.println();
        Serial.print("GyroX:\t");
        Serial.print(GyroX);
        Serial.print("\t");
        Serial.print("GyroY:\t");
        Serial.print(GyroY);
        Serial.print("\t");
        Serial.print("GyroZ:\t");
        Serial.print(GyroZ);
        Serial.print("\t");
        Serial.println();
        Serial.print("Min Motor Speed:\t");
        Serial.print(MinMotorSpeed);
        Serial.print("\t");
        Serial.print("Max Motor Speed:\t");
        Serial.print(MaxMotorSpeed);
        Serial.print("\t");
        Serial.print("Idle Motor Speed:\t");
        Serial.print(IdleMotorSpeed);
        Serial.print("\t");
        Serial.println();
        Serial.print("Throttle:\t");
        Serial.print(Throttle);
        Serial.print("\t");
        Serial.println();
        Serial.print("Motor L:\t");
        Serial.print(motorRL);
        Serial.print("\t");
        Serial.print("Motor R:\t");
        Serial.print(motorRR);
        Serial.print("\t");
        Serial.println();

        // float desiredAnglePitch = PitchInput * MaxAngle;
        // float anglePitchError = desiredAnglePitch - Pitch;

        // float anglePitchOutput = AnglePGain * anglePitchError;

        // float desiredPitchRate = -anglePitchOutput;
        // float ratePitchError = desiredPitchRate - GyroY;
        // float ratePitchOutput = RatePGain * ratePitchError;
        // Serial.print("Pitch error:\t");
        // Serial.println(ratePitchError);
        // Serial.print("Pitch output:\t");
        // Serial.print("\t");
        // Serial.print(ratePitchOutput);
        // Serial.print("\t");
        // Serial.println();

        // Serial.print("Desired angle pitch:\t");
        // Serial.print("\t");
        // Serial.println(desiredAnglePitch);
        // Serial.print("Angle pitch error:\t");
        // Serial.print("\t");
        // Serial.println(anglePitchError);
        // Serial.print("Angle pitch output:\t");
        // Serial.print("\t");
        // Serial.print(anglePitchOutput);
        // Serial.print("\t");
        // Serial.println();
    }
};

#endif