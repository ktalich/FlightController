#ifndef KT_FlightController_H
#define KT_FlightController_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <IMU.h>
#include <PIDController.h>
#include <GlobalConstants.h>

#define MIN_PULSEWIDTH 1000
#define MAX_PULSEWIDTH 2000

#define MOTOR_PIN_FL 2
#define MOTOR_PIN_FR 3
#define MOTOR_PIN_RL 4
#define MOTOR_PIN_RR 5

// #define MIN_MOTOR_SPEED 0.0
// #define MAX_MOTOR_SPEED 0.3
// #define IDLE_MOTOR_SPEED 0.05

// #define MAX_ANGLE 30.0
#define MAX_YAW_RATE 50.0

class FlightController
{
    IMU _imu;
    Servo _motorFL;
    Servo _motorFR;
    Servo _motorRL;
    Servo _motorRR;

    PIDController _pitchRatePID = PIDController(0.1337, 0.0, 0.0);
    PIDController _pitchAnglePID = PIDController(1.4000, 0.0, 0.0087);
    PIDController _rollRatePID = PIDController(0.1337, 0.0, 0.0);
    PIDController _rollAnglePID = PIDController(1.4000, 0.0, 0.0087);
    PIDController _yawRatePID = PIDController(1.60, 0.0, 0.0055);

    PIDController _accXRatePID = PIDController(0.0, 0.0, 0.0);
    PIDController _accYRatePID = PIDController(0.0, 0.0, 0.0);
    PIDController _heightRatePID = PIDController(0.0, 0.0, 0.0);

public:
    volatile float Pitch, Roll, Yaw;
    volatile float GyroX, GyroY, GyroZ;
    volatile float AccX, AccY, AccZ;

    volatile float PitchInput, RollInput, YawInput;

    volatile float Throttle;

    volatile int MaxMotorSpeed = 1410;
    volatile int IdleMotorSpeed = 1000;

    volatile float MaxAngle = 15.0;

    FlightController() { }

    void begin()
    {
        _motorFL.attach(MOTOR_PIN_FL);
        _motorFR.attach(MOTOR_PIN_FR);
        _motorRL.attach(MOTOR_PIN_RL);
        _motorRR.attach(MOTOR_PIN_RR);

        calibrateEscs();

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

        _motorFL.writeMicroseconds(MIN_PULSEWIDTH);
        _motorFR.writeMicroseconds(MIN_PULSEWIDTH);
        _motorRL.writeMicroseconds(MIN_PULSEWIDTH);
        _motorRR.writeMicroseconds(MIN_PULSEWIDTH);
        
        Serial.println("Motors initialized.");
        delay(5000);
    }

    void calibrateEscs()
    {
        Serial.println("Calibrating ESCs...");

        _motorFL.writeMicroseconds(MAX_PULSEWIDTH);
        _motorFR.writeMicroseconds(MAX_PULSEWIDTH);
        _motorRL.writeMicroseconds(MAX_PULSEWIDTH);
        _motorRR.writeMicroseconds(MAX_PULSEWIDTH);

        delay(5000);

        _motorFL.writeMicroseconds(MIN_PULSEWIDTH);
        _motorFR.writeMicroseconds(MIN_PULSEWIDTH);
        _motorRL.writeMicroseconds(MIN_PULSEWIDTH);
        _motorRR.writeMicroseconds(MIN_PULSEWIDTH);

        delay(2000);
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

        AccX = _imu.AccX;
        AccY = _imu.AccY;
        AccZ = _imu.AccZ;

        float throttle = IdleMotorSpeed + (MaxMotorSpeed - IdleMotorSpeed) * Throttle;
        float pitchSignal = getPitchSignal(PitchInput);
        float rollSignal = getRollSignal(RollInput);
        float yawSignal = getYawSignal(YawInput);

        float motorFLSpeed = throttle + yawSignal + pitchSignal + rollSignal;
        float motorFRSpeed = throttle - yawSignal + pitchSignal - rollSignal;
        float motorRLSpeed = throttle - yawSignal - pitchSignal + rollSignal;
        float motorRRSpeed = throttle + yawSignal - pitchSignal - rollSignal;

        _motorFL.writeMicroseconds(constrain(motorFLSpeed, IdleMotorSpeed, MaxMotorSpeed));
        _motorFR.writeMicroseconds(constrain(motorFRSpeed, IdleMotorSpeed, MaxMotorSpeed));
        _motorRL.writeMicroseconds(constrain(motorRLSpeed, IdleMotorSpeed, MaxMotorSpeed));
        _motorRR.writeMicroseconds(constrain(motorRRSpeed, IdleMotorSpeed, MaxMotorSpeed));

        //printInfo(motorFLSpeed, motorFRSpeed, motorRLSpeed, motorRRSpeed);
    }

    inline float getPitchSignal(float input)
    {
        float desiredAnglePitch = input * MaxAngle;
        float anglePitchOutput = _pitchAnglePID.Update(desiredAnglePitch, Pitch);

        // float desiredPitchRate = -anglePitchOutput;

        // float ratePitchOutput = _pitchRatePID.Update(desiredPitchRate, GyroY);
        // float pitchSignal = ratePitchOutput;

        return anglePitchOutput;
    }

    inline float getRollSignal(float input)
    {
        float desiredAngleRoll = input * MaxAngle;
        float angleRollOutput = _rollAnglePID.Update(desiredAngleRoll, Roll);

        // float desiredRollRate = angleRollOutput;

        // float rateRollOutput = _rollRatePID.Update(desiredRollRate, GyroX);
        // float rollSignal = rateRollOutput;

        return angleRollOutput;
    }

    inline float getYawSignal(float input)
    {
        float desiredYawRate = -input * MAX_YAW_RATE;
        float yawSignal = _yawRatePID.Update(desiredYawRate, GyroZ);

        return yawSignal;
    }


    void setMaxMotorSpeed(float ratio)
    {
        MaxMotorSpeed = 1000 + (ratio * 1000);
    }
    void setIdleMotorSpeed(float ratio)
    {
        IdleMotorSpeed = 1000 + (ratio * 1000);
    }
    void setMaxAngle(float maxAngle)
    {
        //MaxAngleRate = maxAngle;
        MaxAngle = maxAngle;
    }
    void setRatePID_KP(float pid_kp)
    {
        _pitchRatePID.setKp(pid_kp);
        _rollRatePID.setKp(pid_kp);
    }
    void setRatePID_KI(float pid_ki)
    {
        _pitchRatePID.setKi(pid_ki);
        _rollRatePID.setKi(pid_ki);
    }
    void setRatePID_KD(float pid_kd)
    {
        _pitchRatePID.setKd(pid_kd);
        _rollRatePID.setKd(pid_kd);
    }
    void setAnglePID_KP(float pid_kp)
    {
        _pitchAnglePID.setKp(pid_kp);
        _rollAnglePID.setKp(pid_kp);
    }
    void setAnglePID_KI(float pid_ki)
    {
        _pitchAnglePID.setKi(pid_ki);
        _rollAnglePID.setKi(pid_ki);
    }
    void setAnglePID_KD(float pid_kd)
    {
        _pitchAnglePID.setKd(pid_kd);
        _rollAnglePID.setKd(pid_kd);
    }
    void setYawRatePID_KP(float pid_kp)
    {
        _yawRatePID.setKp(pid_kp);
    }
    void setYawRatePID_KI(float pid_ki)
    {
        _yawRatePID.setKi(pid_ki);
    }
    void setYawRatePID_KD(float pid_kd)
    {
        _yawRatePID.setKd(pid_kd);
    }
    void setAccRatePID_KP(float pid_kp)
    {
        _accXRatePID.setKp(pid_kp);
        _accYRatePID.setKp(pid_kp);
    }
    void setAccRatePID_KI(float pid_ki)
    {
        _accXRatePID.setKi(pid_ki);
        _accYRatePID.setKi(pid_ki);
    }
    void setAccRatePID_KD(float pid_kd)
    {
        _accXRatePID.setKd(pid_kd);
        _accYRatePID.setKd(pid_kd);
    }
    void setHeightRatePID_KP(float pid_kp)
    {
        _heightRatePID.setKp(pid_kp);
    }
    void setHeightRatePID_KI(float pid_ki)
    {
        _heightRatePID.setKi(pid_ki);
    }
    void setHeightRatePID_KD(float pid_kd)
    {
        _heightRatePID.setKd(pid_kd);
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
    inline void printInfo(float motorFL, float motorFR, float motorRL, float motorRR)
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
        Serial.print("AccX:\t");
        Serial.print(AccX);
        Serial.print("\t");
        Serial.print("AccY:\t");
        Serial.print(AccY);
        Serial.print("\t");
        Serial.print("AccZ:\t");
        Serial.print(AccZ);
        Serial.print("\t");
        Serial.println();
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
        Serial.print("Motor FL:\t");
        Serial.print(motorFL);
        Serial.print("\t");
        Serial.print("Motor FR:\t");
        Serial.print(motorFR);
        Serial.print("\t");
        Serial.print("Motor RL:\t");
        Serial.print(motorRL);
        Serial.print("\t");
        Serial.print("Motor RR:\t");
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