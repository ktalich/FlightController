#include <Arduino.h>
#include <WiFi.h>
#include <FlightController.h>
#include <GlobalConstants.h>

#define CONNECTED 0x0
#define SET_MAX_MOTOR_SPEED 0x2
#define SET_IDLE_MOTOR_SPEED 0x3
#define SET_MAX_ANGLE 0x4
#define SET_RATE_PID_KP 0x5
#define SET_RATE_PID_KI 0x6
#define SET_RATE_PID_KD 0x7
#define SET_ANGLE_PID_KP 0x8
#define SET_ANGLE_PID_KI 0x9
#define SET_ANGLE_PID_KD 0x10
#define SET_THROTTLE 0x11
#define SET_PITCH_INPUT 0x12
#define SET_ROLL_INPUT 0x13
#define SET_YAW_INPUT 0x14
#define UPDATED_EULER_ANGLES 0x15
#define SET_YAW_RATE_PID_KP 0x16
#define SET_YAW_RATE_PID_KI 0x17
#define SET_YAW_RATE_PID_KD 0x18
#define SET_ACC_RATE_PID_KP 0x19
#define SET_ACC_RATE_PID_KI 0x20
#define SET_ACC_RATE_PID_KD 0x21
#define SET_HEIGHT_RATE_PID_KP 0x22
#define SET_HEIGHT_RATE_PID_KI 0x23
#define SET_HEIGHT_RATE_PID_KD 0x24

class RemoteController
{
    WiFiUDP _udp;
    byte _receiveBuffer[BUFFER_SIZE];
    byte _sendBuffer[BUFFER_SIZE];

    FlightController& _flightController;

public:
    volatile float PitchInput, RollInput, YawInput;

public:
    RemoteController(FlightController& flightController) : _flightController(flightController)
    {
    }

    void begin()
    {
        if (!WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, WIFI_HIDDEN, WIFI_MAX_CONN))
        {
            Serial.println("Creating access point failed");
            while (true)
            {
            }
        }

        Serial.println("Wi-Fi Access point created.");
        IPAddress IP = WiFi.softAPIP();
        Serial.print("Drone local IP address: ");
        Serial.println(IP);

        Serial.print("Remote controller ready! Listening on port: ");
        Serial.println(PROTOCOL_PORT);
        _udp.begin(PROTOCOL_PORT);
    }

    void update()
    {
        int packetSize = _udp.parsePacket();
        if (packetSize)
        {
            // Serial.print("Received packet of size ");
            // Serial.println(packetSize);
            // Serial.print("From ");
            // IPAddress remoteIp = _udp.remoteIP();
            // Serial.print(remoteIp);
            // Serial.print(", port ");
            // Serial.println(_udp.remotePort());

            int len = _udp.read(_receiveBuffer, BUFFER_SIZE);
            if (len > 0)
                parsePacket(len);
        }

        broadcast();
    }

private:
    void parsePacket(int length)
    {
        byte command = _receiveBuffer[0];

        // float value = *((float*)(_receiveBuffer + 1));
        //float* value = (float*)&_receiveBuffer[1];
        float value;
        memcpy(&value, (byte*)&_receiveBuffer[1], sizeof(float));

        switch (command)
        {
        case SET_MAX_MOTOR_SPEED:
            _flightController.setMaxMotorSpeed(value);
            break;
        case SET_IDLE_MOTOR_SPEED:
            _flightController.setIdleMotorSpeed(value);
            break;
        case SET_MAX_ANGLE:
            _flightController.setMaxAngle(value);
            break;
        case SET_RATE_PID_KP:
            _flightController.setRatePID_KP(value);
            break;
        case SET_RATE_PID_KI:
            _flightController.setRatePID_KI(value);
            break;
        case SET_RATE_PID_KD:
            _flightController.setRatePID_KD(value);
            break;
        case SET_ANGLE_PID_KP:
            _flightController.setAnglePID_KP(value);
            break;
        case SET_ANGLE_PID_KI:
            _flightController.setAnglePID_KI(value);
            break;
        case SET_ANGLE_PID_KD:
            _flightController.setAnglePID_KD(value);
            break;
        case SET_YAW_RATE_PID_KP:
            _flightController.setYawRatePID_KP(value);
            break;
        case SET_YAW_RATE_PID_KI:
            _flightController.setYawRatePID_KI(value);
            break;
        case SET_YAW_RATE_PID_KD:
            _flightController.setYawRatePID_KD(value);
            break;
        case SET_ACC_RATE_PID_KP:
            _flightController.setAccRatePID_KP(value);
            break;
        case SET_ACC_RATE_PID_KI:
            _flightController.setAccRatePID_KI(value);
            break;
        case SET_ACC_RATE_PID_KD:
            _flightController.setAccRatePID_KD(value);
            break;
        case SET_HEIGHT_RATE_PID_KP:
            _flightController.setHeightRatePID_KP(value);
            break;
        case SET_HEIGHT_RATE_PID_KI:
            _flightController.setHeightRatePID_KI(value);
            break;
        case SET_HEIGHT_RATE_PID_KD:
            _flightController.setHeightRatePID_KD(value);
            break;
        case SET_THROTTLE:
            _flightController.setThrottle(value);
            break;
        case SET_PITCH_INPUT:
            _flightController.setPitchInput(value);
            break;
        case SET_ROLL_INPUT:
            _flightController.setRollInput(value);
            break;
        case SET_YAW_INPUT:
            _flightController.setYawInput(value);
            break;
        }
    }

    unsigned long _lastBroadcast = 0;
    void broadcast()
    {
        if (millis() - _lastBroadcast < 100)
            return;
        _lastBroadcast = millis();

        _udp.beginPacket("255.255.255.255", PROTOCOL_PORT);
        
        _udp.write(UPDATED_EULER_ANGLES);
        _udp.write((byte*)&_flightController.Pitch, sizeof(float));
        _udp.write((byte*)&_flightController.Roll, sizeof(float));
        _udp.write((byte*)&_flightController.Yaw, sizeof(float));

        _udp.endPacket();
    }
};
