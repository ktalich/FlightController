#include <Arduino.h>
#include <WiFi.h>

#define WIFI_SSID "Scout Mk I"
#define WIFI_PASS "SuperHeslo"
#define WIFI_CHANNEL 1
#define WIFI_HIDDEN 0
#define WIFI_MAX_CONN 1

#define PROTOCOL_PORT 12321
#define BUFFER_SIZE 256

class RemoteController
{
    WiFiUDP _udp;
    byte _receieBuffer[BUFFER_SIZE];
    byte _sendBuffer[BUFFER_SIZE];

public:
    volatile float PitchInput, RollInput, YawInput;

public:
    RemoteController();

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
            Serial.print("Received packet of size ");
            Serial.println(packetSize);
            Serial.print("From ");
            IPAddress remoteIp = _udp.remoteIP();
            Serial.print(remoteIp);
            Serial.print(", port ");
            Serial.println(_udp.remotePort());

            int len = _udp.read(_receieBuffer, BUFFER_SIZE);
            if (len > 0)
            {
                _receieBuffer[len] = 0;
            }
            Serial.println("Contents:");
            Serial.println((char *)_receieBuffer);

            _udp.beginPacket(_udp.remoteIP(), _udp.remotePort());
            _udp.write("Hello there!");
            _udp.endPacket();
        }
    }

private:
    void parsePacket()
    {

    }
};
