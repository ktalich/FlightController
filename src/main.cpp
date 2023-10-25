#include <Arduino.h>
#include <EEPROM.h>
#include <FlightController.h>
// #include <RemoteController.h>

#define REFRESH_RATE_HZ 200
#define REFRESH_RATE_MS (1000 / REFRESH_RATE_HZ)

FlightController _flightController;
// RemoteController remoteController;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  _flightController.begin();

  // remoteController.begin();
  // Serial.println("Remote controller initialized.");
  // delay(1000);
}

unsigned int _loopCount = 0;
unsigned long _lastPrintTime = 0;

unsigned long _lastTime = 0;
void loop()
{
  if(millis() - _lastTime < REFRESH_RATE_MS)
    return;
  _lastTime = millis();

  if(millis() - _lastPrintTime >= 1000)
  {
    Serial.print("Loop count:\t");
    Serial.print(_loopCount);
    Serial.println();

    _loopCount = 0;
    _lastPrintTime = millis();
  }

  _flightController.update();
  _loopCount++;
}

void setup1()
{
}

void loop1()
{
}
