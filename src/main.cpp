#include <Arduino.h>
#include <EEPROM.h>
#include <FlightController.h>
#include <RemoteController.h>
#include <GlobalConstants.h>

FlightController _flightController;
RemoteController _remoteController(_flightController);

void setup()
{
  // Serial.begin(115200);
  // while (!Serial)
  // {
  // }

  _flightController.begin();

  _remoteController.begin();
  Serial.println("Remote controller initialized.");
  delay(1000);
}

unsigned long _lastTime = 0;
void loop()
{
  if (millis() - _lastTime < REFRESH_RATE_MS)
    return;
  _lastTime = millis();

  _flightController.update();
  _remoteController.update();
}

// void setup1()
// {
// }

// void loop1()
// {
// }
