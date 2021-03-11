/**
  @author Fabio Fanucchi - fabiofanucchi5@gmail.com
  @date 17/10/2019
  @version 1.0.0

  @brief Every 200ms pressure, temperature and humidity values are read from BME280 and printed on the serial monitor.
         Sensor settings can be changed as explained below (setup section).
*/

// include the library
#include<BME280.h>
BME280 bme;

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  //bme.settings.mode = //MODE_SLEEP, MODE_NORMAL, MODE_FORCED
  //bme.settings.tSleep =
  //bme.settings.filter =
  //bme.settings.pressureOS =
  //bme.settings.temperatureOS =
  //bme.settings.humidityOS =
// setup the sensor
  if (bme.begin()) {
    Serial.println("BME sensor connected.");
  }
  else {
    Serial.println("BME sensor not connected, check the circuit.");
  }

}

void loop() {
  // read and print on serial monitor values
  //bme.forcedMeasurement(); // needed in MODE_FORCED
  Serial.print("Pressure: ");
  Serial.println(bme.readPressure());
  Serial.print("Temperature: ");
  Serial.println(bme.readTemperature());
  Serial.print("Humidity: ");
  Serial.println(bme.readHumidity());

  delay(200);
}
