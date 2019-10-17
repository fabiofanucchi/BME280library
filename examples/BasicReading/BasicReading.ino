/**
  @author Fabio Fanucchi - fabiofanucchi5@gmail.com
  @date 17/10/2019
  @version 1.0.0

  @brief Every 200ms pressure, temperature and humidity values are read from BME280 and printed on the serial monitor.
*/

// include the library
#include<BME280.h>
BME bme;

void setup() {
  // initilize serial communication
  Serial.begin(9600);
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
  Serial.print("Pressure: ");
  Serial.println(bme.readPressure());
  Serial.print("Temperature: ");
  Serial.println(bme.readTemperature());
  Serial.print("Humidity: ");
  Serial.println(bme.readHumidity());

  delay(200);
}
