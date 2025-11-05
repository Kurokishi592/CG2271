#include "DHT.h"

#define DHTPIN 1        // Signal pin connected to GPIO1 on ESP32
#define DHTTYPE DHT11   

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);        // Start serial communication
  dht.begin();                 // Initialize DHT sensor
  Serial.println("DHT11 Sensor Reading Started...");
}

void loop() {
  delay(500); 

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();  // Default is Celsius
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT11 sensor!");
    return;
  }

  // Print readings to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
}

