#include "DHT.h"

// ----- Pin configuration -----
#define DHTPIN 1        // Signal pin connected to GPIO1 on ESP32
#define DHTTYPE DHT11   // DHT11 sensor

// ----- Create DHT object -----
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);        // Start serial communication
  dht.begin();                 // Initialize DHT sensor
  Serial.println("DHT11 Sensor Reading Started...");
}

void loop() {
  // Wait a bit between measurements
  delay(500);  // DHT11 has a max sampling rate of 1 reading per 2 seconds

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();  // Default is Celsius

  // Check if any reads failed and exit early (try again next loop)
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

