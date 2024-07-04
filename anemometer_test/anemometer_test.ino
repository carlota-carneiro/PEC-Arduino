#include <Arduino.h>
#include <SoftwareSerial.h>

// Define pins for SoftwareSerial
#define RX_PIN 2
#define TX_PIN 3

// Create a SoftwareSerial object
SoftwareSerial anemometerSerial(RX_PIN, TX_PIN);

void setup() {
  // Initialize hardware serial communication for debugging
  Serial.begin(9600);

  // Initialize SoftwareSerial communication for the anemometer
  anemometerSerial.begin(9600);

  // Debug message to indicate setup completion
  Serial.println("Setup complete. Starting wind speed and direction measurement...");
}

void loop() {
  if (anemometerSerial.available() > 0) {
    // Read the data from the anemometer
    String data = anemometerSerial.readStringUntil('\n');
    
    // Split the data into wind speed and direction
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      String windSpeedStr = data.substring(0, commaIndex);
      String windDirectionStr = data.substring(commaIndex + 1);

      // Convert to float
      float windSpeed = windSpeedStr.toFloat();
      int windDirection = windDirectionStr.toInt();

      // Print to the serial monitor
      Serial.print("Wind Speed: ");
      Serial.print(windSpeed);
      Serial.print(" m/s, Wind Direction: ");
      Serial.println(windDirection);
    }
  }

  // Add a short delay for readability
  delay(100);
}
