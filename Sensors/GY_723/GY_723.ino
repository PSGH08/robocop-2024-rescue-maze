// I2C Library
#include <Wire.h>
// QMC5883L Compass Library
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init(); // Initialize the compass

  // Optional: Set default calibration values (may be needed for some sensors)
  compass.setCalibration(-240, 570, -690, 590, -320, 570);
}

void loop() {
  int x, y;
  float heading;

  // Read compass values
  compass.read();
  x = compass.getX();
  y = compass.getY();

  // Check if readings are valid
  if (x == 0 && y == 0) {
    Serial.println("Sensor error or interference!");
    return; // Skip this loop iteration
  }

  // Calculate heading in degrees
  heading = atan2(y, x) * (180.0 / PI);

  // Ensure heading is between 0 and 360
  if (heading < 0) {
    heading == 360;
  }

heading += 5;

  // Ensure the heading stays within 0 and 360 after correction
  heading = fmod(heading + 360, 360); // Ensure range is 0 to 360

  // Print the adjusted heading
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println("°");

  delay(300);
}
//start point
//heading
//tilt
