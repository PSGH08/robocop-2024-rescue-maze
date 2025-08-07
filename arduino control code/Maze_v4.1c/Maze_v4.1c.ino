#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <QMC5883LCompass.h>

// ----------------- Pin Definitions ------------------------
#define LEFT_MOTOR_PIN 13
#define RIGHT_MOTOR_PIN 10
#define MEDKIT_PIN 54

#define FRONT_LEFT_TRIG_PIN 67
#define FRONT_LEFT_ECHO_PIN 66
#define FRONT_RIGHT_TRIG_PIN 16
#define FRONT_RIGHT_ECHO_PIN 17
#define SIDE_LEFT_TRIG_PIN 65
#define SIDE_LEFT_ECHO_PIN 64
#define SIDE_RIGHT_TRIG_PIN 8
#define SIDE_RIGHT_ECHO_PIN 7

#define COLOR_SENSOR_PIN A0
#define COLOR_SENSOR_PIN1 58
#define COLOR_SENSOR_PIN2 57
#define COLOR_SENSOR_PIN3 4
#define COLOR_SENSOR_PIN4 5
#define COLOR_SENSOR_PIN5 2

#define LED_STARTUP 61
#define LED_MOVING 60
#define LED_STOPPED 59

// ----------------- Constants ------------------------
#define STOP_POSITION 90
#define SPEED 40
#define MOVE_DURATION 5000
#define STOP_DURATION 2500
#define TURN_DURATION 3000
#define THROW_DURATION 500

#define MIN_DISTANCE 6
#define MAX_DISTANCE 8
#define IGNORE_DISTANCE 7
#define FRONT_TRIGGER_DISTANCE 8

// ----------------- Globals ------------------------
QMC5883LCompass compass;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo leftMotor;
Servo rightMotor;

unsigned long lastMoveTime = 0;
bool isMoving = false;

// ----------------- Setup Functions ------------------------
void initializePins() {
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(MEDKIT_PIN, OUTPUT);

  pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
  pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
  pinMode(SIDE_LEFT_TRIG_PIN, OUTPUT);
  pinMode(SIDE_LEFT_ECHO_PIN, INPUT);
  pinMode(SIDE_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(SIDE_RIGHT_ECHO_PIN, INPUT);

  pinMode(COLOR_SENSOR_PIN, INPUT);
  pinMode(COLOR_SENSOR_PIN1, OUTPUT);
  pinMode(COLOR_SENSOR_PIN2, OUTPUT);
  pinMode(COLOR_SENSOR_PIN3, OUTPUT);
  pinMode(COLOR_SENSOR_PIN4, OUTPUT);
  pinMode(COLOR_SENSOR_PIN5, INPUT);

  pinMode(LED_STARTUP, OUTPUT);
  pinMode(LED_MOVING, OUTPUT);
  pinMode(LED_STOPPED, OUTPUT);

  lcd.begin(16, 2);
  lcd.backlight();

  leftMotor.attach(LEFT_MOTOR_PIN);
  rightMotor.attach(RIGHT_MOTOR_PIN);
}

// ----------------- Sensor Functions ------------------------
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = (duration == 0) ? IGNORE_DISTANCE : (duration * 0.0343) / 2;
  distance = constrain(distance, MIN_DISTANCE, MAX_DISTANCE);

  Serial.print("Ultrasonic Distance: ");
  Serial.println(distance);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Distance: " + String(distance));
  return distance;
}

int getHeading() {
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
int detectColor() {
  int value = analogRead(COLOR_SENSOR_PIN);
  int color = 0;

  if (value > 900) {
    color = 1;
    lcd.setCursor(0, 1); lcd.print("Color: White");
  } else if (value > 600) {
    color = 2;
    lcd.setCursor(0, 1); lcd.print("Color: Gray ");
  } else if (value > 300) {
    color = 3;
    lcd.setCursor(0, 1); lcd.print("Color: Black");
  } else {
    color = 4;
    lcd.setCursor(0, 1); lcd.print("Color: Blue ");
  }
  Serial.println("Color: " + String(color));
  return color;
}

// ----------------- Motor Control ------------------------
void moveForward() {
  leftMotor.write(STOP_POSITION + SPEED);
  rightMotor.write(STOP_POSITION - SPEED);
  lcd.setCursor(0, 0); lcd.print("Moving Forward ");
  Serial.println("Moving forward...");
  isMoving = true;
}

void stopMotors() {
  leftMotor.write(STOP_POSITION);
  rightMotor.write(STOP_POSITION);
  lcd.setCursor(0, 0); lcd.print("Stopped       ");
  Serial.println("Stopped.");
  isMoving = false;
}

void steerLeft() {
  leftMotor.write(STOP_POSITION - SPEED);
  rightMotor.write(STOP_POSITION - SPEED);
  delay(200);
  stopMotors();
  lcd.setCursor(0, 0); lcd.print("Steering Left ");
  Serial.println("Steered left.");
}

void steerRight() {
  leftMotor.write(STOP_POSITION + SPEED);
  rightMotor.write(STOP_POSITION + SPEED);
  delay(200);
  stopMotors();
  lcd.setCursor(0, 0); lcd.print("Steering Right");
  Serial.println("Steered right.");
}

void turnLeft() {
  leftMotor.write(STOP_POSITION - SPEED);
  rightMotor.write(STOP_POSITION - SPEED);
  delay(TURN_DURATION);
  stopMotors();
  lcd.setCursor(0, 0); lcd.print("Turned Left   ");
  Serial.println("Turned left.");
}

void turnRight() {
  leftMotor.write(STOP_POSITION + SPEED);
  rightMotor.write(STOP_POSITION + SPEED);
  delay(TURN_DURATION);
  stopMotors();
  lcd.setCursor(0, 0); lcd.print("Turned Right  ");
  Serial.println("Turned right.");
}

void moveBackward(int duration) {
  leftMotor.write(STOP_POSITION - SPEED);
  rightMotor.write(STOP_POSITION + SPEED);
  delay(duration);
  stopMotors();
  lcd.setCursor(0, 0); lcd.print("Moving Back   ");
  Serial.println("Moved backward.");
}

void throwMedkit() {
  digitalWrite(MEDKIT_PIN, HIGH);
  delay(THROW_DURATION);
  digitalWrite(MEDKIT_PIN, LOW);
  lcd.setCursor(0, 0); lcd.print("Medkit Thrown ");
  Serial.println("Medkit thrown.");
}

void updateLEDs() {
  digitalWrite(LED_MOVING, isMoving ? HIGH : LOW);
  digitalWrite(LED_STOPPED, isMoving ? LOW : HIGH);
}

// ----------------- Main Control Loop ------------------------
void setup() {
  Serial.begin(9600);
  initializePins();
  compass.init();

  digitalWrite(LED_STARTUP, HIGH);
  lcd.setCursor(0, 0); lcd.print("Initializing...");
  delay(3000);
  digitalWrite(LED_STARTUP, LOW);
  lcd.clear();
  lcd.print("Robot Ready");
  Serial.println("Robot initialization complete");
  lastMoveTime = millis();
  compass.setCalibration(-240, 570, -690, 590, -320, 570);
}

void loop() {
  if (millis() - lastMoveTime >= MOVE_DURATION) {
    float leftFront = getDistance(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN);
    float rightFront = getDistance(FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN);
    float leftSide = getDistance(SIDE_LEFT_TRIG_PIN, SIDE_LEFT_ECHO_PIN);
    float rightSide = getDistance(SIDE_RIGHT_TRIG_PIN, SIDE_RIGHT_ECHO_PIN);
    int color = detectColor();

    if (leftFront < 8.0 && rightFront < 8.0 && rightSide < 8.0) {
      turnLeft();
    } else if (leftFront < 8.0 && rightFront < 8.0 && leftSide < 8.0) {
      turnRight();
    } else if (rightSide > 6.0 && rightSide < 8.0 && leftSide > 6.0 && leftSide < 8.0) {
      moveForward();
    } else if (leftSide < 6.0) {
      steerRight();
    } else if (leftSide > 8.0) {
      steerLeft();
    } else if (rightSide > 8.0) {
      steerRight();
    } else if (rightSide < 6.0) {
      steerLeft();
    } else {
      moveForward();
    }

    if (color == 3 || color == 4) {
      moveBackward(1000);
    }

    updateLEDs();

    Serial.println("Sensor Readings:");
    Serial.print("Front Left: "); Serial.println(leftFront);
    Serial.print("Front Right: "); Serial.println(rightFront);
    Serial.print("Side Left: "); Serial.println(leftSide);
    Serial.print("Side Right: "); Serial.println(rightSide);
    Serial.print("Color: "); Serial.println(color);

    lastMoveTime = millis();
  }

  delay(100);
}
