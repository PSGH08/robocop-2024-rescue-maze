#include <Servo.h>

// Define servo pins
Servo servo1, servo2, servo3, servo4;
const int servoPins[4] = {4, 5, 6, 7}; // Left servos: 4,5 | Right servos: 6,7

// Define ultrasonic sensor pin numbers
const int trigPin = 2;
const int echoPin = 3;

// Define constants
const int STOP_POSITION = 90;  // Middle position (stopped)
const int MAX_SPEED = 90;      // Maximum user-defined speed
int userSpeed = 90;            // Default speed (robot stops at 90)
String userMovement = "STOP";  // Default movement state
bool goCrazyMode = false;      // Tracks whether the robot is in auto mode

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Control Initialized!");
  Serial.println("Commands:");
  Serial.println("1. SPEED <0-90>: Set speed (e.g., SPEED 50)");
  Serial.println("2. FORWARDS: Move forward");
  Serial.println("3. BACKWARDS: Move backward");
  Serial.println("4. STOP: Stop robot");
  Serial.println("5. GO CRAZY: Auto movement using sensor");

  // Attach servos
  servo1.attach(servoPins[0]);
  servo2.attach(servoPins[1]);
  servo3.attach(servoPins[2]);
  servo4.attach(servoPins[3]);

  // Setup ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  stopServos();  // Start in stopped mode
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    String userInput = Serial.readStringUntil('\n');
    userInput.trim();

    // Handle SPEED command
    if (userInput.startsWith("SPEED")) {
      int inputSpeed = userInput.substring(6).toInt();
      if (inputSpeed >= 0 && inputSpeed <= MAX_SPEED) {
        userSpeed = inputSpeed;
        Serial.print("Speed set to: ");
        Serial.println(userSpeed);
      } else {
        Serial.println("Invalid speed! Use values between 0 and 90.");
      }
    }
    // Handle movement commands
    else if (userInput == "FORWARDS") {
      goCrazyMode = false;
      userMovement = "FORWARDS";
      moveRobot();
    }
    else if (userInput == "BACKWARDS") {
      goCrazyMode = false;
      userMovement = "BACKWARDS";
      moveRobot();
    }
    else if (userInput == "STOP") {
      goCrazyMode = false;
      stopServos();
      userMovement = "STOP";
      Serial.println("Robot stopped.");
    }
    else if (userInput == "GO CRAZY") {
      goCrazyMode = true;
      Serial.println("GO CRAZY mode activated!");
    }
    else {
      Serial.println("Invalid command. Use SPEED <0-90>, FORWARDS, BACKWARDS, STOP, or GO CRAZY.");
    }
  }

  // If in GO CRAZY mode, update movement dynamically
  if (goCrazyMode) {
    goCrazy();
  }

  delay(100);  // Small delay for smooth operation
}

// Move robot based on user command
void moveRobot() {
  int forwardSpeed = STOP_POSITION - userSpeed;
  int backwardSpeed = STOP_POSITION + userSpeed;

  if (userMovement == "FORWARDS") {
    servo1.write(forwardSpeed);
    servo2.write(forwardSpeed);
    servo3.write(STOP_POSITION + userSpeed);
    servo4.write(STOP_POSITION + userSpeed);
    Serial.println("Moving FORWARDS.");
  } 
  else if (userMovement == "BACKWARDS") {
    servo1.write(backwardSpeed);
    servo2.write(backwardSpeed);
    servo3.write(STOP_POSITION - userSpeed);
    servo4.write(STOP_POSITION - userSpeed);
    Serial.println("Moving BACKWARDS.");
  }
}

// Function to stop all servos
void stopServos() {
  servo1.write(STOP_POSITION);
  servo2.write(STOP_POSITION);
  servo3.write(STOP_POSITION);
  servo4.write(STOP_POSITION);
  Serial.println("Robot stopped.");
}

// Function to control GO CRAZY mode using the ultrasonic sensor
void goCrazy() {
  float distance = measureDistance();

  // If too close to the wall, turn RIGHT (left wheels move faster)
  if (distance < 5) {
    Serial.println("TOO CLOSE! Turning RIGHT.");
    turnRight();
  } 
  // If too far from the wall, turn LEFT (right wheels move faster)
  else if (distance > 5) {
    Serial.println("TOO FAR! Turning LEFT.");
    turnLeft();
  } 
  // If exactly 5cm, move forward
  else {
    Serial.println("PERFECT DISTANCE. Moving FORWARD.");
    moveRobot();
  }
}

// Measure distance from ultrasonic sensor
float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2)                                                           ;

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout to prevent hang-ups
  if (duration == 0) {
    Serial.println("Sensor error! No echo received.");
    return 10;  // Default safe distance if sensor fails
  }

  return (duration * 0.0343) / 2; // Convert to cm
}

// Turn right (left wheels move faster)
void turnRight() {
  int leftSpeed = STOP_POSITION + (userSpeed + 10);
  int rightSpeed = STOP_POSITION - userSpeed;

  servo1.write(leftSpeed);
  servo2.write(leftSpeed);
  servo3.write(rightSpeed);
  servo4.write(rightSpeed);
}

// Turn left (right wheels move faster)
void turnLeft() {
  int leftSpeed = STOP_POSITION + userSpeed;
  int rightSpeed = STOP_POSITION - (userSpeed + 10);

  servo1.write(leftSpeed);
  servo2.write(leftSpeed);
  servo3.write(rightSpeed);
  servo4.write(rightSpeed);
}
