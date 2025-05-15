#include <Servo.h>

// ----------------- Define Area ------------------------
#define LEFT_SERVO1_PIN  9
#define LEFT_SERVO2_PIN  10
#define RIGHT_SERVO1_PIN 11
#define RIGHT_SERVO2_PIN 12

#define FRONT_TRIG_PIN  5
#define FRONT_ECHO_PIN  6
#define RIGHT_TRIG_PIN  4
#define RIGHT_ECHO_PIN  3
#define LEFT_TRIG_PIN   7
#define LEFT_ECHO_PIN   8

#define STOP_POSITION 90
#define SPEED 20  
#define TURN_DURATION 2000  // 2 sec for full turns
#define STEER_DURATION 100  // 0.1 sec for small adjustments
#define MOVE_FORWARD_TIME 3000 // 3 sec for moving forward

#define MIN_DISTANCE 3.5  
#define MAX_DISTANCE 6.0  
#define IGNORE_DISTANCE 8.0  
#define FRONT_TRIGGER_DISTANCE 5.0  

Servo leftServo1, leftServo2, rightServo1, rightServo2;

// ------------------ Hardware Setup ---------------------
void setupHardware() {
    Serial.begin(9600);

    leftServo1.attach(LEFT_SERVO1_PIN);
    leftServo2.attach(LEFT_SERVO2_PIN);
    rightServo1.attach(RIGHT_SERVO1_PIN);
    rightServo2.attach(RIGHT_SERVO2_PIN);

    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);
    pinMode(LEFT_TRIG_PIN, OUTPUT);
    pinMode(LEFT_ECHO_PIN, INPUT);
}

// ------------------ Sensor Functions ---------------------
float measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 10;

    return (duration * 0.0343) / 2;
}

float getFrontDistance() { return measureDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN); }
float getRightDistance() { return measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN); }
float getLeftDistance()  { return measureDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);  }

// ------------------ Movement Functions ---------------------
void moveForwards() {
    leftServo1.write(STOP_POSITION + SPEED);
    leftServo2.write(STOP_POSITION + SPEED);
    rightServo1.write(STOP_POSITION - SPEED);
    rightServo2.write(STOP_POSITION - SPEED);
    delay(MOVE_FORWARD_TIME);
    stopMovement();
}

void moveBackwards() {
    leftServo1.write(STOP_POSITION - SPEED);
    leftServo2.write(STOP_POSITION - SPEED);
    rightServo1.write(STOP_POSITION + SPEED);
    rightServo2.write(STOP_POSITION + SPEED);
    delay(MOVE_FORWARD_TIME);
    stopMovement();
}

void fullTurnRight() {
    leftServo1.write(STOP_POSITION + SPEED);
    leftServo2.write(STOP_POSITION + SPEED);
    rightServo1.write(STOP_POSITION + SPEED);
    rightServo2.write(STOP_POSITION + SPEED);
    delay(TURN_DURATION);
    stopMovement();
}

void fullTurnLeft() {
    leftServo1.write(STOP_POSITION - SPEED);
    leftServo2.write(STOP_POSITION - SPEED);
    rightServo1.write(STOP_POSITION - SPEED);
    rightServo2.write(STOP_POSITION - SPEED);
    delay(TURN_DURATION);
    stopMovement();
}

void steerRight() {
    leftServo1.write(STOP_POSITION + SPEED);
    leftServo2.write(STOP_POSITION + SPEED);
    rightServo1.write(STOP_POSITION);
    rightServo2.write(STOP_POSITION);
    delay(STEER_DURATION);
}

void steerLeft() {
    leftServo1.write(STOP_POSITION);
    leftServo2.write(STOP_POSITION);
    rightServo1.write(STOP_POSITION - SPEED);
    rightServo2.write(STOP_POSITION - SPEED);
    delay(STEER_DURATION);
}

void stopMovement() {
    leftServo1.write(STOP_POSITION);
    leftServo2.write(STOP_POSITION);
    rightServo1.write(STOP_POSITION);
    rightServo2.write(STOP_POSITION);
    delay(500);
}

// ------------------ Decision Making ---------------------
void checkAndMove() {
    float frontDistance = getFrontDistance();
    float rightDistance = getRightDistance();
    float leftDistance = getLeftDistance();

    Serial.print("Front: "); Serial.print(frontDistance);
    Serial.print(" cm | Right: "); Serial.print(rightDistance);
    Serial.print(" cm | Left: "); Serial.println(leftDistance);

    if (rightDistance > MIN_DISTANCE && rightDistance < IGNORE_DISTANCE) {
        Serial.println("Right side open, turning RIGHT.");
        fullTurnRight(); moveForwards(); return;
    }
    if (frontDistance > FRONT_TRIGGER_DISTANCE) {
        Serial.println("Front open, moving FORWARD.");
        moveForwards(); return;
    }
    if (frontDistance < FRONT_TRIGGER_DISTANCE && rightDistance < MIN_DISTANCE) {
        Serial.println("Front and Right blocked, turning LEFT.");
        fullTurnLeft(); moveForwards(); return;
    }
    if (frontDistance < FRONT_TRIGGER_DISTANCE && leftDistance < MIN_DISTANCE) {
        Serial.println("Front and Left blocked, turning RIGHT.");
        fullTurnRight(); moveForwards(); return;
    }
    if (frontDistance < FRONT_TRIGGER_DISTANCE && rightDistance < MIN_DISTANCE && leftDistance < MIN_DISTANCE) {
        Serial.println("All sides blocked, moving BACKWARDS.");
        moveBackwards(); return;
    }
}

void adjustSteering() {
    float rightDistance = getRightDistance();
    float leftDistance = getLeftDistance();

    if (rightDistance < MIN_DISTANCE) {
        Serial.println("Right too close, steering LEFT.");
        steerLeft();
    }
    if (rightDistance > MAX_DISTANCE && rightDistance < IGNORE_DISTANCE) {
        Serial.println("Right too far, steering RIGHT.");
        steerRight();
    }
    if (leftDistance < MIN_DISTANCE) {
        Serial.println("Left too close, steering RIGHT.");
        steerRight();
    }
    if (leftDistance > MAX_DISTANCE && leftDistance < IGNORE_DISTANCE) {
        Serial.println("Left too far, steering LEFT.");
        steerLeft();
    }
}

// ------------------ Main Loop ---------------------
void setup() {
    setupHardware();
}

void loop() {
    checkAndMove();
    adjustSteering();
}
