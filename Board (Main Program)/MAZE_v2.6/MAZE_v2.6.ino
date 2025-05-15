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

#define STOP_POSITION 90
#define SPEED 10
#define TURN_DURATION 4400  // 4.4 seconds for manual turning
#define STEER_DURATION 200  // 0.2 seconds for automatic steering

#define MIN_DISTANCE 3.5
#define MAX_DISTANCE 5.0
#define FRONT_TRIGGER_DISTANCE 5.0

bool goCrazyMode = false;  // Automatic mode (disabled by default)

// ----------------- Hardware Level ---------------------

Servo leftServo1, leftServo2, rightServo1, rightServo2;

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
}

// ------------------ Sensor Level ---------------------

int measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    
    if (duration == 0) {
        return 10;
    }

    return (duration * 0.0343) / 2;
}

int getFrontDistance() {
    return measureDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
}

int getRightDistance() {
    return measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
}

// ------------------ Logging Function ---------------------

void showLog() {
    Serial.print("Front: ");
    Serial.print(getFrontDistance());
    Serial.print(" cm | Right: ");
    Serial.print(getRightDistance());
    Serial.println(" cm");
}

// ------------------ Movement Level ---------------------

void moveForwards() {
    Serial.println("MOVEMENT: Moving FORWARDS");
    leftServo1.write(STOP_POSITION + SPEED);
    leftServo2.write(STOP_POSITION + SPEED);
    rightServo1.write(STOP_POSITION - SPEED);
    rightServo2.write(STOP_POSITION - SPEED);
}

void moveBackwards() {
    Serial.println("MOVEMENT: Moving BACKWARDS");
    leftServo1.write(STOP_POSITION - SPEED);
    leftServo2.write(STOP_POSITION - SPEED);
    rightServo1.write(STOP_POSITION + SPEED);
    rightServo2.write(STOP_POSITION + SPEED);
}

void moveRight() {
    Serial.println("MOVEMENT: Turning RIGHT for 4 seconds");
    leftServo1.write(STOP_POSITION + SPEED);
    leftServo2.write(STOP_POSITION + SPEED);
    rightServo1.write(STOP_POSITION + SPEED);
    rightServo2.write(STOP_POSITION + SPEED);
    delay(TURN_DURATION);
    stopMovement();
}

void moveLeft() {
    Serial.println("MOVEMENT: Turning LEFT for 4 seconds");
    leftServo1.write(STOP_POSITION - SPEED);
    leftServo2.write(STOP_POSITION - SPEED);
    rightServo1.write(STOP_POSITION - SPEED);
    rightServo2.write(STOP_POSITION - SPEED);
    delay(TURN_DURATION);
    stopMovement();
}

void steerRight() {
    Serial.println("MOVEMENT: Steering RIGHT for 0.2 seconds");
    leftServo1.write(STOP_POSITION + SPEED);
    leftServo2.write(STOP_POSITION + SPEED);
    rightServo1.write(STOP_POSITION);
    rightServo2.write(STOP_POSITION);
    delay(STEER_DURATION);
}

void steerLeft() {
    Serial.println("MOVEMENT: Steering LEFT for 0.2 seconds");
    leftServo1.write(STOP_POSITION);
    leftServo2.write(STOP_POSITION);
    rightServo1.write(STOP_POSITION - SPEED);
    rightServo2.write(STOP_POSITION - SPEED);
    delay(STEER_DURATION);
}

void stopMovement() {
    Serial.println("MOVEMENT: Stopping");
    leftServo1.write(STOP_POSITION);
    leftServo2.write(STOP_POSITION);
    rightServo1.write(STOP_POSITION);
    rightServo2.write(STOP_POSITION);
}

// ------------------ Automatic (Go Crazy Mode) ---------------------

void goCrazy() {
    Serial.println("MOVEMENT: Automatic Mode Activated!");
    
    while (goCrazyMode) {
        showLog();

        int frontDistance = getFrontDistance();
        int rightDistance = getRightDistance();

        if (frontDistance < FRONT_TRIGGER_DISTANCE) {
            Serial.println("MOVEMENT: Obstacle detected! Turning LEFT");
            moveLeft();
        } 
        else if (rightDistance < MIN_DISTANCE) {
            Serial.println("MOVEMENT: Too close to wall! Steering LEFT");
            steerLeft();
        } 
        else if (rightDistance > MAX_DISTANCE) {
            Serial.println("MOVEMENT: Too far from wall! Steering RIGHT");
            steerRight();
        } 
        else {
            Serial.println("MOVEMENT: Moving FORWARDS");
            moveForwards();
        }

        delay(100); // Short delay for better sensor response
    }

    Serial.println("MOVEMENT: Automatic Mode Deactivated.");
}

// ------------------ Command Processing ---------------------

void processCommand(String command) {
    command.trim();

    Serial.print("MOVEMENT: Received Command - ");
    Serial.println(command);

    if (command == "FORWARDS") {
        moveForwards();
    } 
    else if (command == "BACKWARDS") {
        moveBackwards();
    } 
    else if (command == "TURNRIGHT") {
        moveRight();
    } 
    else if (command == "TURNLEFT") {
        moveLeft();
    } 
    else if (command == "STOP") {
        goCrazyMode = false; // Stop automatic mode
        stopMovement();
    } 
    else if (command == "AUTOMATIC") {
        goCrazyMode = true;
        goCrazy();
    } 
    else {
        Serial.println("ERROR: Unknown Command");
        stopMovement();
    }
}

// ------------------ Arduino Setup & Loop ---------------------

void setup() {
    setupHardware(); // Initialize hardware (motors, sensors, serial communication)
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }
}
