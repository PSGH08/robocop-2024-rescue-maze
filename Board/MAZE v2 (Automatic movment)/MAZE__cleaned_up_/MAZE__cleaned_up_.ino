#include <Servo.h>

// ----------------------
// BASE LAYER: Hardware & Settings
// ----------------------

Servo leftServo1, leftServo2, rightServo1, rightServo2;

int getLeftServo1Pin() { return 9; }
int getLeftServo2Pin() { return 10; }
int getRightServo1Pin() { return 11; }
int getRightServo2Pin() { return 12; }

int getFrontTrigPin() { return 5; }
int getFrontEchoPin() { return 6; }
int getRightTrigPin() { return 4; }
int getRightEchoPin() { return 3; }

int getStopPosition() { return 90; }
int getSpeed() { return 10; }

unsigned long getStopInterval() { return 8000; }
unsigned long getStopDuration() { return 1000; }
unsigned long getRightSensorResetTime() { return 3000; }

float getMinDistance() { return 3.5; }
float getMaxDistance() { return 5.0; }
float getFrontTriggerDistance() { return 5.0; }

bool goCrazyMode = true;
int rightSensorTriggerCount = 0;
unsigned long lastStopTime = 0;
unsigned long stopStartTime = 0;
unsigned long lastRightTriggerTime = 0;

void setupHardware() {
    Serial.begin(9600);

    leftServo1.attach(getLeftServo1Pin());
    leftServo2.attach(getLeftServo2Pin());
    rightServo1.attach(getRightServo1Pin());
    rightServo2.attach(getRightServo2Pin());

    pinMode(getFrontTrigPin(), OUTPUT);
    pinMode(getFrontEchoPin(), INPUT);
    pinMode(getRightTrigPin(), OUTPUT);
    pinMode(getRightEchoPin(), INPUT);

    lastStopTime = millis();
}

// ----------------------
// SECOND LAYER: Movement Functions
// ----------------------

void moveForwards() {
    leftServo1.write(getStopPosition() + getSpeed());
    leftServo2.write(getStopPosition() + getSpeed());
    rightServo1.write(getStopPosition() - getSpeed());
    rightServo2.write(getStopPosition() - getSpeed());
}

void moveBackwards() {
    leftServo1.write(getStopPosition() - getSpeed());
    leftServo2.write(getStopPosition() - getSpeed());
    rightServo1.write(getStopPosition() + getSpeed());
    rightServo2.write(getStopPosition() + getSpeed());
}

void moveRight() {
    leftServo1.write(getStopPosition() + getSpeed());  
    leftServo2.write(getStopPosition() + getSpeed());
    rightServo1.write(getStopPosition() + getSpeed());  
    rightServo2.write(getStopPosition() + getSpeed());
}

void moveLeft() {
    leftServo1.write(getStopPosition() - getSpeed());  
    leftServo2.write(getStopPosition() - getSpeed());
    rightServo1.write(getStopPosition() - getSpeed());  
    rightServo2.write(getStopPosition() - getSpeed());
}

void stopMovement() {
    leftServo1.write(getStopPosition());
    leftServo2.write(getStopPosition());
    rightServo1.write(getStopPosition());
    rightServo2.write(getStopPosition());
}

// ----------------------
// SENSOR READING FUNCTIONS
// ----------------------

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

// ----------------------
// FINAL LAYER: MAIN CONTROL FUNCTION
// ----------------------

void handleMovement() {
    unsigned long currentTime = millis();

    if (currentTime - lastStopTime >= getStopInterval() && stopStartTime == 0) {
        lastStopTime = currentTime;
        stopStartTime = currentTime;
        stopMovement();
    }

    if (stopStartTime != 0 && currentTime - stopStartTime >= getStopDuration()) {
        stopStartTime = 0;
        moveForwards();
    }

    if (stopStartTime == 0) {
        float frontDistance = measureDistance(getFrontTrigPin(), getFrontEchoPin());
        float rightDistance = measureDistance(getRightTrigPin(), getRightEchoPin());

        Serial.print("Front Distance: ");
        Serial.print(frontDistance);
        Serial.print(" cm | Right Distance: ");
        Serial.println(rightDistance);

        if (rightDistance == 0) {
            moveForwards();
            return;
        }

        if (frontDistance < getFrontTriggerDistance()) {
            moveLeft();
            delay(4000);  
            return;
        }

        if (rightSensorTriggerCount >= 2 && (currentTime - lastRightTriggerTime >= getRightSensorResetTime())) {
            rightSensorTriggerCount = 0;
        }

        if (rightSensorTriggerCount < 2) {
            if (rightDistance < getMinDistance()) {
                moveLeft();
                delay(200);
                moveForwards();
                rightSensorTriggerCount++;
                lastRightTriggerTime = millis();
                return;
            } 

            if (rightDistance > getMaxDistance()) {
                moveRight();
                delay(200);
                moveForwards();
                rightSensorTriggerCount++;
                lastRightTriggerTime = millis();
                return;
            }
        }

        moveForwards();
    }
}

void loop() {
    if (goCrazyMode) {
        handleMovement();
    }
}
