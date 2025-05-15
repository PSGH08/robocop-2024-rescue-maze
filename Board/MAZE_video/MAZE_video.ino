#include <Servo.h>

#define LEFT_SERVO_PIN  13
#define RIGHT_SERVO_PIN 10
#define STOP_POSITION 90
#define SPEED 30
#define MOVE_DURATION 3000  // Move forward for 3 seconds
#define STOP_DURATION 2000  // Stop for 2 seconds

Servo leftServo, rightServo;

#define MEDKIT_MOTOR_PIN  9
#define MEDKIT_SPIN_DURATION 2000  // Spin motor for 2 seconds

void setup() {
    pinMode(MEDKIT_MOTOR_PIN, OUTPUT);
    Serial.begin(9600);
    leftServo.attach(LEFT_SERVO_PIN);
    rightServo.attach(RIGHT_SERVO_PIN);
}

void moveForwards() {
    Serial.println("Moving FORWARDS");
    leftServo.write(STOP_POSITION + SPEED);
    rightServo.write(STOP_POSITION - SPEED);
    delay(MOVE_DURATION);
    stopMovement();
}

void moveLeft() {
    Serial.println("Turning LEFT");
    leftServo.write(STOP_POSITION - SPEED);
    rightServo.write(STOP_POSITION - SPEED);
    delay(MOVE_DURATION);
    stopMovement();
}

void stopMovement() {
    Serial.println("Stopping");
    leftServo.write(STOP_POSITION);
    rightServo.write(STOP_POSITION);
    delay(STOP_DURATION);
}

void activateMedkitMotor() {
    Serial.println("Throwing Medkit");
    digitalWrite(MEDKIT_MOTOR_PIN, HIGH);
    delay(MEDKIT_SPIN_DURATION);
    digitalWrite(MEDKIT_MOTOR_PIN, LOW);
    delay(STOP_DURATION);
}

void loop() {
    for (int i = 0; i < 4; i++) {
        moveForwards();
    }
    moveLeft();
    activateMedkitMotor();
    moveForwards();
    moveLeft();
    while (true);  // Stop further execution
}
