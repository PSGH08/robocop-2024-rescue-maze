#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

enum Direction { NORTH, EAST, SOUTH, WEST };
Direction currentDirection = NORTH;

// Simple memory storage system
const int MAX_MEMORY = 50;
String memoryEntries[MAX_MEMORY];
int memoryIndex = 0;

int positionX = 0;
int positionY = 0;

// ----------------- Definitions ------------------------
const int LEFT_MOTOR_PIN = 13;
const int RIGHT_MOTOR_PIN = 10;

const int FRONT_LEFT_TRIG_PIN = 67;
const int FRONT_LEFT_ECHO_PIN = 66;
const int FRONT_RIGHT_TRIG_PIN = 16;
const int FRONT_RIGHT_ECHO_PIN = 17;
const int SIDE_LEFT_TRIG_PIN = 65;
const int SIDE_LEFT_ECHO_PIN = 64;
const int SIDE_RIGHT_TRIG_PIN = 8;
const int SIDE_RIGHT_ECHO_PIN = 7;

const int COLOR_SENSOR_PIN = A0;

const int LCD_ADDRESS = 0x27;
const int LCD_COLUMNS = 16;
const int LCD_ROWS = 2;

const int STOP_POSITION = 90;
const int SPEED = 20;
const int TURN_CLEAR_THRESHOLD = 10;

const int MIN_DISTANCE = 7;
const int MAX_DISTANCE = 8;
const int MOVE_DURATION = 3000;
const int STOP_DURATION = 2500;

unsigned long lastActionTime = 0;
bool isMoving = true;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

void pinouts() {
    pinMode(LEFT_MOTOR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN, OUTPUT);
    pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
    pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
    pinMode(SIDE_LEFT_TRIG_PIN, OUTPUT);
    pinMode(SIDE_LEFT_ECHO_PIN, INPUT);
    pinMode(SIDE_RIGHT_TRIG_PIN, OUTPUT);
    pinMode(SIDE_RIGHT_ECHO_PIN, INPUT);
    pinMode(COLOR_SENSOR_PIN, INPUT);

    lcd.begin(LCD_COLUMNS, LCD_ROWS);
    lcd.backlight();
}

// ----------------- Motors ------------------------
Servo leftMotor, rightMotor;

void setupMotors() {
    leftMotor.attach(LEFT_MOTOR_PIN);
    rightMotor.attach(RIGHT_MOTOR_PIN);
}

String getLastColor() {
    int colorVal = analogRead(COLOR_SENSOR_PIN);
    if (colorVal > 900) return "White";
    if (colorVal > 600) return "Gray";
    if (colorVal > 300) return "Dark";
    return "Blue";
}

void saveDecision(String action) {
    if (memoryIndex < MAX_MEMORY) {
        memoryEntries[memoryIndex] = "X:" + String(positionX) + " Y:" + String(positionY) + 
                                  " " + action + " Color:" + getLastColor();
        memoryIndex++;
    }
}

void updateLCD(float frontLeft, float frontRight, float sideRight, float sideLeft) {
    lcd.setCursor(0, 0);
    lcd.print("FL:" + String(frontLeft) + " FR:" + String(frontRight));
    lcd.setCursor(0, 1);
    lcd.print("SL:" + String(sideLeft) + " SR:" + String(sideRight));
}

void displayStatus(String text) {
    lcd.setCursor(0, 1);
    lcd.print("X:" + String(positionX) + " Y:" + String(positionY) + " " + text + "    ");
}

float readUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration == 0) ? 10 : (duration * 0.0343) / 2;
}

void moveForward() {
    leftMotor.write(STOP_POSITION + SPEED);
    rightMotor.write(STOP_POSITION - SPEED);

    float frontLeft = readUltrasonic(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN);
    float frontRight = readUltrasonic(FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN);
    float sideLeft = readUltrasonic(SIDE_LEFT_TRIG_PIN, SIDE_LEFT_ECHO_PIN);
    float sideRight = readUltrasonic(SIDE_RIGHT_TRIG_PIN, SIDE_RIGHT_ECHO_PIN);
    
    updateLCD(frontLeft, frontRight, sideRight, sideLeft);
    
    // Update position based on direction
    switch (currentDirection) {
        case NORTH: positionY++; break;
        case SOUTH: positionY--; break;
        case EAST:  positionX++; break;
        case WEST:  positionX--; break;
    }
    
    saveDecision("Forward");

    if (sideRight < MIN_DISTANCE) {
        steerLeft();
    } else if (sideLeft < MIN_DISTANCE) {
        steerRight();
    }
}

void stopMotors() {
    leftMotor.write(STOP_POSITION);
    rightMotor.write(STOP_POSITION);

    float frontLeft = readUltrasonic(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN);
    float frontRight = readUltrasonic(FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN);
    float sideLeft = readUltrasonic(SIDE_LEFT_TRIG_PIN, SIDE_LEFT_ECHO_PIN);
    float sideRight = readUltrasonic(SIDE_RIGHT_TRIG_PIN, SIDE_RIGHT_ECHO_PIN);
    
    updateLCD(frontLeft, frontRight, sideRight, sideLeft);
    saveDecision("Stopped");

    if ((frontLeft < TURN_CLEAR_THRESHOLD || sideLeft < TURN_CLEAR_THRESHOLD) && sideRight > MIN_DISTANCE) {
        fullTurnLeft();
    } else if ((frontRight < TURN_CLEAR_THRESHOLD || sideRight < TURN_CLEAR_THRESHOLD) && sideLeft > MIN_DISTANCE) {
        fullTurnRight();
    }
}

void fullTurnLeft() {
    currentDirection = static_cast<Direction>((currentDirection + 3) % 4);
    saveDecision("Turned Left");
    displayStatus("Turned Left");

    while (true) {
        leftMotor.write(STOP_POSITION - SPEED);
        rightMotor.write(STOP_POSITION - SPEED);
        delay(100);
    }
}

void fullTurnRight() {
    currentDirection = static_cast<Direction>((currentDirection + 1) % 4);
    saveDecision("Turned Right");
    displayStatus("Turned Right");

    while (true) {
        leftMotor.write(STOP_POSITION + SPEED);
        rightMotor.write(STOP_POSITION + SPEED);
        delay(100);
    }
}

void steerLeft() {
    leftMotor.write(STOP_POSITION + SPEED - 5);
    rightMotor.write(STOP_POSITION - SPEED);
    saveDecision("Steer Left");
}

void steerRight() {
    leftMotor.write(STOP_POSITION + SPEED + 5);
    rightMotor.write(STOP_POSITION - SPEED);
    saveDecision("Steer Right");
}

void moveBackward() {
    leftMotor.write(STOP_POSITION - SPEED);
    rightMotor.write(STOP_POSITION + SPEED);
    saveDecision("Backward");
}

// ----------------- Main Program ------------------------
void setup() {
    pinouts();
    setupMotors();
}

void loop() {
    unsigned long currentTime = millis();

    if (isMoving && currentTime - lastActionTime >= MOVE_DURATION) {
        stopMotors();
        lastActionTime = currentTime;
        isMoving = false;
    } else if (!isMoving && currentTime - lastActionTime >= STOP_DURATION) {
        moveForward();
        lastActionTime = currentTime;
        isMoving = true;
    }

    delay(100);
}
