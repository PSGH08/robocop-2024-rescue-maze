#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

// LCD initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin Definitions
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

// Constants
#define STOP_POSITION 90
#define SPEED 30
#define MOVE_DURATION 4000
#define TURN_DURATION 2700
#define THROW_DURATION 500

class Motors {
private:
    Servo leftMotor, rightMotor;

public:
    void begin() {
        leftMotor.attach(LEFT_MOTOR_PIN);
        rightMotor.attach(RIGHT_MOTOR_PIN);
        pinMode(MEDKIT_PIN, OUTPUT);
    }

    void moveForward() {
        leftMotor.write(STOP_POSITION - SPEED);
        rightMotor.write(STOP_POSITION + SPEED);
        lcd.setCursor(0, 0);
        lcd.print("Moving Forward ");
    }

    void stop() {
        leftMotor.write(STOP_POSITION);
        rightMotor.write(STOP_POSITION);
        lcd.setCursor(0, 0);
        lcd.print("Stopped       ");
    }

    void steerLeft() {
        leftMotor.write(STOP_POSITION + SPEED);
        rightMotor.write(STOP_POSITION + SPEED);
        delay(200);
        stop();
        lcd.setCursor(0, 0);
        lcd.print("Steering Left ");
    }

    void steerRight() {
        leftMotor.write(STOP_POSITION - SPEED);
        rightMotor.write(STOP_POSITION - SPEED);
        delay(200);
        stop();
        lcd.setCursor(0, 0);
        lcd.print("Steering Right");
    }

    void turnLeft() {
        leftMotor.write(STOP_POSITION + SPEED);
        rightMotor.write(STOP_POSITION + SPEED);
        delay(TURN_DURATION);
        stop();
        lcd.setCursor(0, 0);
        lcd.print("Turned Left   ");
    }

    void turnRight() {
        leftMotor.write(STOP_POSITION - SPEED);
        rightMotor.write(STOP_POSITION - SPEED);
        delay(TURN_DURATION);
        stop();
        lcd.setCursor(0, 0);
        lcd.print("Turned Right  ");
    }

    void moveBackward(int duration) {
        leftMotor.write(STOP_POSITION + SPEED);
        rightMotor.write(STOP_POSITION - SPEED);
        delay(duration);
        stop();
        lcd.setCursor(0, 0);
        lcd.print("Moving Back   ");
    }

    void throwMedkit() {
        digitalWrite(MEDKIT_PIN, HIGH);
        delay(THROW_DURATION);
        digitalWrite(MEDKIT_PIN, LOW);
        lcd.setCursor(0, 0);
        lcd.print("Medkit Thrown ");
    }
};

class Sensors {
private:
    int trigPin, echoPin;
    QMC5883LCompass compass;

public:
    Sensors(int trig, int echo) : trigPin(trig), echoPin(echo) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    void begin() {
        Wire.begin();
        compass.init();
        compass.setCalibration(-240, 570, -690, 590, -320, 570);
    }

    float getDistance() {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        long duration = pulseIn(echoPin, HIGH, 30000);
        return (duration * 0.0343) / 2;
    }

    int detectColor() {
        int colorValue = analogRead(COLOR_SENSOR_PIN);
        if (colorValue > 900) return 1;
        else if (colorValue > 600) return 2;
        else if (colorValue > 300) return 3;
        else return 4;
    }

    float getCompassHeading() {
        compass.read();
        int x = compass.getX();
        int y = compass.getY();
        if (x == 0 && y == 0) return -1;
        
        float heading = atan2(y, x) * (180.0 / PI);
        if (heading < 0) heading += 360;
        heading += 5;
        return fmod(heading, 360);
    }
};

class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor{FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN};
    Sensors frontRightSensor{FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN};
    Sensors sideLeftSensor{SIDE_LEFT_TRIG_PIN, SIDE_LEFT_ECHO_PIN};
    Sensors sideRightSensor{SIDE_RIGHT_TRIG_PIN, SIDE_RIGHT_ECHO_PIN};
    
    unsigned long lastMoveTime;
    bool isMoving;
    float initialHeading;

    void updateLEDs() {
        digitalWrite(LED_MOVING, isMoving ? HIGH : LOW);
        digitalWrite(LED_STOPPED, isMoving ? LOW : HIGH);
    }

    void verifyTurn(float targetAngleChange) {
        float finalHeading = frontLeftSensor.getCompassHeading();
        float actualAngleChange = fmod(finalHeading - initialHeading + 360, 360);
        
        if (actualAngleChange > 180) {
            actualAngleChange -= 360;
        }
        
        float error = actualAngleChange - targetAngleChange;
        
        if (abs(error) > 5) {
            if (error > 0) {
                motors.steerLeft();
            } else {
                motors.steerRight();
            }
            delay(100);
            motors.stop();
        }
    }

public:
    Robot() : isMoving(false) {
        lastMoveTime = millis();
    }

    void begin() {
        // Initialize pins
        pinMode(LED_STARTUP, OUTPUT);
        pinMode(LED_MOVING, OUTPUT);
        pinMode(LED_STOPPED, OUTPUT);
        
        pinMode(COLOR_SENSOR_PIN1, OUTPUT);
        pinMode(COLOR_SENSOR_PIN2, OUTPUT);
        pinMode(COLOR_SENSOR_PIN3, OUTPUT);
        pinMode(COLOR_SENSOR_PIN4, OUTPUT);
        pinMode(COLOR_SENSOR_PIN5, INPUT);
        
        // Initialize components
        lcd.begin(16, 2);
        lcd.backlight();
        motors.begin();
        frontLeftSensor.begin();
        frontRightSensor.begin();
        sideLeftSensor.begin();
        sideRightSensor.begin();
    }

    void turnLeft() {
        initialHeading = frontLeftSensor.getCompassHeading();
        motors.turnLeft();
        verifyTurn(90);
    }

    void turnRight() {
        initialHeading = frontLeftSensor.getCompassHeading();
        motors.turnRight();
        verifyTurn(-90);
    }

    void updateState() {
        if (millis() - lastMoveTime >= MOVE_DURATION) {
            float leftFront = frontLeftSensor.getDistance();
            float rightFront = frontRightSensor.getDistance();
            float leftSide = sideLeftSensor.getDistance();
            float rightSide = sideRightSensor.getDistance();
            int color = frontLeftSensor.detectColor();

            if (leftFront < 5.0 && rightFront < 5.0 && rightSide < 8.0) {
                turnLeft();
                isMoving = false;
            } 
            else if (leftFront < 5.0 && rightFront < 5.0 && leftSide < 8.0) {
                turnRight();
                isMoving = false;
            }
            else if (rightSide > 6.0 && rightSide < 8.0 && leftSide > 6.0 && leftSide < 8.0) {
                motors.moveForward();
                isMoving = true;
            } 
            else if (leftSide < 6.0) {
                motors.steerRight();
                isMoving = false;
            }
            else if (leftSide > 8.0) {
                motors.steerLeft();
                isMoving = false;
            }
            else if (rightSide > 8.0) {
                motors.steerRight();
                isMoving = false;
            }
            else if (rightSide < 6.0) {
                motors.steerLeft();
                isMoving = false;
            }
            else {
                motors.moveForward();
                isMoving = true;
            }

            if (color == 3 || color == 4) {
                motors.moveBackward(1000);
                isMoving = true;
            }

            updateLEDs();
            lastMoveTime = millis();
        }
    }

    void throwMedkit() {
        motors.throwMedkit();
    }
};

Robot robot;

void setup() {
    Serial.begin(9600);
    robot.begin();
    
    digitalWrite(LED_STARTUP, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(3000);
    digitalWrite(LED_STARTUP, LOW);
    lcd.clear();
    lcd.print("Robot Ready");
}

void loop() {
    robot.updateState();
    delay(100);
}
