#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ----------------- Definitions Class ------------------------
class Definitions {
public:
    // Motor pins
    static const int LEFT_MOTOR_PIN = 13;
    static const int RIGHT_MOTOR_PIN = 10;

    // Ultrasonic sensor pins
    static const int FRONT_LEFT_TRIG_PIN = 67;
    static const int FRONT_LEFT_ECHO_PIN = 66;
    static const int FRONT_RIGHT_TRIG_PIN = 16;
    static const int FRONT_RIGHT_ECHO_PIN = 17;
    static const int SIDE_LEFT_TRIG_PIN = 65;
    static const int SIDE_LEFT_ECHO_PIN = 64;
    static const int SIDE_RIGHT_TRIG_PIN = 8;
    static const int SIDE_RIGHT_ECHO_PIN = 7;

    // Color sensor pin
    static const int COLOR_SENSOR_PIN = A0;

    // LCD pins (I2C address and dimensions)
    static const int LCD_ADDRESS = 0x27;
    static const int LCD_COLUMNS = 16;
    static const int LCD_ROWS = 2;

    // Motor control
    static const int STOP_POSITION = 90;
    static const int SPEED = 60;
    static const int TURN_CLEAR_THRESHOLD = 10;

    // LCD object
    static LiquidCrystal_I2C lcd;

    static void pinouts() {
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
};

LiquidCrystal_I2C Definitions::lcd(Definitions::LCD_ADDRESS, Definitions::LCD_COLUMNS, Definitions::LCD_ROWS);

// ----------------- Motors Class ------------------------
class Motors {
private:
    Servo leftMotor, rightMotor;

public:
    Motors() {
        leftMotor.attach(Definitions::LEFT_MOTOR_PIN);
        rightMotor.attach(Definitions::RIGHT_MOTOR_PIN);
    }

    void moveForward() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);

        // Check for obstacles while moving forward
        float frontLeftDistance = getFrontLeftDistance();
        float frontRightDistance = getFrontRightDistance();
        float sideLeftDistance = getSideLeftDistance();
        float sideRightDistance = getSideRightDistance();

        updateLCD(frontLeftDistance, frontRightDistance);

        // Handle obstacles or turn logic
        if (frontLeftDistance < Definitions::TURN_CLEAR_THRESHOLD || sideLeftDistance < Definitions::TURN_CLEAR_THRESHOLD) {
            fullTurnLeft();  // Turn left if the left side is blocked
        } else if (frontRightDistance < Definitions::TURN_CLEAR_THRESHOLD || sideRightDistance < Definitions::TURN_CLEAR_THRESHOLD) {
            fullTurnRight();  // Turn right if the right side is blocked
        }

        // Update the position after moving
        updatePosition();
    }

    void stop() {
        leftMotor.write(Definitions::STOP_POSITION);
        rightMotor.write(Definitions::STOP_POSITION);

        // Check for obstacles and possible turns when stopped
        float frontLeftDistance = getFrontLeftDistance();
        float frontRightDistance = getFrontRightDistance();
        float sideLeftDistance = getSideLeftDistance();
        float sideRightDistance = getSideRightDistance();

        updateLCD(frontLeftDistance, frontRightDistance);

        if (frontLeftDistance < Definitions::TURN_CLEAR_THRESHOLD || sideLeftDistance < Definitions::TURN_CLEAR_THRESHOLD) {
            fullTurnLeft();
        } else if (frontRightDistance < Definitions::TURN_CLEAR_THRESHOLD || sideRightDistance < Definitions::TURN_CLEAR_THRESHOLD) {
            fullTurnRight();
        }
    }

    void fullTurnLeft() {
        while (true) {
            leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
            rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);

            float frontLeft = getFrontLeftDistance();
            float frontRight = getFrontRightDistance();

            updateLCD(frontLeft, frontRight);

            if (frontLeft > Definitions::TURN_CLEAR_THRESHOLD && frontRight > Definitions::TURN_CLEAR_THRESHOLD) {
                break;
            }
            delay(100);
        }
        stop();
    }

    void fullTurnRight() {
        while (true) {
            leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
            rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);

            float frontLeft = getFrontLeftDistance();
            float frontRight = getFrontRightDistance();

            updateLCD(frontLeft, frontRight);

            if (frontLeft > Definitions::TURN_CLEAR_THRESHOLD && frontRight > Definitions::TURN_CLEAR_THRESHOLD) {
                break;
            }
            delay(100);
        }
        stop();
    }

    float getFrontLeftDistance() {
        digitalWrite(Definitions::FRONT_LEFT_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(Definitions::FRONT_LEFT_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(Definitions::FRONT_LEFT_TRIG_PIN, LOW);
        long duration = pulseIn(Definitions::FRONT_LEFT_ECHO_PIN, HIGH, 30000);
        return (duration == 0) ? 10 : (duration * 0.0343) / 2;
    }

    float getFrontRightDistance() {
        digitalWrite(Definitions::FRONT_RIGHT_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(Definitions::FRONT_RIGHT_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(Definitions::FRONT_RIGHT_TRIG_PIN, LOW);
        long duration = pulseIn(Definitions::FRONT_RIGHT_ECHO_PIN, HIGH, 30000);
        return (duration == 0) ? 10 : (duration * 0.0343) / 2;
    }

    float getSideLeftDistance() {
        digitalWrite(Definitions::SIDE_LEFT_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(Definitions::SIDE_LEFT_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(Definitions::SIDE_LEFT_TRIG_PIN, LOW);
        long duration = pulseIn(Definitions::SIDE_LEFT_ECHO_PIN, HIGH, 30000);
        return (duration == 0) ? 10 : (duration * 0.0343) / 2;
    }

    float getSideRightDistance() {
        digitalWrite(Definitions::SIDE_RIGHT_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(Definitions::SIDE_RIGHT_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(Definitions::SIDE_RIGHT_TRIG_PIN, LOW);
        long duration = pulseIn(Definitions::SIDE_RIGHT_ECHO_PIN, HIGH, 30000);
        return (duration == 0) ? 10 : (duration * 0.0343) / 2;
    }

    void updateLCD(float frontLeft, float frontRight) {
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("FrontL: " + String(frontLeft) + " cm");
        Definitions::lcd.setCursor(0, 1);
        Definitions::lcd.print("FrontR: " + String(frontRight) + " cm");
    }

    void updatePosition() {
        // Update X and Y coordinates
        positionX += cos(radians(currentAngle)) * speed;
        positionY += sin(radians(currentAngle)) * speed;

        // Update LCD or Serial for debugging
        Serial.print("X: ");
        Serial.print(positionX);
        Serial.print(" Y: ");
        Serial.println(positionY);
    }

    float positionX = 0;  // Current X coordinate
    float positionY = 0;  // Current Y coordinate
    float currentAngle = 0;  // Robot's current angle in degrees
    float speed = 0.1;  // Speed factor (modify as needed)
};

// ----------------- Sensors Class ------------------------
class Sensors {
private:
    int colorSensorPin;

public:
    Sensors(int colorPin) : colorSensorPin(colorPin) {}

    int detectColor() {
        int colorValue = analogRead(colorSensorPin);

        if (colorValue > 900) {
            return 1; // White
        } else if (colorValue > 600) {
            return 2; // Gray
        } else if (colorValue > 300) {
            return 3; // Black
        } else {
            return 4; // Blue
        }
    }
};

// ----------------- Robot Class ------------------------
class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor;
    Sensors frontRightSensor;

public:
    Robot() : frontLeftSensor(Definitions::FRONT_LEFT_TRIG_PIN), frontRightSensor(Definitions::FRONT_RIGHT_TRIG_PIN) {}

    void moveForward() {
        motors.moveForward();  // Move forward and check for obstacles
    }

    void stopMovement() {
        motors.stop();  // Stop the robot and check for possible turns
    }
};

// ------------------ Main Control ---------------------
Robot robot;

void setup() {
    Serial.begin(9600);
    Definitions::pinouts();
}

void loop() {
    robot.moveForward();  // Continuously move forward and check for obstacles
    delay(100);  // Short delay to avoid rapid polling
}
