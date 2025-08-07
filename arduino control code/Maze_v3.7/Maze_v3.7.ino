#include <Wire.h>
#include <Servo.h>
#include <QMC5883LCompass.h>
#include <LiquidCrystal_I2C.h>

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

    // Compass pins
    static const int COMPASS_SDA_PIN = 43;
    static const int COMPASS_SCL_PIN = 32;

    // LCD settings
    static const int LCD_ADDRESS = 0x27;
    static const int LCD_COLUMNS = 16;
    static const int LCD_ROWS = 2;

    // Motor control
    static const int STOP_POSITION = 90;
    static const int SPEED = 60;
    static const int WHEEL_DIAMETER_CM = 7;
    static const int TARGET_DISTANCE_CM = 30;
    static const unsigned long MOVE_TIME_MS = 8180;

    // Sensor objects
    static QMC5883LCompass compass;
    static LiquidCrystal_I2C lcd;

    static void pinouts() {
        pinMode(LEFT_MOTOR_PIN, OUTPUT);
        pinMode(RIGHT_MOTOR_PIN, OUTPUT);

        // Ultrasonic sensors
        pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
        pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
        pinMode(SIDE_LEFT_TRIG_PIN, OUTPUT);
        pinMode(SIDE_LEFT_ECHO_PIN, INPUT);
        pinMode(SIDE_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(SIDE_RIGHT_ECHO_PIN, INPUT);

        pinMode(COLOR_SENSOR_PIN, INPUT);

        Wire.begin();
        compass.init();
        compass.setCalibration(-240, 570, -690, 590, -320, 570);

        lcd.begin(LCD_COLUMNS, LCD_ROWS);
        lcd.backlight();
    }
};

// Initialize static objects
QMC5883LCompass Definitions::compass;
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
        Serial.println("Moving forward...");
    }

    void stop() {
        leftMotor.write(Definitions::STOP_POSITION);
        rightMotor.write(Definitions::STOP_POSITION);
        Serial.println("Stopped.");
    }

    void turnLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(1000);
        stop();
        Serial.println("Turned left.");
    }

    void turnRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(1000);
        stop();
        Serial.println("Turned right.");
    }

    void moveBackward(int duration) {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(duration);
        stop();
        Serial.println("Moved backward.");
    }
};

// ----------------- Sensors Class ------------------------
class Sensors {
private:
    int trigPin, echoPin;

public:
    Sensors(int trig, int echo) : trigPin(trig), echoPin(echo) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    float getDistance() {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        long duration = pulseIn(echoPin, HIGH, 30000);
        float distance = (duration == 0) ? 10 : (duration * 0.0343) / 2;
        Serial.print("Ultrasonic Distance: ");
        Serial.println(distance);
        displayOnLCD("Distance: " + String(distance) + " cm", 0);
        return distance;
    }

    float getDirection() {
        int compassX = Definitions::compass.getX();
        int compassY = Definitions::compass.getY();

        if (compassX == 0 && compassY == 0) {
            Serial.println("Sensor error or interference!");
            displayOnLCD("Compass Error!", 0);
            return -1;
        }

        float compassDirection = atan2(compassY, compassX) * (180.0 / PI);
        if (compassDirection < 0) compassDirection += 360;
        Serial.print("Compass Direction: ");
        Serial.println(compassDirection);
        displayOnLCD("Direction: " + String(compassDirection) + "°", 0);
        return compassDirection;
    }

    int detectColor() {
        int colorValue = analogRead(Definitions::COLOR_SENSOR_PIN);

        if (colorValue > 900) {
            Serial.println("Detected color: White");
            displayOnLCD("Color: White", 1);
            return 1;
        } else if (colorValue > 600) {
            Serial.println("Detected color: Gray");
            displayOnLCD("Color: Gray", 1);
            return 2;
        } else if (colorValue > 300) {
            Serial.println("Detected color: Black");
            displayOnLCD("Color: Black", 1);
            return 3;
        } else {
            Serial.println("Detected color: Blue");
            displayOnLCD("Color: Blue", 1);
            return 4;
        }
    }

    void displayOnLCD(String message, int row) {
        Definitions::lcd.setCursor(0, row);
        Definitions::lcd.print("                ");
        Definitions::lcd.setCursor(0, row);
        Definitions::lcd.print(message);
    }
};

// ----------------- Robot Control Class ------------------------
class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor, frontRightSensor, sideLeftSensor, sideRightSensor;
    float initialCompassDirection;
    unsigned long lastMoveTime;
    const unsigned long MOVE_INTERVAL = Definitions::MOVE_TIME_MS;

public:
    Robot()
        : frontLeftSensor(Definitions::FRONT_LEFT_TRIG_PIN, Definitions::FRONT_LEFT_ECHO_PIN),
          frontRightSensor(Definitions::FRONT_RIGHT_TRIG_PIN, Definitions::FRONT_RIGHT_ECHO_PIN),
          sideLeftSensor(Definitions::SIDE_LEFT_TRIG_PIN, Definitions::SIDE_LEFT_ECHO_PIN),
          sideRightSensor(Definitions::SIDE_RIGHT_TRIG_PIN, Definitions::SIDE_RIGHT_ECHO_PIN) {
        initialCompassDirection = frontLeftSensor.getDirection();
    }

    void updateState() {
        if (millis() - lastMoveTime >= MOVE_INTERVAL) {
            moveForwardWithCorrection();
            checkCompassAfterMove();
            performTurn();
            checkCompassAfterTurn();
            
            int color = frontLeftSensor.detectColor();
            if (color == 3 || color == 4) {
                motors.moveBackward(1000);
            }

            lastMoveTime = millis();
        }
    }

    void moveForwardWithCorrection() {
        motors.moveForward();
        delay(Definitions::MOVE_TIME_MS);
        motors.stop();
    }

    void checkCompassAfterMove() {
        float currentDirection = frontLeftSensor.getDirection();
        float deviation = currentDirection - initialCompassDirection;

        if (deviation < -5) {
            motors.turnRight();
        } else if (deviation > 5) {
            motors.turnLeft();
        }
    }

    void performTurn() {
        float frontLeft = frontLeftSensor.getDistance();
        float frontRight = frontRightSensor.getDistance();
        float sideLeft = sideLeftSensor.getDistance();
        float sideRight = sideRightSensor.getDistance();

        // New turn logic
        if (frontLeft < 10.0 && frontRight < 10.0 && sideRight < 10.0) {
            motors.turnLeft();
        } 
        else if (frontLeft < 10.0 && frontRight < 10.0 && sideLeft < 10.0) {
            motors.turnRight();
        }
        // Original fallback logic
        else if (frontLeft < 10.0 && frontRight < 10.0) {
            if (frontLeft < frontRight) {
                motors.turnRight();
            } else {
                motors.turnLeft();
            }
        }
    }

    void checkCompassAfterTurn() {
        float currentDirection = frontLeftSensor.getDirection();
        float expectedDirection = initialCompassDirection + 90;

        if (abs(currentDirection - expectedDirection) > 5) {
            if (currentDirection < expectedDirection) {
                motors.turnRight();
            } else {
                motors.turnLeft();
            }
        }
    }
};

// ------------------ Main Control ---------------------
Robot robot;

void setup() {
    Serial.begin(9600);
    Definitions::pinouts();
}

void loop() {
    robot.updateState();
    delay(100);
}
