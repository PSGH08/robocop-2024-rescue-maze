#include <Wire.h>
#include <Servo.h>
#include <QMC5883LCompass.h>
#include <LiquidCrystal_I2C.h> // Include the LCD library

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

    // Color sensor pin
    static const int COLOR_SENSOR_PIN = A0; // Analog pin for color sensor

    // Compass pins
    static const int COMPASS_SDA_PIN = 43;
    static const int COMPASS_SCL_PIN = 32;

    // LCD pins (I2C address and dimensions)
    static const int LCD_ADDRESS = 0x27; // Default I2C address for QAPASS LCD
    static const int LCD_COLUMNS = 16;
    static const int LCD_ROWS = 2;

    // Motor control
    static const int STOP_POSITION = 90;
    static const int SPEED = 60; // Speed for MG995 servo motors
    static const int WHEEL_DIAMETER_CM = 7; // Wheel diameter in cm
    static const int TARGET_DISTANCE_CM = 30; // Target distance to move (30 cm)

    // Calculate time to move 30 cm (Pre-calculated as ~8.18 seconds)
    static const unsigned long MOVE_TIME_MS = 8180;

    // Compass object
    static QMC5883LCompass compass;

    // LCD object
    static LiquidCrystal_I2C lcd;

    // Initialize pin configurations and sensors
    static void pinouts() {
        pinMode(LEFT_MOTOR_PIN, OUTPUT);
        pinMode(RIGHT_MOTOR_PIN, OUTPUT);

        pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
        pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);

        pinMode(COLOR_SENSOR_PIN, INPUT);

        Wire.begin();  // Initialize I2C for the compass sensor
        compass.init();  // Initialize the compass sensor
        compass.setCalibration(-240, 570, -690, 590, -320, 570);  // Optional calibration

        lcd.begin(LCD_COLUMNS, LCD_ROWS); // Initialize the LCD
        lcd.backlight(); // Turn on the backlight
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
        // Sharp left turn
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(1000); // Turn for 1 second (adjust as needed)
        stop();
        Serial.println("Turned left.");
    }

    void turnRight() {
        // Sharp right turn
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(1000); // Turn for 1 second (adjust as needed)
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
    int trigPin, echoPin;  // Trigger and Echo pins for ultrasonic sensors

public:
    // Constructor for ultrasonic sensors
    Sensors(int trig, int echo) : trigPin(trig), echoPin(echo) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    // Get distance from ultrasonic sensor
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

    // Get compass direction
    float getDirection() {
        int compassX = Definitions::compass.getX();  // Get X-axis reading
        int compassY = Definitions::compass.getY();  // Get Y-axis reading

        if (compassX == 0 && compassY == 0) {
            Serial.println("Sensor error or interference!");
            displayOnLCD("Compass Error!", 0);
            return -1;  // Error case, invalid reading
        }

        float compassDirection = atan2(compassY, compassX) * (180.0 / PI);
        if (compassDirection < 0) compassDirection += 360;
        Serial.print("Compass Direction: ");
        Serial.println(compassDirection);
        displayOnLCD("Direction: " + String(compassDirection) + "°", 0);
        return compassDirection;  // Return direction in 0-360 degrees
    }

    // Detect color (simulated logic)
    int detectColor() {
        int colorValue = analogRead(Definitions::COLOR_SENSOR_PIN);

        if (colorValue > 900) {
            Serial.println("Detected color: White");
            displayOnLCD("Color: White", 1);
            return 1; // White
        } else if (colorValue > 600) {
            Serial.println("Detected color: Gray");
            displayOnLCD("Color: Gray", 1);
            return 2; // Gray
        } else if (colorValue > 300) {
            Serial.println("Detected color: Black");
            displayOnLCD("Color: Black", 1);
            return 3; // Black
        } else {
            Serial.println("Detected color: Blue");
            displayOnLCD("Color: Blue", 1);
            return 4; // Blue
        }
    }

    // Display a message on the LCD
    void displayOnLCD(String message, int row) {
        Definitions::lcd.setCursor(0, row);
        Definitions::lcd.print("                "); // Clear the row
        Definitions::lcd.setCursor(0, row);
        Definitions::lcd.print(message);
    }
};

// ----------------- Robot Control Class ------------------------
class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor, frontRightSensor;
    float initialCompassDirection; // Initial compass heading
    unsigned long lastMoveTime;
    const unsigned long MOVE_INTERVAL = Definitions::MOVE_TIME_MS;

public:
    Robot()
        : frontLeftSensor(Definitions::FRONT_LEFT_TRIG_PIN, Definitions::FRONT_LEFT_ECHO_PIN),
          frontRightSensor(Definitions::FRONT_RIGHT_TRIG_PIN, Definitions::FRONT_RIGHT_ECHO_PIN) {
        // Set initial compass direction
        initialCompassDirection = frontLeftSensor.getDirection();
    }

    void updateState() {
        if (millis() - lastMoveTime >= MOVE_INTERVAL) {
            // Move forward for 30 cm
            moveForwardWithCorrection();

            // Check compass after moving
            checkCompassAfterMove();

            // Perform turn based on ultrasonic sensors
            performTurn();

            // Check compass after turning
            checkCompassAfterTurn();

            // Detect color
            int color = frontLeftSensor.detectColor();
            if (color == 3 || color == 4) { // Black or Blue
                motors.moveBackward(1000); // Move backward for 1 second
            }

            lastMoveTime = millis();
        }
    }

    void moveForwardWithCorrection() {
        motors.moveForward();
        delay(Definitions::MOVE_TIME_MS); // Move for 30 cm
        motors.stop();
    }

    void checkCompassAfterMove() {
        float currentDirection = frontLeftSensor.getDirection();
        float deviation = currentDirection - initialCompassDirection;

        // Adjust direction if deviation is more than 5 degrees
        if (deviation < -5) {
            motors.turnRight();
        } else if (deviation > 5) {
            motors.turnLeft();
        }
    }

    void performTurn() {
        float leftDistance = frontLeftSensor.getDistance();
        float rightDistance = frontRightSensor.getDistance();

        // Turn logic based on ultrasonic sensors
        if (leftDistance < 10.0 && rightDistance < 10.0) {
            if (leftDistance < rightDistance) {
                motors.turnRight();
            } else {
                motors.turnLeft();
            }
        }
    }

    void checkCompassAfterTurn() {
        float currentDirection = frontLeftSensor.getDirection();
        float expectedDirection = initialCompassDirection + 90; // For right turn

        // Adjust direction if not turned 90 degrees
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
    delay(100); // Small delay to avoid overloading the loop
}
