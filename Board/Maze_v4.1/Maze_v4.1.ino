#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <QMC5883LCompass.h>

// ----------------- Definitions Class ------------------------
class Definitions {
public:
    // Motor pins
    static const int LEFT_MOTOR_PIN = 13;
    static const int RIGHT_MOTOR_PIN = 10;
    static const int MEDKIT_PIN = 54;

    // Ultrasonic sensor pins
    static const int FRONT_LEFT_TRIG_PIN = 67;
    static const int FRONT_LEFT_ECHO_PIN = 66;
    static const int FRONT_RIGHT_TRIG_PIN = 16;
    static const int FRONT_RIGHT_ECHO_PIN = 17;
    static const int SIDE_LEFT_TRIG_PIN = 65;
    static const int SIDE_LEFT_ECHO_PIN = 64;
    static const int SIDE_RIGHT_TRIG_PIN = 8;
    static const int SIDE_RIGHT_ECHO_PIN = 7;

    // Color sensor pins
    static const int COLOR_SENSOR_PIN = A0;  // Analog pin for color sensor
    static const int COLOR_SENSOR_PIN1 = 58;
    static const int COLOR_SENSOR_PIN2 = 57;
    static const int COLOR_SENSOR_PIN3 = 4;
    static const int COLOR_SENSOR_PIN4 = 5;
    static const int COLOR_SENSOR_PIN5 = 2;

    // LED pins
    static const int LED_STARTUP = 61;
    static const int LED_MOVING = 60;
    static const int LED_STOPPED = 59;

    // LCD settings
    static const int LCD_ADDRESS = 0x27;
    static const int LCD_COLUMNS = 16;
    static const int LCD_ROWS = 2;

    // Motor control
    static const int STOP_POSITION = 90;
    static const int SPEED = 40;
    static const unsigned long MOVE_TIME_MS = 8180;
    static const int MOVE_DURATION = 5000;
    static const int STOP_DURATION = 2500;
    static const int TURN_DURATION = 3000;
    static const int THROW_DURATION = 500;

    // Distance thresholds
    static constexpr float MIN_DISTANCE = 6;
    static constexpr float MAX_DISTANCE = 8;
    static constexpr float IGNORE_DISTANCE = 7;
    static constexpr float FRONT_TRIGGER_DISTANCE = 8;

    // Sensor objects
    static LiquidCrystal_I2C lcd;

    static void pinouts() {
        // Motor pins
        pinMode(LEFT_MOTOR_PIN, OUTPUT);
        pinMode(RIGHT_MOTOR_PIN, OUTPUT);
        pinMode(MEDKIT_PIN, OUTPUT);

        // Ultrasonic sensors
        pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
        pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
        pinMode(SIDE_LEFT_TRIG_PIN, OUTPUT);
        pinMode(SIDE_LEFT_ECHO_PIN, INPUT);
        pinMode(SIDE_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(SIDE_RIGHT_ECHO_PIN, INPUT);

        // Color sensors
        pinMode(COLOR_SENSOR_PIN, INPUT);
        pinMode(COLOR_SENSOR_PIN1, OUTPUT);
        pinMode(COLOR_SENSOR_PIN2, OUTPUT);
        pinMode(COLOR_SENSOR_PIN3, OUTPUT);
        pinMode(COLOR_SENSOR_PIN4, OUTPUT);
        pinMode(COLOR_SENSOR_PIN5, INPUT);

        // LED pins
        pinMode(LED_STARTUP, OUTPUT);
        pinMode(LED_MOVING, OUTPUT);
        pinMode(LED_STOPPED, OUTPUT);

        // Initialize LCD
        lcd.begin(LCD_COLUMNS, LCD_ROWS);
        lcd.backlight();
    }
};

// Initialize static objects
QMC5883LCompass compass;
LiquidCrystal_I2C Definitions::lcd(Definitions::LCD_ADDRESS, Definitions::LCD_COLUMNS, Definitions::LCD_ROWS);

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
        float distance = (duration == 0) ? Definitions::IGNORE_DISTANCE : (duration * 0.0343) / 2;
        
        // Constrain distance to valid range
        distance = constrain(distance, Definitions::MIN_DISTANCE, Definitions::MAX_DISTANCE);
        
        Serial.print("Ultrasonic Distance: ");
        Serial.println(distance);
        displayOnLCD("Distance: " + String(distance) + " cm", 1);
        return distance;
    }

    int getHeading(){
            int x, y;
            float heading;
          
            // Read compass values
            compass.read();
            x = compass.getX();
            y = compass.getY();
          
            // Check if readings are valid
            if (x == 0 && y == 0) {
              Serial.println("Sensor error or interference!");
              return; // Skip this loop iteration
            }
          
            // Calculate heading in degrees
            heading = atan2(y, x) * (180.0 / PI);
          
            // Ensure heading is between 0 and 360
            if (heading < 0) {
              heading == 360;
            }
          
              heading += 5;
          
            // Ensure the heading stays within 0 and 360 after correction
            heading = fmod(heading + 360, 360); // Ensure range is 0 to 360
          
            // Print the adjusted heading
            Serial.print("Heading: ");
            Serial.print(heading);
            Serial.println("°");
          
            delay(300);
    }

    int detectColor() {
        // Using analog color sensor
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

    void displayOnLCD(String message, int row) {
        Definitions::lcd.setCursor(0, row);
        Definitions::lcd.print("                "); // Clear the row
        Definitions::lcd.setCursor(0, row);
        Definitions::lcd.print(message);
    }
};

// ----------------- Motors Class ------------------------
class Motors {
private:
    Servo leftMotor, rightMotor;
    int motorPin;

public:
    Motors() {
        leftMotor.attach(Definitions::LEFT_MOTOR_PIN);
        rightMotor.attach(Definitions::RIGHT_MOTOR_PIN);
        motorPin = Definitions::MEDKIT_PIN;
        pinMode(motorPin, OUTPUT);
    }

    void moveForward() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        Serial.println("Moving forward...");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Moving Forward ");
    }

    void stop() {
        leftMotor.write(Definitions::STOP_POSITION);
        rightMotor.write(Definitions::STOP_POSITION);
        Serial.println("Stopped.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Stopped       ");
    }

    void steerLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(200);
        stop();
        Serial.println("Steered left.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Steering Left ");
    }

    void steerRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(200);
        stop();
        Serial.println("Steered right.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Steering Right");
    }

    void turnLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(Definitions::TURN_DURATION);
        stop();
        Serial.println("Turned left.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Turned Left   ");
    }

    void turnRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(Definitions::TURN_DURATION);
        stop();
        Serial.println("Turned right.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Turned Right  ");
    }

    void moveBackward(int duration) {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(duration);
        stop();
        Serial.println("Moved backward.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Moving Back   ");
    }

    void throwMedkit() {
        digitalWrite(motorPin, HIGH);
        delay(Definitions::THROW_DURATION);
        digitalWrite(motorPin, LOW);
        Serial.println("Medkit thrown.");
        Definitions::lcd.setCursor(0, 0);
        Definitions::lcd.print("Medkit Thrown ");
    }
};


// ----------------- Robot Control Class ------------------------
class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor, frontRightSensor, sideLeftSensor, sideRightSensor;
    unsigned long lastMoveTime;
    const unsigned long MOVE_INTERVAL = Definitions::MOVE_DURATION;
    bool isMoving;

public:
    Robot()
        : frontLeftSensor(Definitions::FRONT_LEFT_TRIG_PIN, Definitions::FRONT_LEFT_ECHO_PIN),
          frontRightSensor(Definitions::FRONT_RIGHT_TRIG_PIN, Definitions::FRONT_RIGHT_ECHO_PIN),
          sideLeftSensor(Definitions::SIDE_LEFT_TRIG_PIN, Definitions::SIDE_LEFT_ECHO_PIN),
          sideRightSensor(Definitions::SIDE_RIGHT_TRIG_PIN, Definitions::SIDE_RIGHT_ECHO_PIN),
          isMoving(false) {
        lastMoveTime = millis();
    }

    void updateLEDs() {
        digitalWrite(Definitions::LED_MOVING, isMoving ? HIGH : LOW);
        digitalWrite(Definitions::LED_STOPPED, isMoving ? LOW : HIGH);
    }

    void updateState() {
        if (millis() - lastMoveTime >= MOVE_INTERVAL) {
            // Get sensor readings
            float leftFront = frontLeftSensor.getDistance();
            float rightFront = frontRightSensor.getDistance();
            float leftSide = sideLeftSensor.getDistance();
            float rightSide = sideRightSensor.getDistance();
            int color = frontLeftSensor.detectColor();

            // Decision making
            if (leftFront < 8.0 && rightFront < 8.0 && rightSide < 8.0) {
                motors.turnLeft();
                isMoving = false;
            } 
            else if (leftFront < 8.0 && rightFront < 8.0 && leftSide < 8.0) {
                motors.turnRight();
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

            // Special cases
            if (color == 3 || color == 4) { // Black or Blue
                motors.moveBackward(1000);
                isMoving = true;
            }

            // Update LEDs based on movement state
            updateLEDs();

            // Print sensor data
            Serial.println("Current Sensor Readings:");
            Serial.print("Front Left: "); Serial.print(leftFront); Serial.println(" cm");
            Serial.print("Front Right: "); Serial.print(rightFront); Serial.println(" cm");
            Serial.print("Side Left: "); Serial.print(leftSide); Serial.println(" cm");
            Serial.print("Side Right: "); Serial.print(rightSide); Serial.println(" cm");
            Serial.print("Color: "); Serial.println(color);

            lastMoveTime = millis();
        }
    }

    void throwMedkit() {
        motors.throwMedkit();
    }
};

// ------------------ Main Control ---------------------
Robot robot;

void setup() {
    Serial.begin(9600);
    Definitions::pinouts();
    compass.init();

    // Startup sequence
    digitalWrite(Definitions::LED_STARTUP, HIGH);
    Definitions::lcd.setCursor(0, 0);
    Definitions::lcd.print("Initializing...");
    delay(3000);
    digitalWrite(Definitions::LED_STARTUP, LOW);
    Definitions::lcd.clear();
    Definitions::lcd.print("Robot Ready");
    Serial.println("Robot initialization complete");
    compass.setCalibration(-240, 570, -690, 590, -320, 570);
}

void loop() {
    robot.updateState();
    delay(100);
}
