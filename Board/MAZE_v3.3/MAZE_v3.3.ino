#include <Wire.h>
#include <Servo.h>
#include <QMC5883LCompass.h>

// ----------------- Definitions Class ------------------------
class Definitions {
public:
    static const int LEFT_MOTOR_PIN = 13;
    static const int RIGHT_MOTOR_PIN = 10;
    static const int MEDKIT_PIN = 54;

    static const int FRONT_LEFT_TRIG_PIN = 67;
    static const int FRONT_LEFT_ECHO_PIN = 66;
    static const int FRONT_RIGHT_TRIG_PIN = 16;
    static const int FRONT_RIGHT_ECHO_PIN = 17;
    static const int SIDE_LEFT_TRIG_PIN = 65;
    static const int SIDE_LEFT_ECHO_PIN = 64;
    static const int SIDE_RIGHT_TRIG_PIN = 8;
    static const int SIDE_RIGHT_ECHO_PIN = 7;

    static const int COLOR_SENSOR_PIN1 = 2;
    static const int COLOR_SENSOR_PIN2 = 3;
    static const int COLOR_SENSOR_PIN3 = 4;
    static const int COLOR_SENSOR_PIN4 = 5;
    static const int COLOR_SENSOR_PIN5 = 6;

    static const int COMPASS_SDA_PIN = 43;
    static const int COMPASS_SCL_PIN = 32;

    static const int STOP_POSITION = 90;
    static const int SPEED = 30;
    static const int MOVE_DURATION = 4000;
    static const int STOP_DURATION = 2000;
    static const int TURN_DURATION = 2700;
    static const int THROW_DURATION = 500;

    static constexpr float MIN_DISTANCE = 3.5;
    static constexpr float MAX_DISTANCE = 400.0;
    static constexpr float IGNORE_DISTANCE = 100.0;
    static constexpr float FRONT_TRIGGER_DISTANCE = 10.0;

    static QMC5883LCompass compass;  // Compass object

    // Initialize pin configurations and sensors
    static void pinouts() {
        pinMode(LEFT_MOTOR_PIN, OUTPUT);
        pinMode(RIGHT_MOTOR_PIN, OUTPUT);
        pinMode(MEDKIT_PIN, OUTPUT);

        pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
        pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
        pinMode(SIDE_LEFT_TRIG_PIN, OUTPUT);
        pinMode(SIDE_LEFT_ECHO_PIN, INPUT);
        pinMode(SIDE_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(SIDE_RIGHT_ECHO_PIN, INPUT);

        pinMode(COLOR_SENSOR_PIN1, OUTPUT);
        pinMode(COLOR_SENSOR_PIN2, OUTPUT);
        pinMode(COLOR_SENSOR_PIN3, OUTPUT);
        pinMode(COLOR_SENSOR_PIN4, OUTPUT);
        pinMode(COLOR_SENSOR_PIN5, INPUT);

        Wire.begin();  // Initialize I2C for the compass sensor
        compass.init();  // Initialize the compass sensor
        compass.setCalibration(-240, 570, -690, 590, -320, 570);  // Optional calibration
    }
};

// Initialize static compass object
QMC5883LCompass Definitions::compass;

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
        return (duration == 0) ? 10 : (duration * 0.0343) / 2;
    }

    // Get compass direction
    float getDirection() {
        int compassX = Definitions::compass.getX();  // Get X-axis reading
        int compassY = Definitions::compass.getY();  // Get Y-axis reading

        if (compassX == 0 && compassY == 0) {
            Serial.println("Sensor error or interference!");
            return -1;  // Error case, invalid reading
        }

        float compassDirection = atan2(compassY, compassX) * (180.0 / PI);
        if (compassDirection < 0) compassDirection += 360;
        return compassDirection;  // Return direction in 0-360 degrees
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
    }

    void stop() {
        leftMotor.write(Definitions::STOP_POSITION);
        rightMotor.write(Definitions::STOP_POSITION);
    }

    void steerLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(200);
        stop();
    }

    void steerRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(200);
        stop();
    }

    void turnLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(500);
        stop();
    }

    void turnRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(500);
        stop();
    }

    void throwMedkit() {
        digitalWrite(motorPin, HIGH);
        delay(Definitions::THROW_DURATION);
        digitalWrite(motorPin, LOW);
    }
};

// ----------------- Robot Control Class ------------------------
class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor, frontRightSensor, sideRightSensor, sideLeftSensor;
    unsigned long lastMoveTime;
    const unsigned long MOVE_INTERVAL = Definitions::MOVE_DURATION;

public:
    Robot()
        : motors(),
          frontLeftSensor(67, 66),
          frontRightSensor(16, 17),
          sideRightSensor(8, 7),
          sideLeftSensor(65, 64),
          lastMoveTime(0) {}

    void updateState() {
        if (millis() - lastMoveTime >= MOVE_INTERVAL) {
            float leftFrontSensorDistance = frontLeftSensor.getDistance();
            float rightFrontSensorDistance = frontRightSensor.getDistance();
            float compassDirection = frontLeftSensor.getDirection();  // Use getDirection()

            Serial.print("Compass Direction: ");
            Serial.println(compassDirection);

            if (leftFrontSensorDistance < 3.0 && rightFrontSensorDistance < 3.0) {
                if (leftFrontSensorDistance < rightFrontSensorDistance) {
                    motors.turnRight();
                } else {
                    motors.turnLeft();
                }
            } else {
                if (rightFrontSensorDistance > 6.0 && rightFrontSensorDistance < 8.0) {
                    motors.steerRight();
                } else if (leftFrontSensorDistance > 6.0 && leftFrontSensorDistance < 8.0) {
                    motors.steerLeft();
                }
            }

            lastMoveTime = millis();
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
