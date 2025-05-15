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

    static const int COLOR_SENSOR_PIN5 = 2;

    // LED Pins
    static const int LED_STARTUP = 61;
    static const int LED_MOVING = 60;
    static const int LED_STOPPED = 59;

    static const int STOP_POSITION = 90;
    static const int SPEED = 30;

    static constexpr float FRONT_TRIGGER_DISTANCE = 10.0;
    static constexpr float STEER_FAR = 8.0;
    static constexpr float STEER_CLOSE = 3.0;

    static QMC5883LCompass compass;

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

        pinMode(LED_STARTUP, OUTPUT);
        pinMode(LED_MOVING, OUTPUT);
        pinMode(LED_STOPPED, OUTPUT);

        Wire.begin();
        compass.init();
        compass.setCalibration(-240, 570, -690, 590, -320, 570);
    }
};

// Initialize static compass object
QMC5883LCompass Definitions::compass;

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
        return (duration == 0) ? -1 : (duration * 0.0343) / 2;
    }
};

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
};

// ----------------- Robot Control Class ------------------------
class Robot {
private:
    Motors motors;
    Sensors frontLeftSensor, frontRightSensor, sideRightSensor, sideLeftSensor;
    bool isMoving;

public:
    Robot()
        : motors(),
          frontLeftSensor(67, 66),
          frontRightSensor(16, 17),
          sideRightSensor(8, 7),
          sideLeftSensor(65, 64),
          isMoving(false) {}

    void updateLEDs() {
        digitalWrite(Definitions::LED_MOVING, isMoving ? HIGH : LOW);
        digitalWrite(Definitions::LED_STOPPED, isMoving ? LOW : HIGH);
    }

    void updateState() {
        float leftFrontDistance = frontLeftSensor.getDistance();
        float rightFrontDistance = frontRightSensor.getDistance();
        float leftSideDistance = sideLeftSensor.getDistance();
        float rightSideDistance = sideRightSensor.getDistance();

        Serial.println("----- SENSOR READINGS -----");
        Serial.print("Front Left: "); Serial.print(leftFrontDistance); Serial.println(" cm");
        Serial.print("Front Right: "); Serial.print(rightFrontDistance); Serial.println(" cm");
        Serial.print("Side Left: "); Serial.print(leftSideDistance); Serial.println(" cm");
        Serial.print("Side Right: "); Serial.print(rightSideDistance); Serial.println(" cm");

        // Turns
        if (rightSideDistance < Definitions::FRONT_TRIGGER_DISTANCE && leftFrontDistance < Definitions::FRONT_TRIGGER_DISTANCE && rightFrontDistance < Definitions::FRONT_TRIGGER_DISTANCE) {
            motors.turnLeft();
        } else if (leftSideDistance < Definitions::FRONT_TRIGGER_DISTANCE && leftFrontDistance < Definitions::FRONT_TRIGGER_DISTANCE && rightFrontDistance < Definitions::FRONT_TRIGGER_DISTANCE) {
            motors.turnRight();
        }
        // Steers
        else if (rightSideDistance > Definitions::STEER_CLOSE && rightSideDistance < Definitions::STEER_FAR) {
            motors.steerRight();
        } else if (rightSideDistance < Definitions::STEER_CLOSE) {
            motors.steerLeft();
        } else if (leftSideDistance > Definitions::STEER_CLOSE && leftSideDistance < Definitions::STEER_FAR) {
            motors.steerLeft();
        } else if (leftSideDistance < Definitions::STEER_CLOSE) {
            motors.steerRight();
        }
    }

    void move() {
        isMoving = true;
        motors.moveForward();
        updateLEDs();
    }

    void stop() {
        isMoving = false;
        motors.stop();
        updateLEDs();
    }
};

// ------------------ Main Control ---------------------
Robot robot;

void setup() {
    Serial.begin(9600);
    Definitions::pinouts();

    digitalWrite(Definitions::LED_STARTUP, HIGH);
    delay(3000);
    digitalWrite(Definitions::LED_STARTUP, LOW);
}

void loop() {
    robot.move();
    delay(4000);
    robot.updateState();
    robot.stop();
    delay(2000);
    robot.updateState();
}
