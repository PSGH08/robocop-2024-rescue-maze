#include <Wire.h>
#include <Servo.h>
#include <QMC5883LCompass.h>

// ----------------- Definitions Class ------------------------
class Definitions {
public:
    static const int LEFT_MOTOR_PIN = 13;
    static const int RIGHT_MOTOR_PIN = 10;

    static const int COMPASS_SDA_PIN = 43;
    static const int COMPASS_SCL_PIN = 32;

    static const int STOP_POSITION = 90;
    static const int SPEED = 60;
    static const int WHEEL_DIAMETER_CM = 7;
    static const int TARGET_DISTANCE_CM = 30;

    // Calculate move time for 30 cm (Pre-calculated as ~8.18 seconds)
    static const unsigned long MOVE_TIME_MS = 8180;

    static QMC5883LCompass compass;

    static void pinouts() {
        pinMode(LEFT_MOTOR_PIN, OUTPUT);
        pinMode(RIGHT_MOTOR_PIN, OUTPUT);

        Wire.begin();
        compass.init();
        compass.setCalibration(-240, 570, -690, 590, -320, 570);
    }
};

QMC5883LCompass Definitions::compass;

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

    void turnLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(1000);
        stop();
    }

    void turnRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        delay(1000);
        stop();
    }

    void steerLeft() {
        leftMotor.write(Definitions::STOP_POSITION - Definitions::SPEED / 2);
        rightMotor.write(Definitions::STOP_POSITION - Definitions::SPEED);
        delay(200);
        stop();
    }

    void steerRight() {
        leftMotor.write(Definitions::STOP_POSITION + Definitions::SPEED);
        rightMotor.write(Definitions::STOP_POSITION + Definitions::SPEED / 2);
        delay(200);
        stop();
    }
};

// ----------------- Robot Control Class ------------------------
class Robot {
private:
    Motors motors;
    float initialCompassX, initialCompassY;
    int movementSequence[5] = {0, 1, 0, 2, 0};
    int sequenceIndex = 0;

public:
    Robot() {
        initialCompassX = Definitions::compass.getX();
        initialCompassY = Definitions::compass.getY();
    }

    void updateState() {
        if (sequenceIndex < 5) {
            int movement = movementSequence[sequenceIndex];
            switch (movement) {
                case 0:
                    moveForwardWithCompassCorrection();
                    break;
                case 1:
                    motors.turnRight();
                    adjustDirection();
                    break;
                case 2:
                    motors.turnLeft();
                    adjustDirection();
                    break;
            }
            sequenceIndex++;
        }
    }

    void moveForwardWithCompassCorrection() {
        motors.moveForward();
        delay(Definitions::MOVE_TIME_MS);
        motors.stop();
        adjustDirection();
    }

    void adjustDirection() {
        float currentCompassX = Definitions::compass.getX();
        float currentCompassY = Definitions::compass.getY();

        float initialAngle = atan2(initialCompassY, initialCompassX) * (180.0 / PI);
        float currentAngle = atan2(currentCompassY, currentCompassX) * (180.0 / PI);

        if (currentAngle < initialAngle - 5) {
            motors.steerRight();
        } else if (currentAngle > initialAngle + 5) {
            motors.steerLeft();
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
