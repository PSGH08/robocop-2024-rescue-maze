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

    static const int COLOR_SENSOR_PIN1 = 58;
    static const int COLOR_SENSOR_PIN2 = 57;
    static const int COLOR_SENSOR_PIN3 = 4;
    static const int COLOR_SENSOR_PIN4 = 5;
    static const int COLOR_SENSOR_PIN5 = 2;

    // LED Pins
    static const int LED_STARTUP = 61;
    static const int LED_MOVING = 60;
    static const int LED_STOPPED = 59;

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

        pinMode(COLOR_SENSOR_PIN1, OUTPUT);
        pinMode(COLOR_SENSOR_PIN2, OUTPUT);
        pinMode(COLOR_SENSOR_PIN3, OUTPUT);
        pinMode(COLOR_SENSOR_PIN4, OUTPUT);
        pinMode(COLOR_SENSOR_PIN5, INPUT);

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

    float getDirection() {
        Definitions::compass.read();
        int compassX = Definitions::compass.getX();
        int compassY = Definitions::compass.getY();

        if (compassX == 0 && compassY == 0) {
            Serial.println("Compass error or interference!");
            return -1;
        }

        float compassDirection = atan2(compassY, compassX) * (180.0 / PI);
        if (compassDirection < 0) compassDirection += 360;
        return compassDirection;
    }

    int getColorData() {
        return analogRead(Definitions::COLOR_SENSOR_PIN5);
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
        if (isMoving) {
            Serial.println("Robot Moving: Printing Ultrasonic Sensor Data...");
            Serial.print("Front Left Distance: "); Serial.print(frontLeftSensor.getDistance()); Serial.println(" cm");
            Serial.print("Front Right Distance: "); Serial.print(frontRightSensor.getDistance()); Serial.println(" cm");
            Serial.print("Side Left Distance: "); Serial.print(sideLeftSensor.getDistance()); Serial.println(" cm");
            Serial.print("Side Right Distance: "); Serial.print(sideRightSensor.getDistance()); Serial.println(" cm");
        } else {
            Serial.println("Robot Stopped: Printing Sensor Data...");
            Serial.print("Compass Direction: "); Serial.println(frontLeftSensor.getDirection());
            Serial.print("Color Sensor Reading: "); Serial.println(frontLeftSensor.getColorData());
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
