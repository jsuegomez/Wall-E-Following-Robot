#include <NewPing.h>

#define ULTRASONIC_SENSOR_TRIG 5
#define ULTRASONIC_SENSOR_ECHO 4

#define MAX_FORWARD_MOTOR_SPEED 75
#define MAX_MOTOR_TURN_SPEED_ADJUSTMENT 50

#define MIN_DISTANCE 10
#define MAX_DISTANCE 30

#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 10

// Right motor: connect to driver IN1 / IN2
int rightMotorPin1 = 7;
int rightMotorPin2 = 6;

// Left motor: connect to driver IN3 / IN4
int leftMotorPin1 = 3;
int leftMotorPin2 = 2;

// Ultrasonic sensor (max distance 400 cm)
NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);

void setup()
{
  // Motor pins as outputs
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1,  OUTPUT);
  pinMode(leftMotorPin2,  OUTPUT);

  // IR sensors as inputs
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT,  INPUT);

  // Make sure motors are stopped at startup
  stopMotors();
}

void loop()
{
  int distance          = mySensor.ping_cm();
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue  = digitalRead(IR_SENSOR_LEFT);

  // NOTE: If IR sensor detects the hand its value will be LOW,
  // otherwise the value will be HIGH.

  // If right sensor detects hand, then turn right
  if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
  {
    turnRight();
  }
  // If left sensor detects hand, then turn left
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
  {
    turnLeft();
  }
  // If distance is between min and max then go straight
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
  {
    moveForward();
  }
  // Otherwise stop the motors
  else
  {
    stopMotors();
  }
}

// ===== Motor helper functions (no enable pins) =====

// Both motors forward
void moveForward()
{
  // Right motor forward
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);

  // Left motor forward
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
}

// Turn right: left motor forward, right motor stopped
void turnRight()
{
  // Right motor stopped
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);

  // Left motor forward
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
}

// Turn left: right motor forward, left motor stopped
void turnLeft()
{
  // Left motor stopped
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);

  // Right motor forward
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

// Stop both motors
void stopMotors()
{
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
}
