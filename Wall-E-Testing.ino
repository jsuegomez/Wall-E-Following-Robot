#include <NewPing.h>

// Ultrasonic sensor pins
#define ULTRASONIC_SENSOR_TRIG 8
#define ULTRASONIC_SENSOR_ECHO 9

// Logic constants (kept for reference, even though we don't actually drive motors)
#define MAX_FORWARD_MOTOR_SPEED 75
#define MAX_MOTOR_TURN_SPEED_ADJUSTMENT 50

// Distance window (cm) where the robot would follow
#define MIN_DISTANCE 10
#define MAX_DISTANCE 30

// IR sensor pins
#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 10

// Max distance for NewPing (in cm)
#define MAX_ULTRASONIC_DISTANCE 200

NewPing mySensor(ULTRASONIC_SENSOR_TRIG,
                 ULTRASONIC_SENSOR_ECHO,
                 MAX_ULTRASONIC_DISTANCE);

// Helper: print what the robot *would* be doing
void displayAction(const char* action,
                   int distance,
                   int leftIR,
                   int rightIR)
{
  Serial.print("Action: ");
  Serial.print(action);

  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.print(" cm");

  Serial.print(" | IR Left: ");
  Serial.print(leftIR == LOW ? "DETECTED" : "NONE");

  Serial.print(" | IR Right: ");
  Serial.println(rightIR == LOW ? "DETECTED" : "NONE");
}

void setup()
{
  // Serial output for testing
  Serial.begin(9600);

  // IR sensors as inputs
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // NOTE: Motor pins are intentionally NOT configured in this test version.
  // This completely disables any motor motion.

  Serial.println("Wall-E FOLLOWER - TEST MODE (motors disabled)");
}

void loop()
{
  // Read sensors
  int distance = mySensor.ping_cm();  // distance in cm (0 if out of range)
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue  = digitalRead(IR_SENSOR_LEFT);

  // NOTE: If IR sensor detects the hand then its value will be LOW,
  // otherwise the value will be HIGH.

  // Decide action based on sensor readings (same logic as original code)
  if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
  {
    // Right IR detects hand -> robot would turn right
    displayAction("TURN RIGHT", distance, leftIRSensorValue, rightIRSensorValue);
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
  {
    // Left IR detects hand -> robot would turn left
    displayAction("TURN LEFT", distance, leftIRSensorValue, rightIRSensorValue);
  }
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
  {
    // Hand within following distance window -> robot would go forward
    displayAction("MOVE FORWARD", distance, leftIRSensorValue, rightIRSensorValue);
  }
  else
  {
    // No hand in the right place / no IR detection -> robot would stop
    displayAction("STOP", distance, leftIRSensorValue, rightIRSensorValue);
  }

  // Small delay to keep the serial output readable
  delay(100);
}
