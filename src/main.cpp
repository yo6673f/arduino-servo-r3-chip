// Light-seeking servo for the Elegoo UNO R3.
//
// Two photoresistors (light sensors) are read on A0 and A1. The servo turns
// a little at a time toward whichever sensor sees more light, and stops when
// both sides read about the same.
//
// Wiring (each photoresistor is half of a voltage divider):
//
//   5V ──[photoresistor]──┬── A0 (left)      5V ──[photoresistor]──┬── A1 (right)
//                         │                                        │
//                      [10kΩ]                                   [10kΩ]
//                         │                                        │
//                        GND                                      GND
//
//   Servo: brown/black -> GND, red -> 5V, orange/yellow (signal) -> pin 9
//
// With this wiring, MORE light = HIGHER reading (0-1023).

#include <Arduino.h>
#include <Servo.h>

const int LEFT_SENSOR_PIN = A0;
const int RIGHT_SENSOR_PIN = A1;
const int SERVO_PIN = 9;

// How different the two readings must be before the servo moves.
// Raise it if the servo jitters back and forth; lower it for more sensitivity.
const int DEADBAND = 20;

// Degrees moved per step, and the pause between steps (controls speed).
const int STEP_DEGREES = 1;
const unsigned long STEP_DELAY_MS = 15;

// Keep the servo inside a safe range so it doesn't strain against its stops.
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

// If the servo turns AWAY from the light, flip this to true
// (or swap the two sensors' wires).
const bool REVERSE_DIRECTION = false;

Servo servo;
int angle = 90;

// Average a few readings to smooth out noise.
int readLight(int pin) {
  long total = 0;
  for (int i = 0; i < 4; i++) {
    total += analogRead(pin);
  }
  return total / 4;
}

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  servo.write(angle);  // start centered
  delay(500);
}

void loop() {
  int left = readLight(LEFT_SENSOR_PIN);
  int right = readLight(RIGHT_SENSOR_PIN);
  int difference = left - right;  // positive = left side is brighter

  if (abs(difference) > DEADBAND) {
    int direction = (difference > 0) ? STEP_DEGREES : -STEP_DEGREES;
    if (REVERSE_DIRECTION) {
      direction = -direction;
    }
    angle = constrain(angle + direction, MIN_ANGLE, MAX_ANGLE);
    servo.write(angle);
  }

  Serial.print("Left: ");
  Serial.print(left);
  Serial.print("  Right: ");
  Serial.print(right);
  Serial.print("  Diff: ");
  Serial.print(difference);
  Serial.print("  Angle: ");
  Serial.println(angle);

  delay(STEP_DELAY_MS);
}
