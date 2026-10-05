// Single-axis solar tracker for the Elegoo UNO R3.
//
// A small solar panel is mounted on a servo. Two photoresistors (light
// sensors) are mounted ON THE PANEL, one on each side, with a small wall
// (shade divider) between them. The servo turns the panel a little at a time
// toward whichever sensor sees more light. When the panel faces the light
// both sensors read about the same and the panel holds still.
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

// Hysteresis: the panel starts moving when the sensors differ by more than
// START_DEADBAND, and keeps going until they differ by less than
// STOP_DEADBAND. This lets it settle squarely facing the light without
// jittering back and forth. Raise both if it still hunts.
const int START_DEADBAND = 30;
const int STOP_DEADBAND = 10;

// Degrees moved per step, and the pause between steps. The panel has some
// weight, so it moves slower than a bare servo arm to avoid jerking.
const int STEP_DEGREES = 1;
const unsigned long STEP_DELAY_MS = 30;

// Keep the panel away from the servo's hard stops (and from hitting the base
// or its own wires). Adjust once the panel is mounted.
const int MIN_ANGLE = 10;
const int MAX_ANGLE = 170;
const int START_ANGLE = 90;

// When the average light level drops below DARK_THRESHOLD (night, or the
// panel is covered) tracking pauses. It resumes once the light rises above
// LIGHT_THRESHOLD. The gap between them stops flickering at dusk.
// Check the Serial Monitor "Avg" value in the dark and in daylight to tune.
const int DARK_THRESHOLD = 50;
const int LIGHT_THRESHOLD = 80;

// Turn off the servo signal after the panel has been still this long. This
// stops the servo buzzing and saves power. A small panel is light enough
// that the servo's gears hold it in place. Set to false if the panel drifts
// (for example in wind).
const bool DETACH_WHEN_IDLE = true;
const unsigned long IDLE_DETACH_MS = 1000;

// If the panel turns AWAY from the light, flip this to true
// (or swap the two sensors' wires).
const bool REVERSE_DIRECTION = false;

// How often to print readings to the Serial Monitor.
const unsigned long PRINT_INTERVAL_MS = 250;

Servo servo;
int angle = START_ANGLE;
bool tracking = false;  // currently moving toward the light
bool isDark = false;
unsigned long lastMoveTime = 0;
unsigned long lastPrintTime = 0;

// Average a few readings to smooth out noise.
int readLight(int pin) {
  long total = 0;
  for (int i = 0; i < 4; i++) {
    total += analogRead(pin);
  }
  return total / 4;
}

void moveServo(int newAngle) {
  angle = newAngle;
  servo.write(angle);  // set the position first so attach() doesn't twitch
  if (!servo.attached()) {
    servo.attach(SERVO_PIN);
  }
  lastMoveTime = millis();
}

void setup() {
  Serial.begin(9600);
  moveServo(START_ANGLE);
  delay(500);
}

void loop() {
  int left = readLight(LEFT_SENSOR_PIN);
  int right = readLight(RIGHT_SENSOR_PIN);
  int difference = left - right;  // positive = left side is brighter
  int average = (left + right) / 2;

  if (isDark && average > LIGHT_THRESHOLD) {
    isDark = false;
  } else if (!isDark && average < DARK_THRESHOLD) {
    isDark = true;
  }

  if (isDark) {
    tracking = false;
  } else if (!tracking && abs(difference) > START_DEADBAND) {
    tracking = true;
  } else if (tracking && abs(difference) < STOP_DEADBAND) {
    tracking = false;
  }

  if (tracking) {
    int direction = (difference > 0) ? STEP_DEGREES : -STEP_DEGREES;
    if (REVERSE_DIRECTION) {
      direction = -direction;
    }
    int newAngle = constrain(angle + direction, MIN_ANGLE, MAX_ANGLE);
    if (newAngle != angle) {
      moveServo(newAngle);
    }
  }

  if (DETACH_WHEN_IDLE && servo.attached() &&
      millis() - lastMoveTime > IDLE_DETACH_MS) {
    servo.detach();
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    Serial.print("Left: ");
    Serial.print(left);
    Serial.print("  Right: ");
    Serial.print(right);
    Serial.print("  Diff: ");
    Serial.print(difference);
    Serial.print("  Avg: ");
    Serial.print(average);
    Serial.print("  Angle: ");
    Serial.print(angle);
    Serial.println(isDark ? "  [dark - paused]"
                          : (tracking ? "  [tracking]" : "  [holding]"));
  }

  delay(STEP_DELAY_MS);
}
