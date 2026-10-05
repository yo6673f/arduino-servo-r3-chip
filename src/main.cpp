// Two-axis solar tracker for the Elegoo UNO R3.
//
// The solar panel sits on a 3D-printed pan/tilt mount driven by two servos.
// A raised "+" shaped divider on the mount (it moves with the panel) splits
// the area into four quadrants, with one photoresistor (light sensor) in each:
//
//              top
//         ┌─────┃─────┐
//         │ TL  ┃  TR │
//         ━━━━━━╋━━━━━━   <- raised + walls
//         │ BL  ┃  BR │
//         └─────┃─────┘
//            bottom
//
// When the panel points straight at the light, all four sensors read about
// the same. When the light is off to one side, the walls shade the sensors
// on the far side. The code compares:
//   left  (TL + BL) vs right  (TR + BR) -> pan servo turns left/right
//   top   (TL + TR) vs bottom (BL + BR) -> tilt servo tilts up/down
// and moves each servo a little at a time until the sides balance.
//
// Wiring (each photoresistor is half of a voltage divider, all four the same):
//
//   5V ──[photoresistor]──┬── A0 (TL) / A1 (TR) / A2 (BL) / A3 (BR)
//                         │
//                      [10kΩ]
//                         │
//                        GND
//
//   Pan servo signal  -> pin 9
//   Tilt servo signal -> pin 10
//   Both servos: brown/black -> GND, red -> 5V
//
// With this wiring, MORE light = HIGHER reading (0-1023).

#include <Arduino.h>
#include <Servo.h>

const int TOP_LEFT_PIN = A0;
const int TOP_RIGHT_PIN = A1;
const int BOTTOM_LEFT_PIN = A2;
const int BOTTOM_RIGHT_PIN = A3;

const int PAN_SERVO_PIN = 9;
const int TILT_SERVO_PIN = 10;

// Hysteresis: an axis starts moving when its two sides differ by more than
// START_DEADBAND, and keeps going until they differ by less than
// STOP_DEADBAND. This lets the panel settle squarely facing the light without
// jittering back and forth. Raise both if it still hunts.
const int START_DEADBAND = 30;
const int STOP_DEADBAND = 10;

// Degrees moved per step, and the pause between steps. Slow steps keep the
// panel from jerking and keep the servos' current draw down.
const int STEP_DEGREES = 1;
const unsigned long STEP_DELAY_MS = 30;

// Angle limits for each servo. Keep them away from the servos' hard stops and
// from anywhere the mount would hit itself, the base or its own wires.
// Adjust these once the mount is built: try different START angles, upload,
// and watch where things get close.
const int PAN_MIN_ANGLE = 10;
const int PAN_MAX_ANGLE = 170;
const int PAN_START_ANGLE = 90;

const int TILT_MIN_ANGLE = 20;
const int TILT_MAX_ANGLE = 160;
const int TILT_START_ANGLE = 90;

// If an axis moves AWAY from the light, flip its setting to true.
const bool REVERSE_PAN = false;
const bool REVERSE_TILT = false;

// When the average light level drops below DARK_THRESHOLD (night, or the
// panel is covered) tracking pauses. It resumes once the light rises above
// LIGHT_THRESHOLD. The gap between them stops flickering at dusk.
// Check the Serial Monitor "Avg" value in the dark and in daylight to tune.
const int DARK_THRESHOLD = 50;
const int LIGHT_THRESHOLD = 80;

// Turn off each servo's signal after it has been still this long. This stops
// the servos buzzing and saves power. Set to false if the panel sags or
// drifts (for example if the tilt servo can't hold the panel's weight).
const bool DETACH_WHEN_IDLE = true;
const unsigned long IDLE_DETACH_MS = 1000;

// How often to print readings to the Serial Monitor.
const unsigned long PRINT_INTERVAL_MS = 250;

// Everything one servo axis needs to track on its own.
struct Axis {
  Servo servo;
  int pin;
  int minAngle;
  int maxAngle;
  bool reverse;
  int angle;
  bool tracking;
  unsigned long lastMoveTime;

  Axis(int pin, int minAngle, int maxAngle, bool reverse, int startAngle)
      : pin(pin), minAngle(minAngle), maxAngle(maxAngle), reverse(reverse),
        angle(startAngle), tracking(false), lastMoveTime(0) {}
};

Axis pan(PAN_SERVO_PIN, PAN_MIN_ANGLE, PAN_MAX_ANGLE, REVERSE_PAN,
         PAN_START_ANGLE);
Axis tilt(TILT_SERVO_PIN, TILT_MIN_ANGLE, TILT_MAX_ANGLE, REVERSE_TILT,
          TILT_START_ANGLE);

bool isDark = false;
unsigned long lastPrintTime = 0;

// Average a few readings to smooth out noise.
int readLight(int pin) {
  long total = 0;
  for (int i = 0; i < 4; i++) {
    total += analogRead(pin);
  }
  return total / 4;
}

void moveServo(Axis &axis, int newAngle) {
  axis.angle = newAngle;
  axis.servo.write(axis.angle);  // set position first so attach() doesn't twitch
  if (!axis.servo.attached()) {
    axis.servo.attach(axis.pin);
  }
  axis.lastMoveTime = millis();
}

// difference > 0 means the side that should INCREASE the angle is brighter.
void updateAxis(Axis &axis, int difference) {
  if (isDark) {
    axis.tracking = false;
  } else if (!axis.tracking && abs(difference) > START_DEADBAND) {
    axis.tracking = true;
  } else if (axis.tracking && abs(difference) < STOP_DEADBAND) {
    axis.tracking = false;
  }

  if (axis.tracking) {
    int direction = (difference > 0) ? STEP_DEGREES : -STEP_DEGREES;
    if (axis.reverse) {
      direction = -direction;
    }
    int newAngle = constrain(axis.angle + direction, axis.minAngle, axis.maxAngle);
    if (newAngle != axis.angle) {
      moveServo(axis, newAngle);
    }
  }

  if (DETACH_WHEN_IDLE && axis.servo.attached() &&
      millis() - axis.lastMoveTime > IDLE_DETACH_MS) {
    axis.servo.detach();
  }
}

const char *axisStatus(const Axis &axis) {
  if (isDark) return "dark";
  return axis.tracking ? "moving" : "holding";
}

void setup() {
  Serial.begin(9600);
  moveServo(pan, PAN_START_ANGLE);
  moveServo(tilt, TILT_START_ANGLE);
  delay(500);
}

void loop() {
  int topLeft = readLight(TOP_LEFT_PIN);
  int topRight = readLight(TOP_RIGHT_PIN);
  int bottomLeft = readLight(BOTTOM_LEFT_PIN);
  int bottomRight = readLight(BOTTOM_RIGHT_PIN);

  // Average each side so the deadbands mean the same as with single sensors.
  int left = (topLeft + bottomLeft) / 2;
  int right = (topRight + bottomRight) / 2;
  int top = (topLeft + topRight) / 2;
  int bottom = (bottomLeft + bottomRight) / 2;
  int average = (topLeft + topRight + bottomLeft + bottomRight) / 4;

  int panDifference = left - right;   // positive = left side is brighter
  int tiltDifference = top - bottom;  // positive = top side is brighter

  if (isDark && average > LIGHT_THRESHOLD) {
    isDark = false;
  } else if (!isDark && average < DARK_THRESHOLD) {
    isDark = true;
  }

  updateAxis(pan, panDifference);
  updateAxis(tilt, tiltDifference);

  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    Serial.print("TL:");
    Serial.print(topLeft);
    Serial.print(" TR:");
    Serial.print(topRight);
    Serial.print(" BL:");
    Serial.print(bottomLeft);
    Serial.print(" BR:");
    Serial.print(bottomRight);
    Serial.print("  Avg:");
    Serial.print(average);
    Serial.print("  | Pan diff:");
    Serial.print(panDifference);
    Serial.print(" angle:");
    Serial.print(pan.angle);
    Serial.print(" [");
    Serial.print(axisStatus(pan));
    Serial.print("]  | Tilt diff:");
    Serial.print(tiltDifference);
    Serial.print(" angle:");
    Serial.print(tilt.angle);
    Serial.print(" [");
    Serial.print(axisStatus(tilt));
    Serial.println("]");
  }

  delay(STEP_DELAY_MS);
}
