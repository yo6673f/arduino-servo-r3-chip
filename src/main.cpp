// Dual-servo solar tracker for the Arduino Uno R3 (ATmega328P).
//
// HOW IT WORKS
// ------------
// A small solar panel sits on a 3D-printed pan/tilt mount moved by two
// servos: one turns the panel left/right (pan), the other tips it up/down
// (tilt). Mounted above the panel is a "+" shaped set of walls that splits
// the space into four corners, with one light sensor (photoresistor / LDR)
// sitting in each corner:
//
//                      TOP
//              +--------+--------+
//              |        #        |
//              |   TL   #   TR   |
//              |        #        |
//              ##########+########   <- "+" walls
//              |        #        |
//              |   BL   #   BR   |
//              |        #        |
//              +--------+--------+
//                    BOTTOM
//
// When the sun is straight in front of the panel, the walls cast no shadow
// and all four sensors see the same amount of light. When the sun is off to
// one side, the walls shade the sensors on the far side. The tracker:
//   - compares LEFT  (TL + BL) with RIGHT  (TR + BR) and turns the pan servo
//     toward the brighter side
//   - compares TOP   (TL + TR) with BOTTOM (BL + BR) and tips the tilt servo
//     toward the brighter side
// one small step at a time, until both pairs are balanced again.
//
// WIRING
// ------
// Each light sensor is wired as a voltage divider with a 10k resistor:
//
//     5V ---[ LDR ]---+--- analog pin
//                     |
//                  [ 10k ]
//                     |
//                    GND
//
//   Top-left     LDR -> A0
//   Top-right    LDR -> A1
//   Bottom-left  LDR -> A2
//   Bottom-right LDR -> A3
//   (more light = higher reading, 0-1023)
//
//   Pan  servo signal (orange/yellow) -> pin 9
//   Tilt servo signal (orange/yellow) -> pin 10
//   Both servos: red -> 5V, brown/black -> GND
//
// Two servos can pull more current than USB gives. If the board resets or
// the servos twitch, power the servos from a separate 5V supply (4xAA or a
// 5V 2A adapter) and connect that supply's GND to the Arduino's GND.

#include <Arduino.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
const int TOP_LEFT_PIN = A0;
const int TOP_RIGHT_PIN = A1;
const int BOTTOM_LEFT_PIN = A2;
const int BOTTOM_RIGHT_PIN = A3;

const int PAN_SERVO_PIN = 9;
const int TILT_SERVO_PIN = 10;

// ---------------------------------------------------------------------------
// Settings you can tune
// ---------------------------------------------------------------------------

// Servo angle limits. Keep these away from the servos' hard stops and from
// any angle where the 3D-printed mount, panel or wires would collide.
const int PAN_MIN_ANGLE = 10;
const int PAN_MAX_ANGLE = 170;
const int PAN_START_ANGLE = 90;

const int TILT_MIN_ANGLE = 20;
const int TILT_MAX_ANGLE = 160;
const int TILT_START_ANGLE = 90;

// If a servo moves AWAY from the light instead of toward it, flip its
// setting to true (depends on which way the servo is mounted).
const bool REVERSE_PAN = false;
const bool REVERSE_TILT = false;

// An axis starts moving once its two sides differ by more than
// START_DEADBAND and stops once they differ by less than STOP_DEADBAND.
// The gap between the two stops the panel jittering back and forth.
// Raise both numbers if it still twitches when facing the sun.
const int START_DEADBAND = 30;
const int STOP_DEADBAND = 10;

// How far each servo moves per step, and how long to wait between steps.
// Small, slow steps keep the panel steady and the current draw low.
const int STEP_DEGREES = 1;
const unsigned long STEP_DELAY_MS = 30;

// Night mode: when the average light drops below DARK_THRESHOLD the tracker
// stops moving, and it starts again once the light rises above
// LIGHT_THRESHOLD. Watch the "avg" value in the Serial Monitor in the dark
// and in sunlight to pick good numbers.
const int DARK_THRESHOLD = 50;
const int LIGHT_THRESHOLD = 80;

// Switch a servo's signal off after it has been still for IDLE_DETACH_MS.
// This stops the servos humming and saves power. Set to false if the panel
// sags when the servo lets go (e.g. a heavy panel on the tilt servo).
const bool DETACH_WHEN_IDLE = true;
const unsigned long IDLE_DETACH_MS = 1000;

// How often to print readings to the Serial Monitor (9600 baud).
const unsigned long PRINT_INTERVAL_MS = 250;

// ---------------------------------------------------------------------------
// Servo axis
// ---------------------------------------------------------------------------

// One servo plus everything it needs to track on its own.
struct Axis {
  const char *name;
  Servo servo;
  int pin;
  int minAngle;
  int maxAngle;
  bool reversed;
  int angle;
  bool moving;
  unsigned long lastMoveMs;

  Axis(const char *name, int pin, int minAngle, int maxAngle, bool reversed,
       int startAngle)
      : name(name), pin(pin), minAngle(minAngle), maxAngle(maxAngle),
        reversed(reversed), angle(startAngle), moving(false), lastMoveMs(0) {}
};

Axis pan("pan", PAN_SERVO_PIN, PAN_MIN_ANGLE, PAN_MAX_ANGLE, REVERSE_PAN,
         PAN_START_ANGLE);
Axis tilt("tilt", TILT_SERVO_PIN, TILT_MIN_ANGLE, TILT_MAX_ANGLE, REVERSE_TILT,
          TILT_START_ANGLE);

bool nightMode = false;
unsigned long lastPrintMs = 0;

// Send a servo to an angle, reattaching it first if it was switched off.
void setAngle(Axis &axis, int angle) {
  axis.angle = constrain(angle, axis.minAngle, axis.maxAngle);
  // Write before attach so the servo starts at the right angle instead of
  // jumping to the library's default 90 degrees first.
  axis.servo.write(axis.angle);
  if (!axis.servo.attached()) {
    axis.servo.attach(axis.pin);
  }
  axis.lastMoveMs = millis();
}

// Move one axis a step toward the brighter side if it is out of balance.
// "error" is positive when the side that INCREASES the angle is brighter.
void trackAxis(Axis &axis, int error) {
  int size = abs(error);

  if (nightMode) {
    axis.moving = false;
  } else if (!axis.moving && size > START_DEADBAND) {
    axis.moving = true;
  } else if (axis.moving && size < STOP_DEADBAND) {
    axis.moving = false;
  }

  if (axis.moving) {
    int step = (error > 0) ? STEP_DEGREES : -STEP_DEGREES;
    if (axis.reversed) {
      step = -step;
    }
    int target = constrain(axis.angle + step, axis.minAngle, axis.maxAngle);
    if (target != axis.angle) {
      setAngle(axis, target);
    }
  }

  if (DETACH_WHEN_IDLE && axis.servo.attached() &&
      millis() - axis.lastMoveMs > IDLE_DETACH_MS) {
    axis.servo.detach();
  }
}

// ---------------------------------------------------------------------------
// Light sensors
// ---------------------------------------------------------------------------

// Read a sensor a few times and average it to smooth out noise.
int readSensor(int pin) {
  long sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }
  return sum / 4;
}

// ---------------------------------------------------------------------------
// Serial Monitor output
// ---------------------------------------------------------------------------

void printAxis(const Axis &axis, int error) {
  Serial.print(axis.name);
  Serial.print(" err:");
  Serial.print(error);
  Serial.print(" angle:");
  Serial.print(axis.angle);
  Serial.print(nightMode ? " [night]" : axis.moving ? " [moving]" : " [locked]");
}

void printStatus(int tl, int tr, int bl, int br, int avg, int panError,
                 int tiltError) {
  Serial.print("TL:");
  Serial.print(tl);
  Serial.print(" TR:");
  Serial.print(tr);
  Serial.print(" BL:");
  Serial.print(bl);
  Serial.print(" BR:");
  Serial.print(br);
  Serial.print(" avg:");
  Serial.print(avg);
  Serial.print(" | ");
  printAxis(pan, panError);
  Serial.print(" | ");
  printAxis(tilt, tiltError);
  Serial.println();
}

// ---------------------------------------------------------------------------
// Main program
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("Dual-servo solar tracker starting");
  setAngle(pan, PAN_START_ANGLE);
  setAngle(tilt, TILT_START_ANGLE);
  delay(500);  // give the servos time to reach their start angles
}

void loop() {
  int tl = readSensor(TOP_LEFT_PIN);
  int tr = readSensor(TOP_RIGHT_PIN);
  int bl = readSensor(BOTTOM_LEFT_PIN);
  int br = readSensor(BOTTOM_RIGHT_PIN);

  // Average each pair (rather than adding) so the deadbands mean the same
  // thing as comparing two single sensors.
  int left = (tl + bl) / 2;
  int right = (tr + br) / 2;
  int top = (tl + tr) / 2;
  int bottom = (bl + br) / 2;
  int avg = (tl + tr + bl + br) / 4;

  if (nightMode && avg > LIGHT_THRESHOLD) {
    nightMode = false;
  } else if (!nightMode && avg < DARK_THRESHOLD) {
    nightMode = true;
  }

  int panError = left - right;   // positive = left side brighter
  int tiltError = top - bottom;  // positive = top side brighter

  trackAxis(pan, panError);
  trackAxis(tilt, tiltError);

  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printStatus(tl, tr, bl, br, avg, panError, tiltError);
  }

  delay(STEP_DELAY_MS);
}
