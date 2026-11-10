#include <DualTB9051FTGMotorShield.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCB 3        // WHITE
#define BUTTON_PIN 2

volatile long posi = 0; // encoder ticks (long so it can't overflow on long runs)

DualTB9051FTGMotorShield motor;

//=====================================================================
// 1. INPUTS / GIVEN VALUES
//=====================================================================
float targetDistance = 7.05;  // meters
float targetTime     = 12.0;  // seconds

float accelFrac = 0.10;       // fraction of targetTime spent accelerating
float decelFrac = 0.20;       // fraction of targetTime spent decelerating
                              // (cruise gets whatever is left: 1 - accelFrac - decelFrac)

float CPR      = 211.2 * 0.25;        // counts per wheel revolution (rising edge of one channel only)
float diameter = 0.073025;            // wheel diameter in meters
float circumference = PI * diameter;  // wheel circumference in meters

// Controller gain
float Kp = 50.0;      // PWM added per (m/s) of velocity error, applied every control period
float minPWM = 60;    // floor that keeps the car rolling until it reaches the target.
                      // In the test log the car stalled once PWM sagged to about 50.
float maxPWM = 200;   // output cap (library accepts -400 to 400)
float PWM = 0;        // current motor command, carried over from one loop to the next

const unsigned long LOOP_MS = 100;    // control period. One tick is ~4.3 mm, so a shorter
                                      // period makes the velocity measurement very noisy.

//=====================================================================
// Values calculated in setup()
//=====================================================================
float ticksPerMeter;
float setPoint;      // target distance in ticks
float tAccel, tCruise, tDecel;   // phase durations (s)
float vMax;          // m/s
float aAccel, aDecel;            // m/s^2 (for reference / printing)

// Run-time state
bool started  = false;
bool finished = false;
unsigned long startMs, lastLoopMs;
long lastPos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCB, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);

  motor.init();
  motor.enableDrivers();

  //===================================================================
  // 2. CALCULATE PROFILE VALUES
  //===================================================================
  if (accelFrac + decelFrac >= 1.0) {
    Serial.println("accelFrac + decelFrac must be < 1 (need some cruise time)");
    while (true);
  }

  ticksPerMeter = CPR / circumference;
  setPoint = targetDistance * ticksPerMeter;

  tAccel  = accelFrac * targetTime;
  tDecel  = decelFrac * targetTime;
  tCruise = targetTime - tAccel - tDecel;

  // Area of the trapezoid = distance:
  //   D = 0.5*Vmax*tAccel + Vmax*tCruise + 0.5*Vmax*tDecel
  //   -> Vmax = 2D / (T * (2 - accelFrac - decelFrac))
  vMax   = (2.0 * targetDistance) / (targetTime * (2.0 - accelFrac - decelFrac));
  aAccel = vMax / tAccel;
  aDecel = vMax / tDecel;

  Serial.print("Vmax (m/s): ");    Serial.println(vMax);
  Serial.print("Accel (m/s^2): "); Serial.println(aAccel);
  Serial.print("Decel (m/s^2): "); Serial.println(aDecel);
  Serial.print("Target ticks: ");  Serial.println(setPoint);
  Serial.println("t,vDes,vMeas,PWM,ticks");
}

//*************CONTROL LOOP*************//
void loop() {
  // Wait for button, then start the run
  if (!started) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(3000);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        posi = 0;
      }
      lastPos = 0;
      PWM = 0;
      startMs = millis();
      lastLoopMs = startMs;
      started = true;
    }
    return;
  }

  if (finished) {
    motor.setM1Speed(0);
    return;
  }

  // Run the controller on a fixed period so velocity is measured over a clean window
  unsigned long nowMs = millis();
  if (nowMs - lastLoopMs < LOOP_MS) return;
  float dt = (nowMs - lastLoopMs) / 1000.0;
  lastLoopMs = nowMs;
  float t = (nowMs - startMs) / 1000.0;   // seconds since the run started

  // Read position in an atomic block to avoid a misread if the interrupt fires mid-read
  long pos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // Measured velocity (m/s) over the last period
  float vMeas = ((pos - lastPos) / ticksPerMeter) / dt;
  lastPos = pos;

  float vDes = 0;

  //===================================================================
  // 3. MOTION PHASES, nested under the stop condition
  //===================================================================
  if (pos >= setPoint) {
    PWM = 0;               // reached the target distance: motor off
    finished = true;
  }
  else {
    if (t < tAccel) {
      // ACCELERATION: v(t) = (Vmax * t) / (accelFrac * targetTime)
      vDes = (vMax * t) / (accelFrac * targetTime);
    }
    else if (t < tAccel + tCruise) {
      // CRUISE: hold Vmax
      vDes = vMax;
    }
    else {
      // DECELERATION: v(t) = Vmax - (Vmax * t) / (decelFrac * targetTime)
      // where t here is time elapsed since the deceleration phase began
      float tDec = t - tAccel - tCruise;
      vDes = vMax - (vMax * tDec) / (decelFrac * targetTime);
      if (vDes < 0) vDes = 0;
    }

    // P control applied on top of the current PWM: new PWM = old PWM + Kp * error
    float error = vDes - vMeas;
    PWM = PWM + Kp * error;
    if (PWM < minPWM) PWM = minPWM;   // never sag into the stall zone before the target distance
    if (PWM > maxPWM) PWM = maxPWM;   // clamping the stored value also prevents windup
  }

  motor.setM1Speed(PWM);

  // Log (comma separated so it can be pasted into a spreadsheet)
  Serial.print(t);     Serial.print(",");
  Serial.print(vDes);  Serial.print(",");
  Serial.print(vMeas); Serial.print(",");
  Serial.print(PWM);   Serial.print(",");
  Serial.println(pos);
}

void readEncoder() {
  // NOTE: no Serial printing in here, it is far too slow for an interrupt.
  // ENCB is both the interrupt pin and the pin read below, so b is always HIGH on a
  // RISING edge and this only counts up, which is what you want for a forward-only run.
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  }
  else {
    posi--;
  }
}
