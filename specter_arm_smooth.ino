// SPECTER ESP32 UI protocol compatibility build — 2026-11-06
// Preserves existing motion/homing and safety logic. Not hardware verified.
// GPIO limits: shoulder 32/33, elbow LOWER 23, elbow UPPER 15 (boot strap).
// Configure measured base steps/revolution via KIN_BASE:<integer>.
// Repairs only the firmware/UI handshake; it does not override physical safety.

// SPECTER ESP32 Firmware - cautious faster homing + base output calibration
// Original serial protocol and safety/homing phases preserved.
// IMPORTANT: Verify limit switch GPIO32/33/23/15 inputs and A4988 driver wiring.
// Physical base is NOT homed: position 0 is only a software reference.
// KIN_BASE:<measured integer> calibrates steps per physical base output revolution.
// KIN_GET returns calibration; no web/twin changes are included in this file.
// Compile against ESP32 Arduino core + ESP32Servo + ArduinoJson v6.
// Not hardware tested. Test with unloaded arm and safe support, do not disable limits.

// SPECTER final reviewed profile — ESP32 / HW-130, 2026-11-05
// X=base GPIO18/19; Y=elbow GPIO21/22; Z=shoulder1 GPIO25/26;
// A=shoulder2 GPIO13/14 (A independent wiring MUST be verified).
// Wrist MG995 GPIO4 -> isolated HW-130 D9 signal; gripper MG90S GPIO27 -> D10 signal.
// Servo 5V supply separate; GND common. HW-130 D9/D10 are often limit-header nets;
// verify board rev and avoid other loading. GPIO32/33/35/34 limit switches active-HIGH
// with 10k pulldowns on every switch signal. GPIO34/35 have no internal pull resistors.
// ESP32 3.3V I/O / shield logic compatibility and motor supply must be verified.
// Real emergency stop requires independent hardware disconnect / torque-safe design.
// IMPORTANT: This code is not hardware-tested on the assembled robot.

// =====================================================================
//  SPECTER robotic arm firmware  (non-blocking, extra-smooth motion version)
//  Logical coordinates:  + = UP for shoulder/elbow, + = "right" for base
//  Shoulder: 0 .. calibShoulderTotalSteps  (0 = lower limit switch)
//  Elbow   : 0 .. calibElbowTotalSteps     (0 = lower limit switch)
//
//  CHANGES in this version (smoothness patch)
//   - Longer S-curve acceleration ramp (MOVE_ACCEL_STEPS = 500) and slower start speed
//   - Serial TX buffer enlarged; telemetry skipped when TX buffer is nearly full and
//     heavy JSON (cfg / waypoint list) is only sent while the arm is idle -> no step jitter
//   - Softer jog ramp up / ramp down
//   - Shorter settle time between waypoints (60 ms)
//   - Step-loss auto correction, gripper limits, servo staggering etc. kept from before
//   - HARDWARE: power the servos from a separate 5V 2A+ supply, common GND
//     with the ESP32, and add a 470-1000 uF capacitor on the servo supply.
//     Enable microstepping (1/8 or 1/16) on the stepper drivers for the best smoothness
//     (then scale all step counts / delays accordingly).
// =====================================================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>   // v6
#include <Preferences.h>

Preferences prefs;

// Structs must be defined BEFORE any function (Arduino auto-prototypes need them)
struct Sw   { bool st = false; bool candidate = false; bool initialized = false; unsigned long sinceMs = 0; uint32_t transitions = 0; };
struct Step { long t[3]; int w, g, spd, accel; };

// ---------------- PINS ----------------
#define EN_PIN         2   // LOW = motors ON
#define BASE_STEP_PIN  18
#define BASE_DIR_PIN   19
#define SH1_STEP_PIN   25
#define SH1_DIR_PIN    26
#define SH2_STEP_PIN   13
#define SH2_DIR_PIN    14
#define ELBOW_STEP_PIN 21
#define ELBOW_DIR_PIN  22
#define WRIST_PIN      4
#define GRIPPER_PIN    27

// Gripper safe range (degrees). Keep inside the mechanical limits of your jaw.
#define GRIP_MIN 20
#define GRIP_MAX 160
#define GRIP_DETACH_MS 800

// Servo pulse range (us)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// ---- Smoothness tuning ----
#define MOVE_ACCEL_STEPS   500     // steps used to accelerate / decelerate (bigger = softer)
#define MOVE_START_MAX_US  2800    // slowest start delay (us) of a move
#define WP_SETTLE_MS       60      // pause between waypoints
#define JOG_ACCEL_US       4       // jog: delay reduction per step while speeding up
#define JOG_DECEL_US       12      // jog: delay increase per step while stopping
#define TEL_MIN_TX_FREE    400     // skip telemetry if less than this many bytes free in TX buffer

// Classic ESP32 GPIO34/35 are input-only and have NO internal pulls.
// Elbow lower uses GPIO23; elbow UPPER now uses GPIO15 with internal pull-down.
// GPIO15 is a BOOT STRAPPING pin: do NOT press ELB UPPER during ESP32 power-up/reset.
// GPIO34/35 are not used: they require external bias resistors.
// Active HIGH: switch to 3V3 + 10k pulldown to GND.
// Active LOW:  switch to GND + 10k pullup to 3V3.
// NEVER apply 5V to ESP32 GPIO. This firmware cannot repair floating electrical inputs.
int pinShMin = 32, pinShMax = 33, pinElbMax = 15, pinElbMin = 23;
int shTriggerState = HIGH, elbTriggerState = HIGH;

int elbUpDir = 1;     // physical DIR level for "elbow up"   (flip with FLIP:E)
int shDownDir = 1;    // physical DIR level for "shoulder down" (flip with FLIP:S)
int baseDirPol = 1;
// Number of commanded STEP pulses per one *physical output-base* revolution.
// Zero = UNCALIBRATED. No assumption about motor microstepping or gearbox.
long baseStepsPerOutputRev = 0;

#define HOMING_ELBOW_DELAY_US    2400  // ~8% faster; revert to 2600 if stalls/step loss
#define HOMING_SHOULDER_DELAY_US 2800  // ~7% faster; revert to 3000 if stalls/step loss
#define HOMING_BACKOFF_STEPS     60
#define MIN_VALID_ELBOW_STEPS    120
#define HOMING_MAX_STEPS         40000
#define SOFT_MARGIN              20      // waypoint/home targets are kept this many steps away from the switches
#define BAUD                     460800
#define SWITCH_DEBOUNCE_MS       25UL  // stable level for 25ms (not just 1.6ms)
#define SWITCH_POLL_US           1000UL
#define SWITCH_BOOT_SETTLE_MS    100UL

// ---------------- STATE ----------------
long currentBase = 0, currentShoulder = 0, currentElbow = 0;
long calibElbowTotalSteps = 2000, calibElbowCenterSteps = 1000, calibShoulderTotalSteps = 2000;
int  currentWrist = 90, currentGripper = 90;
int  targetWrist = 90,  targetGripper = 90;
bool motorsEnabled = false, limitSwitchEnabled = true, isHomed = false;
long homeBase = 0, homeShoulder = 0, homeElbow = 0;
int  homeWrist = 90, homeGripper = 90;
bool homeCustom = false;                 // true = user-defined home pose (survives re-homing and reboot)

int targetJogDelay = 900, targetShoulderJogDelay = 1350;

#define AXIS_BASE 1
#define AXIS_SHOULDER 2
#define AXIS_ELBOW 3

String statusText = "Ready";
bool statusDirty = true, wplDirty = true, cfgDirty = true;

struct Waypoint { long base, shoulder, elbow; int wrist, gripper, speed_delay; };
#define MAX_WAYPOINTS 20
Waypoint jobQueue[MAX_WAYPOINTS];
int waypointCount = 0;

Servo wristServo, gripperServo;
unsigned long lastTelemetryMs = 0;
unsigned long gripIdleSince = 0;

void setStatus(const String &s) { statusText = s; statusDirty = true; }

// ---------------- PERSISTENCE ----------------
void saveConfig() {
  prefs.begin("arm_cfg", false);
  prefs.putInt("pShMin", pinShMin);  prefs.putInt("pShMax", pinShMax);
  prefs.putInt("pElbMax", pinElbMax); prefs.putInt("pElbMin", pinElbMin);
  prefs.putInt("shTrig", shTriggerState); prefs.putInt("elbTrig", elbTriggerState);
  prefs.putInt("elbDir", elbUpDir);  prefs.putInt("shDir", shDownDir);
  prefs.putLong("shTot", calibShoulderTotalSteps);
  prefs.putLong("bSPR", baseStepsPerOutputRev);
  prefs.putLong("elbTot", calibElbowTotalSteps);
  prefs.putLong("hB", homeBase); prefs.putLong("hS", homeShoulder); prefs.putLong("hE", homeElbow);
  prefs.putInt("hW", homeWrist); prefs.putInt("hG", homeGripper); prefs.putBool("hCus", homeCustom);
  prefs.end();
}
void saveWaypoints() {
  prefs.begin("arm_wp", false);
  prefs.putInt("n", waypointCount);
  prefs.putBytes("wp", jobQueue, sizeof(jobQueue));
  prefs.end();
  wplDirty = true;
}
bool switchPinValid(int p) {
  // Pins with internal pulls only; GPIO15 is a strapping pin (see wiring note).
  return p == 15 || p == 23 || p == 32 || p == 33;
}
void resetSwitchFilters();
void configureSwitchPins() {
  // Validate preferences / CFG values: reject duplicates or unsafe pins.
  if (!switchPinValid(pinShMin) || !switchPinValid(pinShMax) ||
      !switchPinValid(pinElbMin) || !switchPinValid(pinElbMax) ||
      pinShMin == pinShMax || pinShMin == pinElbMin || pinShMin == pinElbMax ||
      pinShMax == pinElbMin || pinShMax == pinElbMax || pinElbMin == pinElbMax) {
    pinShMin = 32; pinShMax = 33; pinElbMax = 15; pinElbMin = 23;
    setStatus("WARNING: Invalid switch GPIO config; restored 32/33/15/23");
  }
  int m1 = (shTriggerState == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN;
  int m2 = (elbTriggerState == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN;
  pinMode(pinShMin, m1);
  pinMode(pinShMax, m1);
  pinMode(pinElbMax, m2);
  pinMode(pinElbMin, m2);
  resetSwitchFilters();
}
void loadConfig() {
  prefs.begin("arm_cfg", true);
  pinShMin = prefs.getInt("pShMin", 32);  pinShMax = prefs.getInt("pShMax", 33);
  pinElbMax = prefs.getInt("pElbMax", 15); pinElbMin = prefs.getInt("pElbMin", 23);
  shTriggerState = prefs.getInt("shTrig", HIGH); elbTriggerState = prefs.getInt("elbTrig", HIGH);
  shTriggerState = (shTriggerState == LOW) ? LOW : HIGH;
  elbTriggerState = (elbTriggerState == LOW) ? LOW : HIGH;
  elbUpDir = prefs.getInt("elbDir", 1); shDownDir = prefs.getInt("shDir", 1);
  calibShoulderTotalSteps = prefs.getLong("shTot", 2000);
  baseStepsPerOutputRev = prefs.getLong("bSPR", 0);
  if (baseStepsPerOutputRev < 0 || baseStepsPerOutputRev > 10000000L) baseStepsPerOutputRev = 0;
  calibElbowTotalSteps = prefs.getLong("elbTot", 2000);
  homeBase = prefs.getLong("hB", 0); homeShoulder = prefs.getLong("hS", 0); homeElbow = prefs.getLong("hE", 0);
  homeWrist = prefs.getInt("hW", 90); homeGripper = prefs.getInt("hG", 90); homeCustom = prefs.getBool("hCus", false);
  prefs.end();
  // Migrate only the two previously supplied SPECTER layouts.
  // This prevents old Preferences from silently reassigning GPIO15 to GPIO34.
  if (pinShMin == 32 && pinShMax == 33 && pinElbMax == 34 &&
      (pinElbMin == 23 || pinElbMin == 35)) {
    pinElbMax = 15;
    pinElbMin = 23;
    saveConfig();
    setStatus("SWITCH REMAP: ELB UPPER GPIO34->GPIO15; ELB LOWER GPIO23");
  }
  homeGripper = constrain(homeGripper, GRIP_MIN, GRIP_MAX);
  calibElbowCenterSteps = calibElbowTotalSteps / 2;
  prefs.begin("arm_wp", true);
  waypointCount = constrain(prefs.getInt("n", 0), 0, MAX_WAYPOINTS);
  if (waypointCount > 0) prefs.getBytes("wp", jobQueue, sizeof(jobQueue));
  prefs.end();
  configureSwitchPins();
}

// ---------------- LIMIT SWITCHES (25ms time-based debounce + fault detection) ----------------
Sw swShMin, swShMax, swElbMin, swElbMax;
unsigned long lastSwPollUs = 0, switchesReadyAt = 0;
bool limitSignalFault = false;
uint32_t switchFaultEvents = 0;

void resetSwitchFilters() {
  swShMin = Sw(); swShMax = Sw(); swElbMin = Sw(); swElbMax = Sw();
  lastSwPollUs = 0;
  switchesReadyAt = millis() + SWITCH_BOOT_SETTLE_MS + SWITCH_DEBOUNCE_MS;
  limitSignalFault = false;
}
void pollSwitch(Sw &s, int pin, int trig, unsigned long nowMs) {
  const bool raw = (digitalRead(pin) == trig);
  if (!s.initialized) {
    s.initialized = true;
    s.candidate = raw;
    s.sinceMs = nowMs;
    return;
  }
  if (raw != s.candidate) {
    s.candidate = raw;
    s.sinceMs = nowMs;
  }
  if (s.st != s.candidate && (uint32_t)(nowMs - s.sinceMs) >= SWITCH_DEBOUNCE_MS) {
    s.st = s.candidate;
    ++s.transitions;
  }
}
bool switchesReady() { return (long)(millis() - switchesReadyAt) >= 0; }
void pollLimits() {
  unsigned long n = micros();
  if ((uint32_t)(n - lastSwPollUs) < SWITCH_POLL_US) return;
  lastSwPollUs = n;
  const unsigned long ms = millis();
  pollSwitch(swShMin, pinShMin, shTriggerState, ms);
  pollSwitch(swShMax, pinShMax, shTriggerState, ms);
  pollSwitch(swElbMin, pinElbMin, elbTriggerState, ms);
  pollSwitch(swElbMax, pinElbMax, elbTriggerState, ms);
  // Both travel ends of one axis cannot be depressed simultaneously.
  const bool bad = (swShMin.st && swShMax.st) || (swElbMin.st && swElbMax.st);
  if (switchesReady() && bad && !limitSignalFault) {
    limitSignalFault = true; cfgDirty = true;
    ++switchFaultEvents;
    setStatus("FAULT: opposite limit switches active together; check wiring/polarity");
  } else if (!bad && limitSignalFault) {
    limitSignalFault = false; cfgDirty = true; // motion still needs a fresh explicit command
    setStatus("Limit signals recovered; inspect wiring before homing");
  }
}
bool canShoulderMove(int dir) {
  if (!switchesReady() || limitSignalFault) return false;
  if (!limitSwitchEnabled) return true;
  if (dir > 0 && swShMax.st) return false;
  if (dir < 0 && swShMin.st) return false;
  return true;
}
bool canElbowMove(int dir) {
  if (!switchesReady() || limitSignalFault) return false;
  if (!limitSwitchEnabled) return true;
  if (dir > 0 && swElbMax.st) return false;
  if (dir < 0 && swElbMin.st) return false;
  return true;
}

void setMotorsPower(bool en) {
  motorsEnabled = en;
  digitalWrite(EN_PIN, en ? LOW : HIGH);
  if (!en) isHomed = false;       // position can't be trusted without holding torque
}

// ---------------- STEP PULSES (dir: +1 = logical positive) ----------------
void pulseBase(int dir) {
  digitalWrite(BASE_DIR_PIN, (dir * baseDirPol > 0) ? HIGH : LOW);
  delayMicroseconds(3);
  digitalWrite(BASE_STEP_PIN, HIGH); delayMicroseconds(5); digitalWrite(BASE_STEP_PIN, LOW);
  currentBase += dir;
}
void pulseShoulder(int dir) {               // + = up
  int lvl = (dir * -shDownDir > 0) ? HIGH : LOW;
  digitalWrite(SH1_DIR_PIN, lvl); digitalWrite(SH2_DIR_PIN, lvl);
  delayMicroseconds(3);
  digitalWrite(SH1_STEP_PIN, HIGH); digitalWrite(SH2_STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SH1_STEP_PIN, LOW);  digitalWrite(SH2_STEP_PIN, LOW);
  currentShoulder += dir;
}
void pulseElbow(int dir) {                  // + = up
  digitalWrite(ELBOW_DIR_PIN, (dir * elbUpDir > 0) ? HIGH : LOW);
  delayMicroseconds(3);
  digitalWrite(ELBOW_STEP_PIN, HIGH); delayMicroseconds(5); digitalWrite(ELBOW_STEP_PIN, LOW);
  currentElbow += dir;
}

// ---------------- SERVOS (smooth ramp, non-blocking) ----------------
unsigned long lastServoMs = 0;
void servoService() {
  unsigned long ms = millis();
  if (ms - lastServoMs < 10) return;
  lastServoMs = ms;
  if (currentWrist != targetWrist) {
    int d = targetWrist - currentWrist;
    currentWrist += constrain(d, -1, 1);
    wristServo.write(currentWrist);
  }
  if (currentGripper != targetGripper) {
    if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN, SERVO_MIN_US, SERVO_MAX_US);
    int d = targetGripper - currentGripper;
    currentGripper += constrain(d, -2, 2);
    gripperServo.write(currentGripper);
    gripIdleSince = ms;
  } else if (gripperServo.attached() && ms - gripIdleSince > GRIP_DETACH_MS) {
    gripperServo.detach();                  // stops idle buzzing / tik tik
  }
}
bool servosIdle() { return currentWrist == targetWrist && currentGripper == targetGripper; }

// ---------------- COORDINATED MOVE ENGINE ----------------
struct Move {
  bool active = false, stopReq = false;
  long steps[3], err[3], total = 0, done = 0;
  int dir[3], cruise = 900, startD = 1800, accel = 150, lastD = 1800;
  unsigned long nextUs = 0;
} mv;
bool limitHitDuringMove = false;

bool startMove(long tb, long ts, long te, int cruise, int accel) {
  if (!switchesReady() || limitSignalFault) {
    setStatus("FAULT: limit inputs not ready or contradictory; motion blocked");
    return false;
  }
  limitHitDuringMove = false;                   // FIX: stale flag from an old limit hit used to silently abort later jobs
  if (isHomed && limitSwitchEnabled) {          // soft range clamp, keeps a margin so the switch itself never trips
    if (calibShoulderTotalSteps > 2 * SOFT_MARGIN) ts = constrain(ts, (long)SOFT_MARGIN, calibShoulderTotalSteps - SOFT_MARGIN);
    if (calibElbowTotalSteps > 2 * SOFT_MARGIN)    te = constrain(te, (long)SOFT_MARGIN, calibElbowTotalSteps - SOFT_MARGIN);
  }
  long d[3] = { tb - currentBase, ts - currentShoulder, te - currentElbow };
  long total = 0; int master = 0;
  for (int i = 0; i < 3; i++) {
    mv.steps[i] = labs(d[i]); mv.dir[i] = (d[i] >= 0) ? 1 : -1;
    if (mv.steps[i] > total) { total = mv.steps[i]; master = i; }
  }
  if (total == 0) { mv.active = false; return false; }
  int minD = (master == 1) ? 1200 : 850;        // shoulder (2 motors) is slower
  if (cruise < minD) cruise = minD;
  mv.cruise = cruise;
  mv.startD = min((int)MOVE_START_MAX_US, (cruise * 5) / 2);   // slower, gentler start
  if (mv.startD < cruise) mv.startD = cruise;
  mv.accel = constrain(accel, 1, (int)max(1L, total / 2));
  mv.total = total; mv.done = 0; mv.stopReq = false;
  for (int i = 0; i < 3; i++) mv.err[i] = total / 2;
  if (!motorsEnabled) setMotorsPower(true);
  mv.lastD = mv.startD;
  mv.nextUs = micros();
  mv.active = true;
  return true;
}

void moveService() {
  if (!mv.active) return;
  unsigned long now = micros();
  if ((long)(now - mv.nextUs) < 0) return;

  int d;
  if (mv.stopReq) {                              // controlled deceleration
    mv.lastD += 40; d = mv.lastD;
    if (d >= 2600) { mv.active = false; return; }
  } else {
    long r = min(mv.done, mv.total - mv.done);
    if (r >= mv.accel) d = mv.cruise;
    else {
      float f = (float)r / mv.accel;
      f = f * f * (3.0f - 2.0f * f);             // smoothstep S-curve
      d = mv.startD - (int)((mv.startD - mv.cruise) * f);
    }
    mv.lastD = d;
  }

  bool st[3];
  for (int i = 0; i < 3; i++) {
    mv.err[i] -= mv.steps[i];
    if (mv.err[i] < 0) { mv.err[i] += mv.total; st[i] = true; } else st[i] = false;
  }
  if ((st[1] && !canShoulderMove(mv.dir[1])) || (st[2] && !canElbowMove(mv.dir[2]))) {
    mv.active = false; // hard stop on limit; do NOT continue a commanded base trajectory
    limitHitDuringMove = true;
    setStatus("🛑 Limit switch hit - motion stopped");
  }
  if (!mv.active) return;
  if (st[0]) pulseBase(mv.dir[0]);
  if (st[1]) pulseShoulder(mv.dir[1]);
  if (st[2]) pulseElbow(mv.dir[2]);

  mv.done++;
  mv.nextUs += d;
  if ((long)(micros() - mv.nextUs) > 3000) mv.nextUs = micros() + d;   // no burst catch-up
  if (mv.done >= mv.total) mv.active = false;
}

// ---------------- JOG ENGINE (ramp up, ramp down, watchdog) ----------------
int jogAxis = 0, jogDir = 0, jogDelay = 2200, pendAxis = 0, pendDir = 0;
bool jogStopping = false;
unsigned long jogNextUs = 0, lastJogCmdMs = 0;

enum { J_IDLE, J_MOVE, J_SETTLE };
int jobState = J_IDLE;
enum { H_IDLE, H_WAIT, H_ELB_UP, H_ELB_BACK, H_ELB_COUNT, H_ELB_CENTER, H_SH_DOWN, H_SH_UP,
       H_SH_BACK, H_ELB_FINAL, H_ELB_FINBACK, H_ELB_SPAN, H_DONE, H_ABORT };
int homingPhase = H_IDLE;
bool homingActive() { return homingPhase != H_IDLE && homingPhase != H_DONE && homingPhase != H_ABORT; }

void startJog(int ax, int dir) {
  if (!switchesReady() || limitSignalFault) { setStatus("FAULT: switch signals not ready; jog refused"); return; }
  bool ok = (ax == AXIS_SHOULDER) ? canShoulderMove(dir) : (ax == AXIS_ELBOW) ? canElbowMove(dir) : true;
  if (!ok) { setStatus(ax == AXIS_SHOULDER ? "🛑 Shoulder limit reached" : "🛑 Elbow limit reached"); return; }
  if (!motorsEnabled) setMotorsPower(true);
  jogAxis = ax; jogDir = dir; jogDelay = 2200; jogStopping = false;
  jogNextUs = micros(); lastJogCmdMs = millis();
}
void handleJog(int ax, int dir) {
  if (ax == 0 || dir == 0) { if (jogAxis) jogStopping = true; pendAxis = 0; return; }
  dir = (dir > 0) ? 1 : -1;
  if (mv.active || jobState != J_IDLE || homingActive()) return;
  lastJogCmdMs = millis();
  if (jogAxis == ax && jogDir == dir) { jogStopping = false; return; }
  if (jogAxis) { jogStopping = true; pendAxis = ax; pendDir = dir; return; }
  startJog(ax, dir);
}
void jogService() {
  if (!jogAxis) return;
  unsigned long now = micros();
  if ((long)(now - jogNextUs) < 0) return;
  if (!jogStopping && millis() - lastJogCmdMs > 350) jogStopping = true;   // watchdog (USB lost / key stuck)

  int target = (jogAxis == AXIS_SHOULDER) ? targetShoulderJogDelay : targetJogDelay;
  if (jogStopping) {
    jogDelay += JOG_DECEL_US;
    if (jogDelay >= 2400) {
      jogAxis = 0; jogDir = 0;
      if (pendAxis) { startJog(pendAxis, pendDir); pendAxis = 0; }
      return;
    }
  } else if (jogDelay > target) { jogDelay -= JOG_ACCEL_US; if (jogDelay < target) jogDelay = target; }
  else if (jogDelay < target)   { jogDelay += JOG_ACCEL_US; if (jogDelay > target) jogDelay = target; }

  if ((jogAxis == AXIS_SHOULDER && !canShoulderMove(jogDir)) ||
      (jogAxis == AXIS_ELBOW && !canElbowMove(jogDir))) {
    setStatus("🛑 Limit reached"); jogAxis = 0; jogDir = 0; return;
  }
  if (jogAxis == AXIS_BASE) pulseBase(jogDir);
  else if (jogAxis == AXIS_SHOULDER) pulseShoulder(jogDir);
  else pulseElbow(jogDir);
  jogNextUs += jogDelay;
  if ((long)(micros() - jogNextUs) > 3000) jogNextUs = micros() + jogDelay;
}

// ---------------- JOB / WAYPOINT RUNNER ----------------
Step runQ[MAX_WAYPOINTS + 3];
int runLen = 0, runIdx = 0;
bool jobLoop = false;
unsigned long settleUntil = 0;
const char *jobDoneMsg = "✅ Job sequence complete";

void beginStep(int i) {
  runIdx = i;
  Step &s = runQ[i];
  targetWrist = constrain(s.w, 0, 180);
  targetGripper = constrain(s.g, GRIP_MIN, GRIP_MAX);
  const bool moving = startMove(s.t[0], s.t[1], s.t[2], s.spd, s.accel);
  if (!moving && (!switchesReady() || limitSignalFault)) { jobState = J_IDLE; jobLoop = false; return; }
  jobState = J_MOVE;
  setStatus("▶️ Step " + String(i + 1) + "/" + String(runLen));
}
bool canStartJob() {
  if (!switchesReady() || limitSignalFault) { setStatus("FAULT: limit switch input problem; job refused"); return false; }
  if (homingActive() || jobState != J_IDLE || mv.active) { setStatus("⚠️ Busy"); return false; }
  if (!isHomed && limitSwitchEnabled) { setStatus("⚠️ Run HOME first (positions unknown)"); return false; }
  return true;
}
void startJob(int len, bool loop) {
  if (jogAxis) { jogAxis = 0; jogDir = 0; }
  runLen = len; jobLoop = loop;
  beginStep(0);
}
void stopAll(const char *msg) {
  jobState = J_IDLE; jobLoop = false;
  if (mv.active) mv.stopReq = true;
  if (jogAxis) jogStopping = true;
  pendAxis = 0;
  targetWrist = currentWrist; targetGripper = currentGripper;
  setStatus(msg);
}
void jobService() {
  if (jobState == J_IDLE) return;
  if (jobState == J_MOVE) {
    if (!mv.active) {
      if (limitHitDuringMove) {
        jobState = J_IDLE; jobLoop = false;
        setStatus("🛑 Job aborted: limit switch hit at step " + String(runIdx + 1) + "/" + String(runLen) + " - re-save that pose a little further from the limit");
        return;
      }
      jobState = J_SETTLE; settleUntil = millis() + WP_SETTLE_MS;
    }
  } else if (jobState == J_SETTLE) {
    if (millis() >= settleUntil && servosIdle()) {
      int next = runIdx + 1;
      if (next >= runLen) {
        if (jobLoop) beginStep(0);
        else { jobState = J_IDLE; setStatus(jobDoneMsg); }
      } else beginStep(next);
    }
  }
}
Step stepFrom(long b, long s, long e, int w, int g, int spd, int acc) {
  Step st; st.t[0] = b; st.t[1] = s; st.t[2] = e; st.w = w; st.g = g; st.spd = spd; st.accel = acc; return st;
}

// ---------------- AUTO HOMING (ORIGINAL FULL STATE MACHINE, SAFER SWITCH HANDLING) ----------------
unsigned long homingTimerMs = 0, homeDwellUntil = 0, lastHomingStepUs = 0;
long homeCnt = 0;
long spanLast = 0; int spanPass = 0, spanDir = 1, driftEvents = 0;
// Extra diagnostics only; no original homing phase or function is removed.
uint32_t homingAbortEvents = 0;
int homingLastAbortPhase = H_IDLE;
// Backoff must release the switch. Allow a limited extra distance for switch hysteresis.
#define HOMING_RELEASE_EXTRA_STEPS  100
#define HOMING_PHASE_TIMEOUT_MS     150000UL
unsigned long homingPhaseStartMs = 0;
const char *homingPhaseName(int ph) {
  switch (ph) {
    case H_WAIT: return "WAIT"; case H_ELB_UP: return "ELB_UP";
    case H_ELB_BACK: return "ELB_BACK"; case H_ELB_COUNT: return "ELB_COUNT";
    case H_ELB_SPAN: return "ELB_SPAN"; case H_ELB_CENTER: return "ELB_CENTER";
    case H_SH_DOWN: return "SH_DOWN"; case H_SH_UP: return "SH_UP";
    case H_SH_BACK: return "SH_BACK"; case H_ELB_FINAL: return "ELB_FINAL";
    case H_ELB_FINBACK: return "ELB_FINBACK"; case H_DONE: return "DONE";
    case H_ABORT: return "ABORT"; default: return "IDLE";
  }
}

// ---------------- STEP-LOSS AUTO CORRECTION ----------------
// Whenever a limit switch is touched outside homing, the exact position is known there
// (low switch = 0, high switch = total span). We compare it with the counted position,
// report any drift and re-sync, so lost steps never pile up.
bool pvShMin = false, pvShMax = false, pvElbMin = false, pvElbMax = false;
void resyncAxis(long &cur, long ref, const char *name) {
  long d = cur - ref;
  if (labs(d) > 25) {
    driftEvents++;
    setStatus("⚠️ Step loss: " + String(name) + " was off by " + String(d) + " steps - position corrected");
  }
  cur = ref;
}
void checkResync() {
  // Only correct coordinates when the switch was approached by a moving axis.
  // Stationary/noisy switch edges must never fabricate "step loss".
  bool active = isHomed && limitSwitchEnabled && !homingActive() && switchesReady() && !limitSignalFault;
  if (active) {
    if (swShMin.st && !pvShMin && ((jogAxis == AXIS_SHOULDER && jogDir < 0) || (mv.active && mv.dir[1] < 0)))  resyncAxis(currentShoulder, 0, "shoulder (low)");
    if (swShMax.st && !pvShMax && ((jogAxis == AXIS_SHOULDER && jogDir > 0) || (mv.active && mv.dir[1] > 0)))  resyncAxis(currentShoulder, calibShoulderTotalSteps, "shoulder (high)");
    if (swElbMin.st && !pvElbMin && ((jogAxis == AXIS_ELBOW && jogDir < 0) || (mv.active && mv.dir[2] < 0))) resyncAxis(currentElbow, 0, "elbow (low)");
    if (swElbMax.st && !pvElbMax && ((jogAxis == AXIS_ELBOW && jogDir > 0) || (mv.active && mv.dir[2] > 0))) resyncAxis(currentElbow, calibElbowTotalSteps, "elbow (high)");
  }
  pvShMin = swShMin.st; pvShMax = swShMax.st; pvElbMin = swElbMin.st; pvElbMax = swElbMax.st;
}

void homingNext(int ph, const String &msg, unsigned long dwellMs) {
  homingPhase = ph; homeCnt = 0; homingPhaseStartMs = millis(); homeDwellUntil = millis() + dwellMs; setStatus(msg);
}
void homingAbort(const String &msg) {
  // Stop stepping, but retain existing holding torque when motor power was ON.
  // Switching EN off here can make a gravity-loaded arm drop at a limit.
  // Hardware E-stop/power disconnect remains a separate deliberate action.
  homingLastAbortPhase = homingPhase;
  ++homingAbortEvents;
  homingPhase = H_ABORT;
  isHomed = false;
  jogAxis = 0; jogDir = 0; pendAxis = 0; jogStopping = false;
  mv.active = false; jobState = J_IDLE; jobLoop = false;
  targetWrist = currentWrist; targetGripper = currentGripper;
  setStatus("HOME ABORT [" + String(homingPhaseName(homingLastAbortPhase)) + "]: " + msg + " | hold retained");
}

void homingService() {
  if (!homingActive()) return;
  if (!switchesReady() || limitSignalFault) {
    homingAbort("FAULT: invalid limit signals; auto-home aborted");
    return;
  }
  unsigned long ms = millis();
  if ((uint32_t)(ms - homingPhaseStartMs) > HOMING_PHASE_TIMEOUT_MS) {
    homingAbort("phase timeout; check switch release, DIR polarity and supply"); return;
  }
  if (homingPhase == H_WAIT) {
    if (ms - homingTimerMs >= 1200) homingNext(H_ELB_UP, "1/6: Elbow -> upper limit", 0);
    return;
  }
  if (ms < homeDwellUntil) return;
  bool shPhase = (homingPhase == H_SH_DOWN || homingPhase == H_SH_UP || homingPhase == H_SH_BACK);
  unsigned long us = micros();
  unsigned long dly = shPhase ? HOMING_SHOULDER_DELAY_US : HOMING_ELBOW_DELAY_US;
  if (homingPhase == H_ELB_SPAN && spanPass >= 2) dly = dly * 3 / 2;     // slower retries = fewer lost steps
  if (us - lastHomingStepUs < dly) return;
  lastHomingStepUs = us;
  // Stable pressed switch in the *approach direction* means no more pulses in that direction.
  // The normal end-seeking phases check their target before issuing a pulse below.
  switch (homingPhase) {
    case H_ELB_UP:
      if (swElbMax.st) { homingNext(H_ELB_BACK, "1.5/6: Upper limit hit, backing off", 150); break; }
      pulseElbow(+1);
      if (++homeCnt > HOMING_MAX_STEPS) homingAbort("⚠️ Elbow upper limit not found");
      break;
    case H_ELB_BACK:
      if (homeCnt >= HOMING_BACKOFF_STEPS && !swElbMax.st) {
        calibElbowTotalSteps = homeCnt;
        homingNext(H_ELB_COUNT, "2/6: Measuring elbow span", 180);
      } else if (homeCnt >= HOMING_BACKOFF_STEPS + HOMING_RELEASE_EXTRA_STEPS) {
        homingAbort("elbow UPPER switch did not release during backoff");
      } else if (swElbMin.st) {
        homingAbort("elbow LOWER triggered during upper backoff");
      } else { pulseElbow(-1); homeCnt++; }
      break;
    case H_ELB_COUNT:
      if (swElbMin.st) {
        if (calibElbowTotalSteps < MIN_VALID_ELBOW_STEPS) {
          homingAbort("elbow LOWER reached too early; span too short or switch noisy");
        } else {
          currentElbow = 0;
          calibElbowCenterSteps = calibElbowTotalSteps / 2;
          spanLast = calibElbowTotalSteps; spanPass = 1; spanDir = 1;
          homingNext(H_ELB_SPAN, "2.5/6: Verifying elbow span (up)", 180);
        }
      } else if (swElbMax.st) {
        homingAbort("elbow UPPER re-triggered while moving down");
      } else {
        pulseElbow(-1); calibElbowTotalSteps++;
        if (calibElbowTotalSteps > HOMING_MAX_STEPS) homingAbort("Elbow lower limit not found");
      }
      break;
    case H_ELB_CENTER:
      if (currentElbow == calibElbowCenterSteps ||
          (currentElbow < calibElbowCenterSteps && homeCnt > MIN_VALID_ELBOW_STEPS && swElbMax.st)) {
        homingNext(H_SH_DOWN, "4/6: Shoulder -> lower limit", 250);
      } else if (currentElbow > calibElbowCenterSteps) {   // came from the upper limit after the span check
        if (swElbMin.st) homingAbort("⚠️ Elbow hit lower limit while centring");
        else pulseElbow(-1);
      } else if (swElbMax.st) {
        homingAbort("elbow UPPER hit before desired centre");
      } else { pulseElbow(+1); homeCnt++; }
      break;
    case H_ELB_SPAN: {                       // measure the elbow span again (up/down) until two passes agree
      bool atEnd = (spanDir > 0) ? swElbMax.st : swElbMin.st;
      if (atEnd && homeCnt < MIN_VALID_ELBOW_STEPS) {
        homingAbort("elbow verification limit reached too early; noisy switch or bad span");
      } else if (atEnd) {
        long span = homeCnt;
        long tol = max(12L, spanLast / 100);          // 1 % (at least 12 steps)
        bool same = labs(span - spanLast) <= tol;
        currentElbow = (spanDir > 0) ? span : 0;
        if (same) {
          calibElbowTotalSteps = span;
          calibElbowCenterSteps = span / 2;
          homingNext(H_ELB_CENTER, "3/6: Elbow span verified (" + String(span) + " steps), centring", 300);
        } else if (spanPass >= 4) {
          // Never accept an unverified span: an incorrect soft limit can cause a collision.
          homingAbort("elbow span disagreed on 4 passes: " + String(spanLast) + " vs " + String(span));
        } else {
          driftEvents++;
          homingNext(H_ELB_SPAN, "⚠️ Step loss: elbow span " + String(spanLast) + " vs " + String(span) + ", measuring again slower", 300);
          spanLast = span; spanPass++; spanDir = -spanDir;
        }
      } else {
        // The opposite end must release as the arm leaves it. Never step INTO a
        // currently active destination switch merely to satisfy a step threshold.
        bool leavingEnd = (spanDir > 0) ? swElbMin.st : swElbMax.st;
        if (leavingEnd && homeCnt > HOMING_BACKOFF_STEPS + HOMING_RELEASE_EXTRA_STEPS) {
          homingAbort("elbow starting switch stuck during span verification");
        } else {
          pulseElbow(spanDir); homeCnt++;
          if (homeCnt > HOMING_MAX_STEPS) homingAbort("Elbow limit not found while verifying span");
        }
      }
      break;
    }
    case H_SH_DOWN:
      if (swShMin.st) {
        currentShoulder = 0;
        homingNext(H_SH_UP, "5/6: Measuring shoulder span", 250);
      } else if (swShMax.st && homeCnt >= HOMING_BACKOFF_STEPS + HOMING_RELEASE_EXTRA_STEPS) {
        homingAbort("shoulder UPPER switch did not release while moving down; check switch and DIR");
      } else {
        pulseShoulder(-1);
        if (++homeCnt > HOMING_MAX_STEPS) homingAbort("⚠️ Shoulder lower limit not found");
      }
      break;
    case H_SH_UP:
      if (swShMax.st) {
        if (currentShoulder <= 50) homingAbort("shoulder UPPER reached too early");
        else {
          calibShoulderTotalSteps = currentShoulder;
          homingNext(H_SH_BACK, "5.5/6: Shoulder backoff", 180);
        }
      } else if (swShMin.st && currentShoulder >= HOMING_BACKOFF_STEPS + HOMING_RELEASE_EXTRA_STEPS) {
        homingAbort("shoulder LOWER switch did not release while moving up; check switch and DIR");
      } else {
        pulseShoulder(+1);
        if (currentShoulder > HOMING_MAX_STEPS) homingAbort("Shoulder upper limit not found");
      }
      break;
    case H_SH_BACK:
      if (homeCnt >= HOMING_BACKOFF_STEPS && !swShMax.st) {
        if (!homeCustom) homeShoulder = currentShoulder;
        homingNext(H_ELB_FINAL, "6/6: Elbow -> home", 250);
      } else if (homeCnt >= HOMING_BACKOFF_STEPS + HOMING_RELEASE_EXTRA_STEPS) {
        homingAbort("shoulder UPPER switch did not release during backoff");
      } else if (swShMin.st) {
        homingAbort("shoulder LOWER triggered during upper backoff");
      } else { pulseShoulder(-1); homeCnt++; }
      break;
    case H_ELB_FINAL:
      if (swElbMin.st) {
        long dr = currentElbow; currentElbow = 0;                 // should be exactly 0 here: free consistency check
        if (labs(dr) > max(12L, calibElbowTotalSteps / 100)) { driftEvents++; homingNext(H_ELB_FINBACK, "⚠️ Step loss: elbow drifted " + String(dr) + " steps during homing (corrected)", 250); }
        else homingNext(H_ELB_FINBACK, "6/6: Elbow backoff", 250);
      }
      else { pulseElbow(-1); if (++homeCnt > HOMING_MAX_STEPS) homingAbort("⚠️ Elbow lower limit not found"); }
      break;
    case H_ELB_FINBACK:
      if (homeCnt >= HOMING_BACKOFF_STEPS && !swElbMin.st) {
        currentBase = 0;
        if (!homeCustom) {                       // FIX: homing no longer overwrites a user-defined home pose
          homeBase = 0; homeShoulder = currentShoulder; homeElbow = currentElbow; homeWrist = 90; homeGripper = 90;
        }
        isHomed = true; cfgDirty = true; saveConfig();
        targetWrist = homeWrist; targetGripper = homeGripper;   // wrist + gripper go to the saved home values after homing
        homingPhase = H_DONE;
        setStatus("✅ Homing complete (Sh " + String(calibShoulderTotalSteps) + " / Elb " + String(calibElbowTotalSteps) + ")");
      } else if (homeCnt >= HOMING_BACKOFF_STEPS + HOMING_RELEASE_EXTRA_STEPS) {
        homingAbort("elbow LOWER switch did not release during final backoff");
      } else if (swElbMax.st) {
        homingAbort("elbow UPPER triggered during final backoff");
      } else { pulseElbow(+1); homeCnt++; }
      break;
  }
}

// ---------------- LIVE SAFETY / UI HANDSHAKE ----------------
// A configuration acknowledgement is not a clearance to move. Every command
// still goes through the original GPIO, limit and homing safety conditions.
static bool repairLockedNow() {
  return !switchesReady() || limitSignalFault;
}
void sendHomeDiag() {
  StaticJsonDocument<512> d;
  d["t"] = "home_diag";
  d["protocolVersion"] = 2;
  d["repairLockSupported"] = true;
  d["repairLock"] = repairLockedNow();
  d["repair_lock"] = repairLockedNow();
  d["configValid"] = switchesReady() && !limitSignalFault;
  d["ph"] = homingPhase;
  d["phase"] = homingPhaseName(homingPhase);
  d["abortPhase"] = homingPhaseName(homingLastAbortPhase);
  d["abortCount"] = homingAbortEvents;
  d["lf"] = limitSignalFault ? 1 : 0;
  d["sw"] = (swShMin.st ? 1 : 0) | (swShMax.st ? 2 : 0) |
             (swElbMin.st ? 4 : 0) | (swElbMax.st ? 8 : 0);
  d["en"] = motorsEnabled ? 1 : 0;
  d["hm"] = isHomed ? 1 : 0;
  serializeJson(d, Serial); Serial.println();
}
// ---------------- SERIAL OUT ----------------
void sendTelemetry() {
  uint8_t sw = (swShMin.st ? 1 : 0) | (swShMax.st ? 2 : 0) | (swElbMin.st ? 4 : 0) | (swElbMax.st ? 8 : 0);
  // Same bit order as dashboard. rSw is instantaneous electrical level (not safety-debounced).
  uint8_t rSw = ((digitalRead(pinShMin) == shTriggerState) ? 1 : 0) |
                ((digitalRead(pinShMax) == shTriggerState) ? 2 : 0) |
                ((digitalRead(pinElbMin) == elbTriggerState) ? 4 : 0) |
                ((digitalRead(pinElbMax) == elbTriggerState) ? 8 : 0);
  char b[512];
  int n = snprintf(b, sizeof(b),
    "{\"t\":\"tel\",\"b\":%ld,\"s\":%ld,\"e\":%ld,\"w\":%d,\"g\":%d,\"en\":%d,\"lim\":%d,\"hm\":%d,\"job\":%d,\"ri\":%d,\"rn\":%d,\"ph\":%d,\"sw\":%d,\"dr\":%d,\"rawSw\":%d,\"lf\":%d,\"fl\":%lu,\"repairLock\":%d,\"repair_lock\":%d}\n",
    currentBase, currentShoulder, currentElbow, currentWrist, currentGripper,
    motorsEnabled, limitSwitchEnabled, isHomed, jobState != J_IDLE, runIdx, runLen, homingPhase, sw, driftEvents, rSw, (int)limitSignalFault, (unsigned long)switchFaultEvents, (int)repairLockedNow(), (int)repairLockedNow());
  Serial.write((const uint8_t *)b, n);
}
void sendStatus() {
  StaticJsonDocument<320> d; d["t"] = "st"; d["m"] = statusText;
  serializeJson(d, Serial); Serial.println(); statusDirty = false;
}
void sendCfg() {
  StaticJsonDocument<1024> d;
  d["t"] = "cfg"; d["pShMin"] = pinShMin; d["pShMax"] = pinShMax; d["pElbMin"] = pinElbMin; d["pElbMax"] = pinElbMax;
  d["shTrig"] = shTriggerState; d["elbTrig"] = elbTriggerState; d["shDir"] = shDownDir; d["elbDir"] = elbUpDir;
  d["baseStepsPerRev"] = baseStepsPerOutputRev;
  // Compat keys for a Web UI expecting explicit firmware safety negotiation.
  d["protocolVersion"] = 2;
  d["firmware"] = "SPECTER";
  d["repairLockSupported"] = true;
  d["repairLock"] = repairLockedNow();
  d["repair_lock"] = repairLockedNow();
  d["configValid"] = switchesReady() && !limitSignalFault;
  d["lf"] = limitSignalFault ? 1 : 0;
  d["en"] = motorsEnabled ? 1 : 0;
  d["hm"] = isHomed ? 1 : 0;
  d["shTot"] = calibShoulderTotalSteps; d["elbTot"] = calibElbowTotalSteps; d["elbCen"] = calibElbowCenterSteps;
  d["hB"] = homeBase; d["hS"] = homeShoulder; d["hE"] = homeElbow; d["hW"] = homeWrist; d["hG"] = homeGripper; d["hC"] = homeCustom ? 1 : 0;
  serializeJson(d, Serial); Serial.println(); cfgDirty = false;
}
void sendWpl() {
  Serial.print("{\"t\":\"wpl\",\"w\":[");
  for (int i = 0; i < waypointCount; i++) {
    if (i) Serial.print(',');
    Serial.printf("[%ld,%ld,%ld,%d,%d,%d]", jobQueue[i].base, jobQueue[i].shoulder, jobQueue[i].elbow,
                  jobQueue[i].wrist, jobQueue[i].gripper, jobQueue[i].speed_delay);
  }
  Serial.println("]}");
  wplDirty = false;
}

// ---------------- COMMANDS ----------------
void haltEverything() {
  jogAxis = 0; jogDir = 0; pendAxis = 0; jobState = J_IDLE; jobLoop = false;
  mv.active = false; homingPhase = H_ABORT;
  targetWrist = currentWrist; targetGripper = currentGripper;
}

void processCommand(String line) {
  line.trim();
  if (!line.length()) return;

  if (line == "ESTOP") { haltEverything(); setMotorsPower(false); setStatus("🛑 EMERGENCY STOP: motors OFF"); return; }
  if (line == "STOP")  { if (homingActive()) homingAbort("Stopped by user"); else stopAll("⏹ Motion stopped"); return; }

  if (line.startsWith("J:")) {
    int c2 = line.indexOf(':', 2);
    if (c2 > 0) handleJog(line.substring(2, c2).toInt(), line.substring(c2 + 1).toInt());
  }
  else if (line.startsWith("G:")) targetGripper = constrain(line.substring(2).toInt(), GRIP_MIN, GRIP_MAX);
  else if (line.startsWith("W:")) targetWrist   = constrain(line.substring(2).toInt(), 0, 180);
  else if (line.startsWith("SPD:")) {
    targetJogDelay = constrain(line.substring(4).toInt(), 650, 2500);
    targetShoulderJogDelay = max(1200, targetJogDelay + 350);
  }
  else if (line == "GET") {
    // The UI needs an explicit response before enabling commands.
    // Don't wait indefinitely for an idle-only periodic telemetry slot.
    cfgDirty = wplDirty = statusDirty = true;
    if (!homingActive() && !mv.active && !jogAxis && jobState == J_IDLE && Serial.availableForWrite() >= 1000) {
      sendCfg();
      sendHomeDiag();
    }
  }
  else if (line == "KIN_GET") {
    StaticJsonDocument<256> k;
    k["t"] = "kin";
    k["bSteps"] = currentBase;
    k["sSteps"] = currentShoulder;
    k["eSteps"] = currentElbow;
    k["shSpan"] = calibShoulderTotalSteps;
    k["elSpan"] = calibElbowTotalSteps;
    k["baseStepsPerRev"] = baseStepsPerOutputRev;
    k["homed"] = isHomed ? 1 : 0;
    serializeJson(k, Serial); Serial.println();
  }
  else if (line.startsWith("KIN_BASE:")) {
    // Manual measured calibration only, NEVER guessed automatically.
    // Only accept while completely idle. Actual base position still needs
    // an external reference; homing other joints does not home the base.
    if (homingActive() || mv.active || jogAxis || jobState != J_IDLE) {
      setStatus("KIN_BASE rejected: stop all motion first"); return;
    }
    String raw = line.substring(9); raw.trim();
    bool valid = raw.length() > 0;
    for (size_t i = 0; i < raw.length(); ++i)
      if (raw[i] < '0' || raw[i] > '9') valid = false;
    long value = valid ? raw.toInt() : -1;
    if (value < 200 || value > 10000000L) {
      setStatus("KIN_BASE rejected: supply measured output steps/rev 200..10000000"); return;
    }
    baseStepsPerOutputRev = value;
    saveConfig(); cfgDirty = true;
    setStatus("Base revolution calibration saved: " + String(value) + " steps/output revolution");
  }
  else if (line == "SW_DIAG") {
    // Independent per-pin measurements. Never mask an activated safety input.
    const int a = digitalRead(pinShMin), b = digitalRead(pinShMax);
    const int c = digitalRead(pinElbMin), d = digitalRead(pinElbMax);
    const int activeSh = shTriggerState, activeEl = elbTriggerState;
    setStatus("SW_DIAG SH_DOWN(GPIO" + String(pinShMin) + ")=" + String(a) +
              " SH_UP(GPIO" + String(pinShMax) + ")=" + String(b) +
              " ELB_DOWN(GPIO" + String(pinElbMin) + ")=" + String(c) +
              " ELB_UP(GPIO" + String(pinElbMax) + ")=" + String(d) +
              " TRIG_SH=" + String(activeSh) + " TRIG_EL=" + String(activeEl) +
              " (1=HIGH,0=LOW; compare released/pressed)");
  }
  else if (line == "MOTOR_DIAG") {
    // Non-motion diagnostic: GPIO2 is active LOW on CNC Shield enable.
    // GPIO readback verifies only ESP32 output, not voltage at the A4988 EN pad.
    StaticJsonDocument<384> d;
    d["t"] = "motor_diag";
    d["enable_gpio"] = EN_PIN;
    d["enable_level"] = digitalRead(EN_PIN);
    d["enabled_software"] = motorsEnabled ? 1 : 0;
    d["switch_fault"] = limitSignalFault ? 1 : 0;
    d["raw_sh_min"] = digitalRead(pinShMin);
    d["raw_sh_max"] = digitalRead(pinShMax);
    d["raw_el_min"] = digitalRead(pinElbMin);
    d["raw_el_max"] = digitalRead(pinElbMax);
    d["trig_sh"] = shTriggerState;
    d["trig_el"] = elbTriggerState;
    serializeJson(d, Serial); Serial.println();
  }
  else if (line == "HOME_DIAG") {
    sendHomeDiag();
    setStatus("HOME DIAG phase=" + String(homingPhaseName(homingPhase)) +
              " lastAbort=" + String(homingPhaseName(homingLastAbortPhase)) +
              " count=" + String(homingAbortEvents) +
              " SW=" + String((swShMin.st ? 1 : 0) | (swShMax.st ? 2 : 0) |
                             (swElbMin.st ? 4 : 0) | (swElbMax.st ? 8 : 0)) +
              " EN=" + String((int)motorsEnabled));
  }
  else if (line.startsWith("CFG:")) {
    StaticJsonDocument<512> c;
    if (!deserializeJson(c, line.substring(4))) {
      if (homingActive() || mv.active || jogAxis || jobState != J_IDLE) {
        setStatus("Stop motion before changing GPIO configuration"); return;
      }
      const int a = c["pShMin"] | pinShMin, b = c["pShMax"] | pinShMax;
      const int d = c["pElbMin"] | pinElbMin, e = c["pElbMax"] | pinElbMax;
      // Reject invalid/duplicate pins rather than silently reassigning live safety inputs.
      if (!switchPinValid(a) || !switchPinValid(b) || !switchPinValid(d) || !switchPinValid(e) ||
          a == b || a == d || a == e || b == d || b == e || d == e) {
        setStatus("FAULT: invalid or duplicate limit GPIOs; config rejected"); return;
      }
      long shNew = c["shTot"] | calibShoulderTotalSteps;
      long elNew = c["elbTot"] | calibElbowTotalSteps;
      if (shNew < 100 || shNew > HOMING_MAX_STEPS || elNew < 100 || elNew > HOMING_MAX_STEPS) {
        setStatus("FAULT: invalid travel span; config rejected"); return;
      }
      pinShMin = a; pinShMax = b; pinElbMin = d; pinElbMax = e;
      shTriggerState = (c["shTrig"] | shTriggerState) == LOW ? LOW : HIGH;
      elbTriggerState = (c["elbTrig"] | elbTriggerState) == LOW ? LOW : HIGH;
      calibShoulderTotalSteps = shNew; calibElbowTotalSteps = elNew;
      calibElbowCenterSteps = calibElbowTotalSteps / 2;
      configureSwitchPins(); saveConfig(); cfgDirty = true;
      isHomed = false;
      setStatus("✅ Config saved. Verify released/pressed switch readings before HOME.");
    }
  }
  else if (line == "SW_DEFAULT") {
    if (homingActive() || mv.active || jogAxis || jobState != J_IDLE) {
      setStatus("Stop all movement before restoring switch defaults"); return;
    }
    pinShMin = 32; pinShMax = 33; pinElbMin = 23; pinElbMax = 15;
    shTriggerState = HIGH; elbTriggerState = HIGH;
    configureSwitchPins(); saveConfig(); cfgDirty = true; isHomed = false;
    setStatus("Switch defaults restored: active HIGH, 32/33/23/15. Verify wiring.");
  }
  else if (line == "PWR") {
    haltEverything(); setMotorsPower(!motorsEnabled);
    if (motorsEnabled && limitSignalFault)
      setStatus("ENABLE GPIO2 LOW, but motion BLOCKED: opposite elbow/shoulder limits active. Run MOTOR_DIAG; measure EN pad and driver VDD.");
    else setStatus(motorsEnabled ? "Motors commanded ON: GPIO2 LOW. If no holding torque, check CNC Shield EN/VDD/VMOT and A4988." : "Motors commanded OFF: GPIO2 HIGH (re-home before use)");
  }
  else if (line == "LIM") { limitSwitchEnabled = true; setStatus("Limit protection stays ON for safety"); }
  else if (line == "FLIP:S") { shDownDir = -shDownDir; saveConfig(); cfgDirty = true; setStatus("🔄 Shoulder direction flipped"); }
  else if (line == "FLIP:E") { elbUpDir = -elbUpDir;   saveConfig(); cfgDirty = true; setStatus("🔄 Elbow direction flipped"); }
  else if (line == "FLIP:B") { baseDirPol = -baseDirPol; setStatus("🔄 Base direction flipped"); }
  else if (line == "HOME") {
    haltEverything();
    limitSwitchEnabled = true;
    if (!switchesReady() || limitSignalFault) { setStatus("FAULT: limit inputs unstable/contradictory; HOME refused"); return; }
    // A simultaneous shoulder+elbow upper reading is suspicious during the reported fault.
    // Refuse automatic homing until signals are checked; DO NOT ignore either switch.
    if (swShMax.st && swElbMax.st) {
      setStatus("FAULT: SH UP and ELB UP both active; inspect GPIO33/34 signals before HOME");
      return;
    }
    setMotorsPower(true); isHomed = false;
    currentBase = 0; homeBase = 0;
    homingTimerMs = millis(); homingPhaseStartMs = homingTimerMs; homingPhase = H_WAIT;
    setStatus("Auto-homing starts in 1.2s; clear the workspace");
  }
  else if (line == "WP_ADD") {
    if (waypointCount < MAX_WAYPOINTS) {
      jobQueue[waypointCount++] = { currentBase, currentShoulder, currentElbow, targetWrist, targetGripper, targetJogDelay };
      saveWaypoints(); setStatus("✅ Saved waypoint #" + String(waypointCount));
    } else setStatus("⚠️ Waypoint list full (20)");
  }
  else if (line.startsWith("WP_UPD:")) {
    int i = line.substring(7).toInt();
    if (i >= 0 && i < waypointCount) {
      jobQueue[i] = { currentBase, currentShoulder, currentElbow, targetWrist, targetGripper, targetJogDelay };
      saveWaypoints(); setStatus("✏️ Waypoint #" + String(i + 1) + " updated");
    }
  }
  else if (line.startsWith("WP_DEL:")) {
    int i = line.substring(7).toInt();
    if (i >= 0 && i < waypointCount && jobState == J_IDLE) {
      for (int k = i; k < waypointCount - 1; k++) jobQueue[k] = jobQueue[k + 1];
      waypointCount--; saveWaypoints(); setStatus("🗑️ Waypoint deleted");
    }
  }
  else if (line == "WP_CLR") { if (jobState == J_IDLE) { waypointCount = 0; saveWaypoints(); setStatus("🗑️ Waypoints cleared"); } }
  else if (line.startsWith("WP_GO:")) {
    int i = line.substring(6).toInt();
    if (i >= 0 && i < waypointCount && canStartJob()) {
      Waypoint &w = jobQueue[i];
      runQ[0] = stepFrom(w.base, w.shoulder, w.elbow, w.wrist, w.gripper, w.speed_delay, MOVE_ACCEL_STEPS);
      jobDoneMsg = "✅ Reached pose";
      startJob(1, false);
    }
  }
  else if (line.startsWith("WP_RUN")) {
    if (waypointCount == 0) { setStatus("⚠️ No waypoints saved"); return; }
    if (!canStartJob()) return;
    for (int i = 0; i < waypointCount; i++) {
      Waypoint &w = jobQueue[i];
      runQ[i] = stepFrom(w.base, w.shoulder, w.elbow, w.wrist, w.gripper, w.speed_delay, MOVE_ACCEL_STEPS);
    }
    jobDoneMsg = "✅ Job sequence complete";
    startJob(waypointCount, line.endsWith(":L"));
  }
  else if (line == "SET_HOME") {
    if (!isHomed && limitSwitchEnabled) { setStatus("⚠️ Run HOME first - positions are unknown, home pose not saved"); return; }
    homeBase = currentBase; homeShoulder = currentShoulder; homeElbow = currentElbow;
    homeWrist = targetWrist; homeGripper = targetGripper; homeCustom = true;   // save what the sliders say, not the lagging position
    saveConfig(); cfgDirty = true;                       // stored in flash, survives reboot + re-homing
    setStatus("🏠 Home pose saved (Sh " + String(homeShoulder) + " / Elb " + String(homeElbow) + " / Base " + String(homeBase) + ")");
  }
  else if (line.startsWith("HCFG:")) {                   // home pose typed in the UI (values in steps)
    StaticJsonDocument<256> c;
    if (!deserializeJson(c, line.substring(5))) {
      homeBase     = c["b"] | homeBase;
      homeShoulder = constrain((long)(c["s"] | homeShoulder), 0L, calibShoulderTotalSteps);
      homeElbow    = constrain((long)(c["e"] | homeElbow),    0L, calibElbowTotalSteps);
      homeWrist    = constrain((int)(c["w"] | homeWrist),   0, 180);
      homeGripper  = constrain((int)(c["g"] | homeGripper), GRIP_MIN, GRIP_MAX);
      homeCustom = true; saveConfig(); cfgDirty = true;
      setStatus("🏠 Home pose updated and saved");
    } else setStatus("⚠️ Home pose data invalid");
  }
  else if (line == "HOME_RESET") {
    homeCustom = false;
    homeBase = 0; homeShoulder = max(0L, calibShoulderTotalSteps - HOMING_BACKOFF_STEPS); homeElbow = HOMING_BACKOFF_STEPS;
    homeWrist = 90; homeGripper = 90;
    saveConfig(); cfgDirty = true;
    setStatus("🏠 Home pose reset to the auto-home default");
  }
  else if (line.startsWith("GOTO:")) {                   // single move: GOTO:base,shoulder,elbow,wrist,gripper,speed (used by UI modes)
    long b, s, e; int w, g, spd;
    if (sscanf(line.c_str() + 5, "%ld,%ld,%ld,%d,%d,%d", &b, &s, &e, &w, &g, &spd) == 6 && canStartJob()) {
      runQ[0] = stepFrom(b, s, e, w, g, constrain(spd, 650, 2500), MOVE_ACCEL_STEPS);
      jobDoneMsg = "✅ Move done";
      startJob(1, false);
    }
  }
  else if (line == "GO_HOME") {
    if (!canStartJob()) return;
    long safeElb = (calibElbowTotalSteps > 100) ? calibElbowTotalSteps / 2 : currentElbow;
    runQ[0] = stepFrom(currentBase, currentShoulder, safeElb, currentWrist, currentGripper, 1050, MOVE_ACCEL_STEPS);  // elbow to safe centre first
    runQ[1] = stepFrom(homeBase, homeShoulder, safeElb, currentWrist, currentGripper, 1050, MOVE_ACCEL_STEPS);       // then shoulder + base
    runQ[2] = stepFrom(homeBase, homeShoulder, homeElbow, homeWrist, homeGripper, 1050, MOVE_ACCEL_STEPS);           // then final elbow
    jobDoneMsg = "🏠 Arrived at home pose";
    startJob(3, false);
  }
}

char rxBuf[256]; int rxLen = 0;
void pollSerial() {                       // never blocks (no readStringUntil)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) { rxBuf[rxLen] = 0; rxLen = 0; processCommand(String(rxBuf)); }
    } else if (rxLen < (int)sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    else rxLen = 0;
  }
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.setRxBufferSize(512);
  Serial.setTxBufferSize(2048);            // must be called BEFORE begin(): big TX buffer = serial prints never block step timing
  Serial.begin(BAUD);
  pinMode(EN_PIN, OUTPUT); setMotorsPower(false);
  pinMode(BASE_STEP_PIN, OUTPUT);  pinMode(BASE_DIR_PIN, OUTPUT);
  pinMode(SH1_STEP_PIN, OUTPUT);   pinMode(SH1_DIR_PIN, OUTPUT);
  pinMode(SH2_STEP_PIN, OUTPUT);   pinMode(SH2_DIR_PIN, OUTPUT);
  pinMode(ELBOW_STEP_PIN, OUTPUT); pinMode(ELBOW_DIR_PIN, OUTPUT);
  loadConfig();

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  wristServo.setPeriodHertz(50); gripperServo.setPeriodHertz(50);
  wristServo.attach(WRIST_PIN, SERVO_MIN_US, SERVO_MAX_US);
  wristServo.write(currentWrist);
  delay(400);                               // stagger servo inrush current
  gripperServo.attach(GRIPPER_PIN, SERVO_MIN_US, SERVO_MAX_US);
  gripperServo.write(currentGripper);
  gripIdleSince = millis();                 // gripper detaches by itself after GRIP_DETACH_MS
  setStatus("Ready - press HOME after power-on");
}

void loop() {
  pollSerial();
  pollLimits();
  if (limitSignalFault && (homingActive() || jogAxis || mv.active || jobState != J_IDLE)) {
    // Motion is halted and homing validity cleared; do NOT remove holding
    // torque automatically. A real fault must be inspected before retrying.
    haltEverything();
    isHomed = false;
    setStatus("FAULT: conflicting limit inputs; motion stopped, motor holding power retained");
  }
  checkResync();
  homingService();
  jogService();
  moveService();
  jobService();
  servoService();

  unsigned long ms = millis();
  // 20 Hz telemetry (UI interpolates between frames). Skipped if the TX buffer is nearly full,
  // so a slow USB host can never stall the step generator.
  if (ms - lastTelemetryMs >= 50 && Serial.availableForWrite() > TEL_MIN_TX_FREE) {
    lastTelemetryMs = ms;
    sendTelemetry();
    const bool busy = mv.active || jogAxis || homingActive();
    if (statusDirty) sendStatus();                       // small message, always allowed
    else if (!busy) {                                    // heavy JSON only while the arm is idle
      if (cfgDirty) sendCfg();
      else if (wplDirty) sendWpl();
    }
  }
}
