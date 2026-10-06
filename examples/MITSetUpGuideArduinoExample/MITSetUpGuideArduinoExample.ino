#include <gl-ii-CAN.h>
#include <math.h>
using namespace glii;

// Board must already be configured for MIT mode (control mode 0).
// These examples do not change its saved mode or encoding ranges.

float p; // MIT position
float v; // MIT velocity
float t; // MIT torque
float kp;    // MIT kd.
float kd;    // MIT kd.

constexpr uint32_t kCommandPeriodMs = 10; // 100 Hz
constexpr uint32_t kFeedbackTimeoutMs = 250;
constexpr uint32_t kEnableTimeoutMs = 1000;


Config makeConfig() {
  Config c;
  c.txPin = GPIO_NUM_17; // CoreS3; Tab5 routing may require GPIO 6/7.
  c.rxPin = GPIO_NUM_18; // MUST match motor configuration; not automatically detected.
  c.bitrate = 1000000; // MUST match motor configuration; not automatically detected.
  c.motorId = 1; // MUST match motor configuration; not automatically detected.
  c.feedbackId = 0; // Confirmed standard reply ID for this board.
  c.feedbackExtended = false;
  c.positionRange = 12.5f;
  c.velocityRange = 30.0f; // MUST match motor configuration; not automatically detected.
  c.torqueRange = 10.0f;
  return c;
}
GLIICAN motor(makeConfig()); // call to config the motor

// data recieved from motor
struct UserData {
  MotorFeedback feedback;
  uint32_t receivedAtMs = 0;
  bool received = false;
};

UserData motorData;

// state opf the motor
enum class State { Stopped, Enabling, Running };
State state = State::Stopped; // motor is stopped initially 
uint32_t enableAtMs = 0, runAtMs = 0, lastCommandMs = 0, lastPositionCommandMs = 0;
bool stopPending = false;
uint32_t stopAtMs = 0;
float requestedPosition = 0.0f, requestedVelocity = 0.0f, requestedTorque = 0.0f;
enum class Mode {position, velocity, torque}; // control modes within mit mode
Mode mode = Mode::position; // start in position mode

// call back for when we get feedback from the motor
void onFeedback(const MotorFeedback& feedback, void* context) {
  auto* data = static_cast<UserData*>(context);
  data->feedback = feedback;
  data->receivedAtMs = millis();
  data->received = true;
}
void stopMotor(const char* reason) {
  state = State::Stopped;
  requestedPosition = requestedVelocity = requestedTorque = 0.0f;
  motor.mit(0, 0, 0, 0, 0);
  motor.exitMotorControlMode();
  // Retry briefly if an earlier disable was lost/queue-full. A broken CAN link
  // cannot guarantee a stop; configure the board's own communication watchdog.
  stopPending = true;
  stopAtMs = millis();
  Serial.println(reason);
}

// serial commands to start, stop, zero, and clear errors on the motor
void handleConsole() {
  while (Serial.available()) {
    char command = Serial.read();
    if (command == 'x') stopMotor("Stop requested");
    if (command == 'e' && state == State::Stopped) {
      stopPending = false;
      // Discard software feedback from before this explicit enable request.
      motorData.received = false;
      if (!motor.mit(0, 0, 0, 0, 0) || !motor.enterMotorControlMode()) {
        stopMotor("Enable enqueue failed");
      } else {
        enableAtMs = millis(); state = State::Enabling;
        Serial.println("Enable requested; waiting for fresh enabled feedback...");
      }
    } if (command == 'z'){
      motor.setCurrentPositionToZero();
    } if (command == 'c'){
      motor.clearErrors();
    } if (command == 'p'){
      mode = Mode::position;
      motor.setCurrentPositionToZero(); // zero the motor
      p = 0;
      v = requestedVelocity = 0;
      kp = .123;
      kd = .005;
      t = 0;
      Serial.println("Position control, advance 90 degrees every two seconds");
    } if (command == 'v'){
      mode = Mode::velocity;
      p = 0;
      v = requestedVelocity = 30;
      kp = 0;
      kd = 0.005;
      t = 0;
      Serial.println("Velocity control, spin at 30 rad/s");
    } if (command == 't'){
      mode = Mode::torque;
      p = 0;
      v = requestedVelocity = 0;
      kp = 0;
      kd = 0.0;
      t = 0.03;
      Serial.println("Torque control, 0.03Nm");
    }
  }
}
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100); // wait 3 seconds for a serial connection
  motor.onFeedback(onFeedback, &motorData); // feedback callback
  if (!motor.begin()) { // start motor
    Serial.printf("CAN failed: %s\n", esp_err_to_name(motor.lastError()));
    while (true) delay(100);
  }
  Serial.println("Motor set up: e=start, x=stop. Motor must be in MIT mode.");
}
void loop() {
  motor.update();
  handleConsole(); // check for serial commands
  const uint32_t now = millis();
  const bool fresh = motorData.received && uint32_t(now - motorData.receivedAtMs) <= kFeedbackTimeoutMs; // are we getting fresh feedback from the motor?
  
  // stop motor recieve an error from the motor stop command prints error
  if (state != State::Stopped && fresh && motorData.feedback.status >= 8)
    stopMotor(motorStatusName(motorData.feedback.status)); 
  if (state == State::Enabling) {
    if (fresh && motorData.feedback.status == 1) { // if we should be enabled and we have feedback let it rip
      state = State::Running; runAtMs = now;
      Serial.println("Running");
    } else if (uint32_t(now - enableAtMs) >= kEnableTimeoutMs) { 
      stopMotor("Enable timed out: check reply ID, bitrate, mode and fault status");
    }
  }
  if (state == State::Running && (!fresh || motorData.feedback.status != 1))
    stopMotor("Stopped: feedback stale or motor no longer enabled");

  if (uint32_t(now - lastCommandMs) >= kCommandPeriodMs) {  // send commands at 10 Hz
    lastCommandMs = now;
    if (state == State::Enabling) {
      // Solicit feedback on boards that respond only to command packets.
      if (!motor.mit(0, 0, 0, 0, 0)) stopMotor("Feedback probe enqueue failed");
    } else if (state == State::Running) { // sin wave commands
      if ((mode == Mode::position) && ((now - lastPositionCommandMs) >= 2000)) { // in position mode update the position ever7 2 second
      p += PI / 2; 
      lastPositionCommandMs = now;
      }
      const bool queued = motor.mit(p, v, kp, kd, t); // send mit command
      if (!queued) stopMotor("Command enqueue failed");
    } else if (stopPending) {
      motor.mit(0, 0, 0, 0, 0);
      motor.exitMotorControlMode();
      if (uint32_t(now - stopAtMs) >= 1000) stopPending = false;
    }
  }
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK &&
      (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED)) && state != State::Stopped)
    stopMotor("Stopped: CAN transmission fault");
  static uint32_t lastPrintMs = 0;
  if (uint32_t(now - lastPrintMs) >= 500) {
    lastPrintMs = now;
    if (fresh) Serial.printf("%s targetPos=%.3f targetVel=%.3f targetTorque=%.3f Nm | measuredPos=%.3f measuredVel=%.3f torque=%.3f Nm | driverTemp=%.3f C motorTemp =%.3f C\n",
        motorStatusName(motorData.feedback.status), requestedPosition, requestedVelocity, requestedTorque,
        motorData.feedback.positionRad, motorData.feedback.velocity, motorData.feedback.torqueNm, motorData.feedback.driverTemperatureC, motorData.feedback.motorTemperatureC);
    else Serial.println("No fresh motor feedback (e=start, x=stop)");
  }
  delay(1);
}
