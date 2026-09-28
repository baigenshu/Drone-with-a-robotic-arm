// =============================================================================
//  gimbal_device_v2.ino  --  minimal MAVLink gimbal-protocol-v2 "device" side
// =============================================================================
//  This is the REAL-HARDWARE counterpart of the Gazebo plugin
//  `libgazebo_gimbal_controller_plugin.so` used in the simulation.
//
//  In SITL:
//      PX4 gimbal module  -->  MAVLink gimbal v2  -->  Gazebo plugin  -->  joints
//  On a real drone replace the Gazebo plugin with THIS firmware:
//      PX4 gimbal module  -->  MAVLink gimbal v2  -->  this MCU       -->  servos
//
//  Wiring: one spare UART of the flight controller (or a UDP/CAN MAVLink link)
//  connected to this MCU. PX4 side:
//      MNT_MODE_OUT = 2                 (MAVLink gimbal protocol v2)
//      MNT_MODE_IN  = 3  (or 0/4)       (how commands reach the autopilot)
//      MAV_x_CONFIG / MAV_x_BAUD        route a MAVLink instance to this UART
//  and start the instance in gimbal mode, e.g.:
//      mavlink start -d /dev/ttyS2 -b 115200 -m gimbal
//
//  Reference: https://mavlink.io/en/services/gimbal_v2.html
//
//  NOTE: this is a *reference skeleton*. Fill in your own servo pins,
//  kinematics and safety handling before using it on hardware.
// =============================================================================

#include <Servo.h>          // Arduino; on STM32 use the HAL timer/PWM API instead
#include <math.h>

// ---- MAVLink C library -----------------------------------------------------
// Get it from PX4 (src/modules/mavlink/mavlink) or
// https://github.com/mavlink/c_library_v2 , with MAVLINK_DIALECT=common
#include <mavlink.h>

// ---- identity --------------------------------------------------------------
static const uint8_t  SYSID  = 1;                 // same system as the autopilot
static const uint8_t  COMPID = MAV_COMP_ID_GIMBAL; // 154, learned by PX4
static const uint8_t  CHAN   = MAVLINK_COMM_0;

// ---- joints ----------------------------------------------------------------
// Order matches the sim: yaw (vertical arm), roll (horizontal arm), pitch (gripper)
enum { J_YAW = 0, J_ROLL = 1, J_PITCH = 2, N_JOINTS = 3 };

static const uint8_t SERVO_PIN[N_JOINTS] = { 3, 5, 6 };       // <-- your pins
// software limits in degrees (keep inside the mechanical limits!)
static const float LIM_MIN_DEG[N_JOINTS] = { -90.0f, -45.0f, -60.0f };
static const float LIM_MAX_DEG[N_JOINTS] = {  90.0f,  45.0f, 120.0f };
// servo pulse range for min..max angle
static const int   SERVO_US_MIN = 1000;
static const int   SERVO_US_MAX = 2000;
// max joint speed for the slew limiter (deg/s) -- protects the mechanics
static const float MAX_RATE_DPS = 60.0f;

Servo servo[N_JOINTS];
float targetDeg[N_JOINTS] = { 0, 0, 0 };   // what the autopilot asked for
float actualDeg[N_JOINTS] = { 0, 0, 0 };   // what we command now
float vehicleQ[4]         = { 1, 0, 0, 0 }; // AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
bool  neutral             = false;

// ---------------------------------------------------------------------------
//  helpers
// ---------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// quaternion (w,x,y,z) -> roll/pitch/yaw in radians (ZYX / yaw-pitch-roll order)
static void quatToRPY(const float q[4], float* roll, float* pitch, float* yaw) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  *roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  *pitch = asinf(clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
  *yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

static void writeServo(int j) {
  const float span = LIM_MAX_DEG[j] - LIM_MIN_DEG[j];
  const float t    = (actualDeg[j] - LIM_MIN_DEG[j]) / span;   // 0..1
  const int   us   = SERVO_US_MIN + (int)(t * (SERVO_US_MAX - SERVO_US_MIN));
  servo[j].writeMicroseconds(us);
}

// ---------------------------------------------------------------------------
//  MAVLink: outgoing
// ---------------------------------------------------------------------------
static void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack_chan(
      SYSID, COMPID, CHAN, &msg,
      MAV_TYPE_GIMBAL, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

static void sendGimbalDeviceInformation() {
  mavlink_message_t msg;
  const uint16_t capFlags =
      GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL |
      GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS |
      GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
      GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
      GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW;

  mavlink_msg_gimbal_device_information_pack_chan(
      SYSID, COMPID, CHAN, &msg,
      millis(),
      "DIY", "Aerial manipulator", "",
      0, 1, 0x2f0b7a92295a7b3cULL,
      capFlags, 0,
      LIM_MIN_DEG[J_ROLL]  * (float)M_PI / 180.0f,   // radians!
      LIM_MAX_DEG[J_ROLL]  * (float)M_PI / 180.0f,
      LIM_MIN_DEG[J_PITCH] * (float)M_PI / 180.0f,
      LIM_MAX_DEG[J_PITCH] * (float)M_PI / 180.0f,
      LIM_MIN_DEG[J_YAW]   * (float)M_PI / 180.0f,
      LIM_MAX_DEG[J_YAW]   * (float)M_PI / 180.0f,
      0 /* gimbal_device_id */, 0 /* cap_flags2 */);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

static void sendAttitudeStatus() {
  float roll, pitch, yaw;
  // report our current commanded angles as the device attitude
  const float q[4] = { 1, 0, 0, 0 };   // TODO: use a real estimator / joint feedback
  (void)roll; (void)pitch; (void)yaw;
  mavlink_message_t msg;
  mavlink_msg_gimbal_device_attitude_status_pack_chan(
      SYSID, COMPID, CHAN, &msg,
      0, 0, millis(),
      GIMBAL_DEVICE_FLAGS_HAS_ROLL | GIMBAL_DEVICE_FLAGS_HAS_PITCH |
      GIMBAL_DEVICE_FLAGS_HAS_YAW,
      q, NAN, NAN, NAN, 0 /* failure_flags */, NAN, NAN, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

static void sendCommandAck(uint8_t targetSys, uint8_t targetComp,
                           uint16_t command, uint8_t result) {
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack_chan(
      SYSID, COMPID, CHAN, &msg,
      command, result, 255 /* progress */, 0 /* result_param2 */, targetSys, targetComp);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

// ---------------------------------------------------------------------------
//  MAVLink: incoming
// ---------------------------------------------------------------------------
static void handleSetAttitude(const mavlink_message_t& msg) {
  mavlink_gimbal_device_set_attitude_t sa;
  mavlink_msg_gimbal_device_set_attitude_decode(&msg, &sa);

  if (sa.flags & GIMBAL_DEVICE_FLAGS_NEUTRAL) {      // park at 0
    neutral = true;
    for (int j = 0; j < N_JOINTS; ++j) targetDeg[j] = 0.0f;
    return;
  }
  neutral = false;

  float roll, pitch, yaw;
  quatToRPY(sa.q, &roll, &pitch, &yaw);

  // If GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, yaw is earth-referenced and must be
  // compensated with the vehicle yaw (see handleAutopilotState()). Here we keep
  // it simple: body-frame mapping, exactly like the simulation.
  // (void)(sa.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK);

  targetDeg[J_YAW]   = clampf(yaw   * 180.0f / (float)M_PI, LIM_MIN_DEG[J_YAW],   LIM_MAX_DEG[J_YAW]);
  targetDeg[J_ROLL]  = clampf(roll  * 180.0f / (float)M_PI, LIM_MIN_DEG[J_ROLL],  LIM_MAX_DEG[J_ROLL]);
  targetDeg[J_PITCH] = clampf(pitch * 180.0f / (float)M_PI, LIM_MIN_DEG[J_PITCH], LIM_MAX_DEG[J_PITCH]);
}

static void handleAutopilotState(const mavlink_message_t& msg) {
  mavlink_autopilot_state_for_gimbal_device_t st;
  mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &st);
  // vehicle attitude, useful if you implement earth-frame stabilization
  vehicleQ[0] = st.q[0]; vehicleQ[1] = st.q[1];
  vehicleQ[2] = st.q[2]; vehicleQ[3] = st.q[3];
}

static void handleCommandLong(const mavlink_message_t& msg) {
  mavlink_command_long_t cmd;
  mavlink_msg_command_long_decode(&msg, &cmd);
  if (cmd.target_system != SYSID && cmd.target_system != 0) return;
  if (cmd.target_component != COMPID && cmd.target_component != 0) return;

  switch (cmd.command) {
    case MAV_CMD_REQUEST_MESSAGE:
      if ((uint16_t)cmd.param1 == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION) {
        sendGimbalDeviceInformation();
        sendCommandAck(msg.sysid, msg.compid, cmd.command, MAV_RESULT_ACCEPTED);
      } else {
        sendCommandAck(msg.sysid, msg.compid, cmd.command, MAV_RESULT_UNSUPPORTED);
      }
      break;
    default:
      sendCommandAck(msg.sysid, msg.compid, cmd.command, MAV_RESULT_UNSUPPORTED);
      break;
  }
}

static void handleMessage(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE:          handleSetAttitude(msg);      break;
    case MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE:   handleAutopilotState(msg);  break;
    case MAVLINK_MSG_ID_COMMAND_LONG:                        handleCommandLong(msg);     break;
    default: break;
  }
}

static void pollMavlink() {
  static mavlink_message_t msg;
  static mavlink_status_t  status;
  while (Serial.available() > 0) {
    uint8_t c = (uint8_t)Serial.read();
    if (mavlink_parse_char(CHAN, c, &msg, &status)) {
      handleMessage(msg);
    }
  }
}

// ---------------------------------------------------------------------------
//  Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);              // link to the flight controller
  for (int j = 0; j < N_JOINTS; ++j) {
    servo[j].attach(SERVO_PIN[j]);
    writeServo(j);
  }
  delay(200);
  sendHeartbeat();
  sendGimbalDeviceInformation();
}

void loop() {
  static uint32_t tHeartbeat = 0, tInfo = 0, tStatus = 0, tServo = 0;
  const uint32_t now = millis();

  pollMavlink();

  // 1 Hz heartbeat, 1 Hz device information (in case PX4 missed the first one),
  // 10 Hz attitude status
  if (now - tHeartbeat >= 1000) { tHeartbeat = now; sendHeartbeat(); }
  if (now - tInfo      >= 1000) { tInfo      = now; sendGimbalDeviceInformation(); }
  if (now - tStatus    >= 100)  { tStatus    = now; sendAttitudeStatus(); }

  // 50 Hz slew-rate limited servo update
  if (now - tServo >= 20) {
    const float dt   = (now - tServo) / 1000.0f;
    const float step = MAX_RATE_DPS * dt;
    tServo = now;
    for (int j = 0; j < N_JOINTS; ++j) {
      const float d = targetDeg[j] - actualDeg[j];
      actualDeg[j] += (fabsf(d) <= step) ? d : copysignf(step, d);
      writeServo(j);
    }
  }
}
