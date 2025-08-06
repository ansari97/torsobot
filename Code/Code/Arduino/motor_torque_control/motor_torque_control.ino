#include <Arduino.h>
#include <Wire.h>
#include <cstdlib>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

// ——————————————————————————————————————————————————————————————————————————————
//  Pin definition
// ——————————————————————————————————————————————————————————————————————————————

// I2C lines
#define I2C_SDA 0
#define I2C_SCL 1

// I2C slave address
#define PICO_SLAVE_ADDRESS 0x30

// SPI lines for the IMU and the CAN controller
#define BNO08X_SCK 2
#define BNO08X_MOSI 3
#define BNO08X_MISO 4
#define BNO08X_CS 5
#define BNO08X_INT 6
#define BNO08X_RESET 7

#define MCP2517_SCK 10
#define MCP2517_MOSI 11
#define MCP2517_MISO 12
#define MCP2517_CS 13  // CS input of MCP2517
#define MCP2517_INT 9  // INT output of MCP2517

// Defining fast drv_mode runs the IMU at a higher frequency but causes values to drift
// #define BNO08X_FAST_MODE

// State LED
#define STOP_LED 19
#define OK_LED 18

// Euler angles for the IMU
struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool deegrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees);
void setReports(long report_interval);
float degToCycles(float);
float imuAngleCorrection(float, float, float, float);
float gravityMagnitude(float, float, float);
float degToRad(float);
float radToDeg(float);
float wrapTo360(float);
bool checkHeartbeatValidity(void);
void emergencyStop(void);

// ——————————————————————————————————————————————————————————————————————————————
//   Variable declarations
// ——————————————————————————————————————————————————————————————————————————————

// I2C speed; try changing it to 50kHz
const int I2C_BAUDRATE = 25 * 1000;  // I2C speed in Hz
int i2c_read_ctr = 0;                // reads the number of times mcu is read from

// heartbeat variables
const uint8_t HEARTBEAT_ID = 0XAA;
volatile uint8_t heartbeat_rcvd_ID;
volatile uint8_t heartbeat_ctr;
volatile uint8_t heartbeat_expected_checksum;
volatile uint8_t heartbeat_rcvd_checksum;
volatile uint8_t heartbeat_last_ctr;
volatile bool was_heartbeat_rcvd = false;
volatile bool is_heartbeat_valid = false;
volatile bool is_heartbeat_ctr_valid = false;
const int HEARTBEAT_FREQ = 1000 / 25;                      // hz; must be same as from pi
const int HEARTBEAT_TIMEOUT = 10 * 1000 / HEARTBEAT_FREQ;  // miliseconds
volatile uint64_t heartbeat_timer = 0;                     // = millis();
uint32_t millis_since_last_heartbeat = 0;

// I2C cmd for mcu data
const uint8_t DESIRED_TORSO_PITCH_CMD = 0x00;  // desired torso_pitch
const uint8_t IMU_PITCH_CMD = 0x01;            // torso_pitch
const uint8_t IMU_PITCH_RATE_CMD = 0x02;       // torso_pitch_rate
const uint8_t MOTOR_POS_CMD = 0x03;            // motor rotor position (not wheel)
const uint8_t MOTOR_VEL_CMD = 0x04;            // motor velocity position (not wheel)
const uint8_t MOTOR_TORQUE_CMD = 0X05;         // motor torque at motor shaft (not wheel)
const uint8_t MOTOR_DRV_MODE_CMD = 0X06;       // motor driver mode

const uint8_t MOTOR_MAX_TORQUE_CMD = 0X07;   // motor max torque value
const uint8_t KP_CMD = 0X08;                 // Kp value
const uint8_t KI_CMD = 0X09;                 // Ki value
const uint8_t KD_CMD = 0X0A;                 // Kd value
const uint8_t CTRL_MAX_INTEGRAL_CMD = 0X0B;  // max integral term

const uint8_t MOTOR_CMD_TORQUE_CMD = 0X0C;  // commanded motor torque


const int data_len = sizeof(float);  // 4 bytes
char desired_torso_pitch_data[data_len];
char imu_pitch_data[data_len];
char imu_pitch_rate_data[data_len];
char mot_pos_data[data_len];
char mot_vel_data[data_len];
char mot_torque_data[data_len];
char drv_mode_data[sizeof(int8_t)];

char mot_max_torque_data[data_len];
char kp_data[data_len];
char ki_data[data_len];
char kd_data[data_len];
char control_max_integral_data[data_len];

char cmd_torque_data[data_len];

// char write_buff[data_len + 10];
// char write_buff2[data_len + 10];

volatile char read_command;

int8_t drv_mode;
float imu_pitch;
float imu_pitch_rate;
float mot_vel;
float mot_pos;
float mot_torque;
float gravity_x = 0;
float gravity_y = 0;
float gravity_z = 0;

// Control parameters
const float control_freq = 50;  // control torque write freq

// ------------Received from PI---------------
volatile float desired_torso_pitch = 180.0;  // default desired torso angle in degrees

volatile float kp = 0.5;  // N.m per rad
volatile float ki = 0.0;  // N.m per (rad.s)
volatile float kd = 0.0;  // N.m per (rad/s)

// max motor torque
volatile float max_torque = 0.5;

// prevents integral windup
volatile float max_integral = 100;  // rad.s
// -------------------------------------------

float control_error, control_error_integral = 0;

float ff_torque, ff_torque_calc;  // torque command to the motor

uint64_t time_now = 0, time_last, time_since_last = 0;

// For the control signals
static uint32_t control_next_send_millis = 1000 / control_freq;  // initialized so that first control signal is NOT sent in the first loop iter
uint16_t control_loop_ctr = 0;

uint64_t start_time = 0, wait_time = 2 * 1000;  //milliseconds
uint64_t loop_count = 0;

// IMU struct for euler angles
euler_t ypr;

// ——————————————————————————————————————————————————————————————————————————————
//   BNO08X object
// ——————————————————————————————————————————————————————————————————————————————

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool resetOccurred = false;

// #ifdef BNO08X_FAST_MODE
// // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
// sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
// long reportIntervalUs = 2000;
// #else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
// #endif

// ——————————————————————————————————————————————————————————————————————————————
//   ACAN2517FD Driver object
// ——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can(MCP2517_CS, SPI1, MCP2517_INT);

Moteus moteus(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

Moteus::PositionMode::Command position_cmd;
Moteus::PositionMode::Format position_fmt;
Moteus::CurrentMode::Command current_cmd;
Moteus::CurrentMode::Format current_fmt;

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(STOP_LED, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(OK_LED, LOW);
  digitalWrite(STOP_LED, LOW);

  Serial.begin(115200);
  delay(100);

  Serial.println("Starting torsobot ...");

  // I2C lines
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);

  Wire.begin(PICO_SLAVE_ADDRESS);
  Wire.setClock(I2C_BAUDRATE);

  // set interrup functions for I2C
  Wire.onReceive(i2cRecv);
  Wire.onRequest(i2cReq);

  // Set SPI pins
  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);

  SPI1.setSCK(MCP2517_SCK);
  SPI1.setMOSI(MCP2517_MOSI);
  SPI1.setMISO(MCP2517_MISO);
  SPI1.setCS(MCP2517_CS);

  SPI1.begin();

  // bno spi communication
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println(F("Failed to find BNO08x chip"));
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  bno08x.hardwareReset();
  delay(100);
  Serial.print("Sensor reset?\t");
  Serial.println(bno08x.wasReset());

  setReports(reportIntervalUs);

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  ACAN2517FDSettings settings(
    ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

  // The atmega32u4 on the CANbed has only a tiny amount of memory.
  // The ACAN2517FD driver needs custom settings so as to not exhaust
  // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;

  const uint32_t errorCode = can.begin(settings, [] {
    can.isr();
  });

  if (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(100);
    return;
  }

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus.SetStop();
  Serial.println("motor stopped");

  // position_fmt.velocity_limit = Moteus::kFloat;
  // position_fmt.accel_limit = Moteus::kFloat;

  // position_cmd.velocity_limit = 2.0;
  // position_cmd.accel_limit = 3.0;

  // Serial.println("wait...");
  // delay(5000);

  position_fmt.kp_scale = Moteus::kFloat;
  position_fmt.kd_scale = Moteus::kFloat;
  position_fmt.feedforward_torque = Moteus::kFloat;
  // position_fmt.velocity = Moteus::kIgnore;

  position_cmd.position = NaN;
  position_cmd.velocity = 0.0f;  // Not required as resolution is set to ignore
  position_cmd.kp_scale = 0.0f;
  position_cmd.kd_scale = 0.0f;
  position_cmd.maximum_torque = max_torque;

  // angle changed to radians
  desired_torso_pitch = degToRad(desired_torso_pitch);  // radians

  // Copy to char buffers
  memcpy(desired_torso_pitch_data, const_cast<float *>(&desired_torso_pitch), sizeof(desired_torso_pitch));
  memcpy(mot_max_torque_data, const_cast<float *>(&max_torque), sizeof(max_torque));
  memcpy(kp_data, const_cast<float *>(&kp), sizeof(kp));
  memcpy(ki_data, const_cast<float *>(&ki), sizeof(ki));
  memcpy(kd_data, const_cast<float *>(&kd), sizeof(kd));

  Serial.println("Waiting for heartbeat from the PI...");

  // waits for heatrbeat
  while (!was_heartbeat_rcvd) {
    digitalWrite(STOP_LED, HIGH);
  }  // Serial.println(was_heartbeat_rcvd); }

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(OK_LED, LOW);

  // save the last counter value
  heartbeat_last_ctr = heartbeat_ctr;
  was_heartbeat_rcvd = false;

  Serial.println("Heartbeat received from PI!");
  Serial.println("Setup complete!");
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(OK_LED, HIGH);
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void loop() {

  // checks then increments
  if (loop_count++ == 0) {
    start_time = millis();  // stores start time of the loop function
    Serial.print("loop start time:  ");
    Serial.println(start_time);
  }

  // is_heartbeat_valid = checkHeartbeatValidity();
  millis_since_last_heartbeat = millis() - heartbeat_timer;  // heartbeat_timer is updated by the i2cRecv function if heartbeat is valid

  // checks for 1) heartbeat validity (checksum an expected counter value) and 2) time since last heartbeat received
  if (!is_heartbeat_valid || (millis_since_last_heartbeat > HEARTBEAT_TIMEOUT)) {
    Serial.print("millis_since_last_heartbeat: ");
    Serial.println(millis_since_last_heartbeat);

    Serial.print("is_heartbeat_valid?: ");
    Serial.println(is_heartbeat_valid);

    Serial.println("heartbeat not valid or heartbeat timeout reached!");

    emergencyStop();  // call the stop routine
  }

  Serial.println(desired_torso_pitch);
  Serial.println(kp);
  Serial.println(ki);
  Serial.println(kd);
  Serial.println(max_torque);
  Serial.println(max_integral);
  Serial.println("---");


  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset during loop!");
    setReports(reportIntervalUs);
    // resetOccurred = true;
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on BNO08X_FAST_MODE define (above)

    // Serial.print("Sensor ID:");
    // Serial.println(sensorValue.sensorId);

    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        imu_pitch = degToRad(imuAngleCorrection(ypr.pitch, gravity_x, gravity_y, gravity_z));
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        imu_pitch_rate = sensorValue.un.gyroscope.z;
        break;

      case SH2_GRAVITY:
        gravity_x = sensorValue.un.gravity.x;
        gravity_y = sensorValue.un.gravity.y;
        gravity_z = sensorValue.un.gravity.z;
        break;
    }
    //   case SH2_GYRO_INTEGRATED_RV:
    //     // faster (more noise?)
    //     quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
    //     break;
    // }

    // imu_pitch_rate = sensorValue.un.gyroscope.z;
    // Serial.print(imu_pitch, 4);
    // Serial.print(", ");

    // imu_pitch = imuAngleCorrection(ypr.pitch);
    // Serial.print(imu_pitch_rate, 4);
    // Serial.print(", ");

    // Serial.println(gravity_y, 4);

    memcpy(imu_pitch_data, &imu_pitch, sizeof(float));
    memcpy(imu_pitch_rate_data, &imu_pitch_rate, sizeof(float));
  }
  // // Serial.println();

  // **************************************************************
  // Controls part

  // We intend to send control frames every 1000/control_freq milliseconds; second condition allows imu values to stabilise at program start
  if (control_next_send_millis >= millis() || (millis() - start_time) < wait_time) {
    return;
  }

  // Fault drv_mode
  if (static_cast<int>(moteus.last_result().values.mode) == 11) {
    Serial.println("motor driver fault");
    moteus.SetStop();
  }

  time_last = time_now;  // store value from previous control loop's timer

  // If control_loop_ctr is 0 (first loop)
  if (!control_loop_ctr) { time_last = millis(); }

  time_now = millis();  // get current time
  time_since_last = time_now - time_last;

  // Calculate torque required
  control_error = desired_torso_pitch - imu_pitch;  // radians

  // Integral term
  control_error_integral += control_error * time_since_last;
  // clamp the integral to prevent integral windup
  control_error_integral = min(max_integral, max(-max_integral, control_error_integral));

  // time derivative of control error = -imu_pitch_rate ; because desired_pitch_rate is constant

  ff_torque = kp * control_error + ki * control_error_integral + kd * (-imu_pitch_rate);
  ff_torque = min(max_torque, max(-max_torque, ff_torque));  // second safety net; max torque has already been defined for the board but this line ensures no value greater than max is written

  // ff_torque = min(abs(max_torque), abs(ff_torque_calc)); // always +ve; second safety net; max torque has already been defined for the board but this line ensures no value greater than max is written

  // // retain sign of ff_torque
  // if (ff_torque_calc <= 0)
  // {
  //   ff_torque = -ff_torque;
  // }

  // write to cmd
  position_cmd.feedforward_torque = ff_torque;

  // **Write to the motor
  // Serial.println(millis());
  moteus.SetPosition(position_cmd, &position_fmt);
  // Serial.println(millis());

  control_next_send_millis += 1000 / control_freq;
  control_loop_ctr++;
  // **************************************************************


  // **************************************************************
  // copy data from driver to buffers

  // Get values every 5 loop iters
  if (control_loop_ctr % 5 != 0) {
    return;
  }
  // digitalWrite(LED_BUILTIN, HIGH);

  // print_moteus(moteus.last_result().values);
  drv_mode = static_cast<int8_t>(moteus.last_result().values.mode);
  mot_vel = moteus.last_result().values.velocity;
  mot_pos = moteus.last_result().values.position;
  mot_torque = moteus.last_result().values.torque;

  // Serial.print("millis_since_last_heartbeat: ");
  // Serial.println(millis_since_last_heartbeat);
  // Serial.print("drv_mode: ");
  // Serial.print(drv_mode);
  // Serial.print("\tcommand_torque: ");
  // Serial.print(ff_torque);
  // Serial.print("\tmot_torque: ");
  // Serial.print(mot_torque);
  // Serial.print("\tmot_pos: ");
  // Serial.print(mot_pos);
  // Serial.print("\tmot_vel: ");
  // Serial.println(mot_vel);

  memcpy(cmd_torque_data, &ff_torque, sizeof(float));
  memcpy(mot_pos_data, &mot_pos, sizeof(float));
  memcpy(mot_vel_data, &mot_vel, sizeof(float));
  memcpy(mot_torque_data, &mot_torque, sizeof(float));
  memcpy(drv_mode_data, &drv_mode, sizeof(int8_t));

  // **************************************************************

  // Delay function messes with the IMU data and causes buffers to overflow(?);
  // bno085 goes into reset state and needs a hardwareReset() call; hardwareReset() doesn't always work
  // delay(5);
}

///////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////
// Called when the I2C slave gets written to
void i2cRecv(int len) {
  read_command = Wire.read();  // or the ID

  int remaining_bytes = len - 1;

  if (static_cast<uint8_t>(read_command) == HEARTBEAT_ID) {
    if (len == 3) {
      heartbeat_rcvd_ID = read_command;
      heartbeat_ctr = Wire.read();            // read second bit
      heartbeat_rcvd_checksum = Wire.read();  // read third bit
      was_heartbeat_rcvd = true;

      heartbeat_expected_checksum = heartbeat_ctr ^ heartbeat_rcvd_ID;

      if ((heartbeat_expected_checksum == heartbeat_rcvd_checksum))  // && (heartbeat_ctr == (heartbeat_last_ctr + 1) % 256)) {
      {
        is_heartbeat_valid = true;

        if (heartbeat_ctr == (heartbeat_last_ctr + 1) % 256) {
          is_heartbeat_ctr_valid = true;
          heartbeat_last_ctr = heartbeat_ctr;
          heartbeat_timer = millis();
        }
      } else {
        is_heartbeat_valid = false;
      }
    }
  }
  // else if it is the desired torso angle
  else if (static_cast<uint8_t>(read_command) == DESIRED_TORSO_PITCH_CMD) {
    if (len == 5) {
      for (int i = 0; i < remaining_bytes; i++) { desired_torso_pitch_data[i] = Wire.read(); }
      float temp_val;
      memcpy(&temp_val, desired_torso_pitch_data, sizeof(float));

      desired_torso_pitch = temp_val;
    }
  }
  // else if it is the kp
  else if (static_cast<uint8_t>(read_command) == KP_CMD) {
    if (len == 5) {
      for (int i = 0; i < remaining_bytes; i++) { kp_data[i] = Wire.read(); }
      float temp_val;
      memcpy(&temp_val, kp_data, sizeof(float));

      kp = temp_val;
    }
  }
  // else if it is the ki
  else if (static_cast<uint8_t>(read_command) == KI_CMD) {
    if (len == 5) {
      for (int i = 0; i < remaining_bytes; i++) { ki_data[i] = Wire.read(); }
      float temp_val;
      memcpy(&temp_val, ki_data, sizeof(float));

      ki = temp_val;
    }
  }
  // else if it is the kd
  else if (static_cast<uint8_t>(read_command) == KD_CMD) {
    if (len == 5) {
      for (int i = 0; i < remaining_bytes; i++) { kd_data[i] = Wire.read(); }
      float temp_val;
      memcpy(&temp_val, kd_data, sizeof(float));

      kd = temp_val;
    }
  }
  // else if it is the motor max torque
  else if (static_cast<uint8_t>(read_command) == MOTOR_MAX_TORQUE_CMD) {
    if (len == 5) {
      for (int i = 0; i < remaining_bytes; i++) { mot_max_torque_data[i] = Wire.read(); }
      float temp_val;
      memcpy(&temp_val, mot_max_torque_data, sizeof(float));

      max_torque = temp_val;
    }
  }
  // else if it is the control max integral torque
  else if (static_cast<uint8_t>(read_command) == CTRL_MAX_INTEGRAL_CMD) {
    if (len == 5) {
      for (int i = 0; i < remaining_bytes; i++) { control_max_integral_data[i] = Wire.read(); }
      float temp_val;
      memcpy(&temp_val, control_max_integral_data, sizeof(float));

      max_integral = temp_val;
    }
  }
  // ensures that if there are remaining bytes, they are read so that the buffer is emptied
  else {
    while (remaining_bytes--) {
      (void)Wire.read();
    }
  }
}

// Called when the I2C slave is read from
void i2cReq() {
  i2c_read_ctr++;
  switch (read_command) {
    case DESIRED_TORSO_PITCH_CMD:
      // do nothing; since we're now writing this value to the pico
      // Wire.write(desired_torso_pitch_data, sizeof(desired_torso_pitch_data));
      break;
    case IMU_PITCH_CMD:
      Wire.write(imu_pitch_data, sizeof(imu_pitch_data));
      break;
    case IMU_PITCH_RATE_CMD:
      Wire.write(imu_pitch_rate_data, sizeof(imu_pitch_rate_data));
      break;
    case MOTOR_POS_CMD:
      Wire.write(mot_pos_data, sizeof(mot_pos_data));
      break;
    case MOTOR_VEL_CMD:
      Wire.write(mot_vel_data, sizeof(mot_vel_data));
      break;
    case MOTOR_TORQUE_CMD:
      Wire.write(mot_torque_data, sizeof(mot_torque_data));
      break;
    case MOTOR_DRV_MODE_CMD:
      Wire.write(drv_mode_data, sizeof(drv_mode_data));
      break;
    case MOTOR_MAX_TORQUE_CMD:
      Wire.write(mot_max_torque_data, sizeof(mot_max_torque_data));
      break;
    case KP_CMD:
      // Wire.write(kp_data, sizeof(kp_data));
      break;
    case KI_CMD:
      // Wire.write(ki_data, sizeof(ki_data));
      break;
    case KD_CMD:
      // Wire.write(kd_data, sizeof(kd_data));
      break;
    case CTRL_MAX_INTEGRAL_CMD:
      // Wire.write(kd_data, sizeof(kd_data));
      break;
    case MOTOR_CMD_TORQUE_CMD:
      Wire.write(cmd_torque_data, sizeof(cmd_torque_data));
      break;
  }
}

void setReports(long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_GRAVITY, report_interval)) {
    Serial.println("Could not enable gravity vector");
  }
  delay(100);
}

// user-defined functions
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float degToCycles(float deg) {
  return deg / 360;
}

float degToRad(float deg) {
  return deg / 180 * PI;
}

float radToDeg(float rad) {
  return rad * 180 / PI;
}

// wrap angle from 0 to 360
float wrapTo360(float angle) {

  if (angle < 0) { angle += 360; }
  return angle;
}

float imuAngleCorrection(float pitch, float gravity_x, float gravity_y, float gravity_z) {
  pitch = 90 - pitch;  // horizontal is originally zero

  // pitch is always positive so no need to take abs

  // Serial.println(gravity_x);
  // Serial.println(gravity_y);
  // Serial.println(gravity_z);
  // Serial.println(gravityMagnitude(gravity_x, gravity_y, gravity_z));
  // Serial.println(atan2(gravity_y, gravity_x));

  // Evaluate angle from gravity vector when close to vertical
  // if (abs(gravity_y) < 1.0) {
  //   pitch = 180 - atan2(gravity_y, gravity_x);
  //   // Serial.println(pitch);
  // }

  // Change sign of angle based on gravity vector's j component
  if (gravity_y < 0)
    pitch = -abs(pitch);

  pitch = wrapTo360(pitch);

  // Serial.println(pitch);
  return pitch;
}

float gravityMagnitude(float gravity_x, float gravity_y, float gravity_z) {
  return sqrt(pow(gravity_x, 2) + pow(gravity_y, 2) + pow(gravity_z, 2));
}

// not used; checks heartbeat validity
bool checkHeartbeatValidity(void) {
  Serial.print("rcvd heartbeat: ");
  Serial.print("ID = 0x");
  Serial.print(heartbeat_rcvd_ID);
  Serial.print(" Counter = 0x");
  Serial.print(heartbeat_ctr);
  Serial.print(" Checksum = 0x");
  Serial.print(heartbeat_rcvd_checksum);

  // if (!was_heartbeat_rcvd) {
  //   return false;
  // } else was_heartbeat_rcvd = false;

  // Evaluate expected checksum
  heartbeat_expected_checksum = heartbeat_ctr ^ heartbeat_rcvd_ID;

  Serial.print(" Expected Checksum = 0x");
  Serial.print(heartbeat_expected_checksum);
  Serial.println("\t");

  if ((heartbeat_expected_checksum == heartbeat_rcvd_checksum))  // && (heartbeat_ctr == (heartbeat_last_ctr + 1) % 256)) {
  {
    return true;
  }

  return false;
}

// called when program has to stop
void emergencyStop(void) {
  Serial.println("Stopping program! :(");
  Serial.println(is_heartbeat_valid);
  Serial.println(millis_since_last_heartbeat);
  Serial.print(heartbeat_last_ctr);
  Serial.print("\t");
  Serial.println(heartbeat_ctr);
  // Serial.print("\t");
  moteus.SetStop();
  Serial.println("motor stopped");
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(OK_LED, LOW);
  digitalWrite(STOP_LED, HIGH);

  delay(500);
  Serial.println("Entering infinite while loop!");

  while (true) {
    // infinite while loop
  }
}
