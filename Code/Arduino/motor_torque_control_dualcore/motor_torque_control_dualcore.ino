// Includes
#include <Arduino.h>
#include "PicoEncoder.h"
#include <Wire.h>
#include <cstdlib>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <ACAN2517FD.h>
#include <Moteus.h>
#include <cmath>
#include "pico/mutex.h"  // for dual core
#include "shared_messages.h"

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

#define MCP2517_INT 9  // INT output of MCP2517
#define MCP2517_SCK 10
#define MCP2517_MOSI 11
#define MCP2517_MISO 12
#define MCP2517_CS 13  // CS input of MCP2517

// Defining fast mot_drv_mode runs the IMU at a higher frequency but causes values to drift
// #define BNO08X_FAST_MODE

// driveshaft encoder
// must be consecutive pins on the pico
#define encoder_pinA 15
#define encoder_pinB 16

// State LED
#define OK_LED 18
#define STOP_LED 19

// Reed switch
#define REED_SWITCH 20

// ——————————————————————————————————————————————————————————————————————————————
//  Variables definition
// ——————————————————————————————————————————————————————————————————————————————

// Euler angles for the IMU
struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

// function prototypes
void quaternionToPitch(sh2_RotationVector_t *, float *, bool degrees = false);
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool deegrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees);
void setReports(long report_interval);
void i2cReq(void);
void i2cRecv(int);
void emergencyStop(void);

int signof(float var);

float imuAngleCorrection(float pitch, float gravity_x, float gravity_y, float gravity_z);
float gravityMagnitude(float gravity_x, float gravity_y, float gravity_z);
float degToRevs(float deg);
float revsToRad(float rev);
float degToRad(float deg);
float radToDeg(float rad);
float wrapTo2PI(float ang);
float encoderCountsToRad(int32_t local_signed_encoder_steps);
float encoderCountsToDeg(int32_t local_signed_encoder_steps);

bool checkHeartbeatValidity(void);

// ——————————————————————————————————————————————————————————————————————————————
//   Variable declarations
// ——————————————————————————————————————————————————————————————————————————————

// i2c structs
mcuConfigData config_data_interrupt, config_data;
InitData init_data;
StateData state_data;

// config data okay
volatile uint8_t config_ack = 0;

// I2C speed; try changing it to 50kHz
const int I2C_BAUDRATE = 100 * 1000;  // I2C speed in Hz, must be the same on Pi
int i2c_read_ctr = 0;                 // reads the number of times mcu is read from
volatile uint8_t i2c_cmd_id;             // received command over i2c
volatile uint8_t i2c_error_code = 0;

// dual core variables
volatile bool encoder_initialized = false;
volatile bool core1_setup_complete = false;

// heartbeat variables
volatile uint8_t heartbeat_rcvd_ID;
volatile uint8_t heartbeat_ctr;
volatile uint8_t heartbeat_expected_checksum;
volatile uint8_t heartbeat_rcvd_checksum;
volatile uint8_t heartbeat_last_ctr = 255;

volatile bool was_heartbeat_rcvd = false;
volatile bool is_heartbeat_valid = true;

volatile bool heartbeat_timeout_exceeded = false;

volatile bool is_heartbeat_ctr_valid = true;

volatile bool is_reed_switch_present = false;

const int HEARTBEAT_PERIOD = 50;                     // milliseconds
const int HEARTBEAT_FREQ = 1000 / HEARTBEAT_PERIOD;  // hz; must be same as from pi
const int HEARTBEAT_TIMEOUT = 5 * HEARTBEAT_PERIOD;  // miliseconds

volatile uint64_t heartbeat_timer = 0;  // = millis();
uint32_t millis_since_last_heartbeat = 0;

// hardware parameters
const float GEAR_RATIO = (38.0 / 16.0) * (48.0 / 16.0);
const int NUM_SPOKES = 10;

const float TORSO_MASS = 1577.83e-3;  //mass in kg from Solidworks
const float TORSO_COM_LENGTH = sqrt(pow(83.6, 2) + pow(6.12, 2)) / 1000;

// gravity
const float g = 9.81;                 // gravity m/s-2
float gravity_compensation_term = 0;  // for gravity compensation of the controller
float gravity_constant = 0;           // mass * length * g

// robot data variables
uint8_t mot_drv_mode;
float torso_pitch, torso_pitch_rate;
float mot_vel, mot_pos, mot_torque;        // at the motor
float wheel_pos, wheel_vel, wheel_torque;  // at the wheel
float mot_pos_init, torso_pitch_init;
float wheel_pos_init = -PI / NUM_SPOKES;  //assuming wheel going in positive direction

// Encoder variables
const float ENCODER_PPR = 2048.0;             // puses per rev
const float ENCODER_CPR = ENCODER_PPR * 4;    // counts per rev
const float ENCODER_uSPR = ENCODER_CPR * 64;  //usteps per rev
volatile int32_t signed_encoder_steps;        // volatile shared variable between cores
int32_t local_signed_encoder_steps;           // local encoder step variable
float wheel_rel_pos, wheel_rel_pos_init;      // relative angle

volatile float encoder_ang;

float local_encoder_ang;

volatile float wheel_rel_vel;
float local_wheel_rel_vel;

// for imu correction
float gravity_x = 0;
float gravity_y = 0;
float gravity_z = 0;

float imu_angle_adjustment = 4.0f * DEG_TO_RAD;  // imu angle adjustment relative to COM in radians; adjusted in torso_pitch >> from the imu angle code
float imu_scale_adjustment = PI / 2.9679;        //for some reason imu does not consider down to be 3.14

// Control parameters
const float CONTROL_FREQ = 100;             // control torque write freq
const int SENSORS_UPDATE_PERIOD_LOOPS = 1;  // get motor values every x loop

// max motor torque (default value, new config value received from Pi)
volatile float mot_max_torque = 0.5;

// -------------------------------------------

float control_error = 0, control_error_integral = 0;

float mot_cmd_torque = 0, wheel_cmd_torque = 0;  // torque command to the motor

uint64_t time_now = 0, time_last, time_since_last = 0;

// For the control signals
static uint32_t control_next_send_millis = 1000 / CONTROL_FREQ;  // initialized so that first control signal is NOT sent in the first loop iter
uint16_t control_loop_ctr = 0;

uint64_t start_time = 0, wait_time = 2 * 1000;  //milliseconds
uint64_t loop_count = 0;

// for the reed switch
volatile uint8_t robot_run_status = 0x00;  // 0x00 is okay

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
// sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
// need to change in the setReports() function
sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
// long IMU_report_freq = 200;    //Hz
long reportIntervalUs = 5000;  //1 / IMU_report_freq * 1000 * 1000;  //usec
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
// Moteus::CurrentMode::Command current_cmd;
// Moteus::CurrentMode::Format current_fmt;

// encoder object
PicoEncoder encoder;
// mutex_t encoder_mutex;  // mutex object

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void setup() {

  // mutex_init(&encoder_mutex);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(STOP_LED, OUTPUT);
  pinMode(REED_SWITCH, INPUT);

  // turn all LEDs high momentarily
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(OK_LED, HIGH);
  digitalWrite(STOP_LED, HIGH);

  Serial.begin(115200);

  unsigned long t = millis();  //current time
  // and condition ensures that the mcu starts even when not connected to Serial
  while (!Serial && millis() - t < 1000) {
    delay(1);
  }

  // turn all LEDs OFF
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(OK_LED, LOW);
  digitalWrite(STOP_LED, LOW);

  Serial.println(">>> Starting torsobot on core 0 ...");

  // check core 1 setup completion status
  Serial.println("Waiting for core 1 setup to complete ...");
  while (!core1_setup_complete) {
    if (digitalRead(STOP_LED) == LOW) digitalWrite(STOP_LED, HIGH);
    delay(10);
  }
  Serial.println("Core 1 setup complete!");
  digitalWrite(STOP_LED, LOW);

  // check encoder initializatin from core 1
  Serial.println("Checking encoder PIO initialization ...");
  if (encoder_initialized) {
    Serial.println("Encoder PIO setup okay!");
    digitalWrite(STOP_LED, LOW);
  } else {
    digitalWrite(STOP_LED, HIGH);
    Serial.println("Encoder PIO setup not okay!");
    while (1) delay(1);
  }

  // Check reed switch
  Serial.println("Checking reed switch ...");
  while (!digitalRead(REED_SWITCH)) {
    digitalWrite(STOP_LED, HIGH);
    delay(10);
  }
  Serial.println("Reed switch okay");
  is_reed_switch_present = true;
  digitalWrite(STOP_LED, LOW);

  Serial.println("Setting up I2C ...");
  // I2C setup
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);

  Wire.begin(PICO_SLAVE_ADDRESS);
  Wire.setClock(I2C_BAUDRATE);

  // set interrup functions for I2C
  Wire.onReceive(i2cRecv);
  Wire.onRequest(i2cReq);

  Serial.println("Setting up SPI ...");
  // Set SPI pins
  // IMU on SPI0
  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);

  // CAN controller on SPI1
  SPI1.setSCK(MCP2517_SCK);
  SPI1.setMOSI(MCP2517_MOSI);
  SPI1.setMISO(MCP2517_MISO);
  SPI1.setCS(MCP2517_CS);

  SPI1.begin();

  Serial.println("Initializing IMU ...");
  // IMU spi communication
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    digitalWrite(STOP_LED, HIGH);
    Serial.println(F("Failed to find BNO08x IMU chip"));
    while (1) delay(10);
  }
  Serial.println("BNO08x IMU Found!");
  digitalWrite(STOP_LED, LOW);

  // Reset the IMU
  delay(100);
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

  Serial.println("Checking CAN controller connection ...");
  const uint32_t errorCode = can.begin(settings, [] {
    can.isr();
  });
  // errorCode of 0 means success
  if (errorCode) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(100);
    return;
  }
  Serial.println("No CAN errors!");


  // Set formats for the moteus command
  position_fmt.kp_scale = Moteus::kFloat;
  position_fmt.kd_scale = Moteus::kFloat;
  position_fmt.feedforward_torque = Moteus::kFloat;

  position_cmd.position = NaN;
  position_cmd.velocity = 0.0f;  // Not required as resolution is set to ignore
  position_cmd.kp_scale = 0.0f;
  position_cmd.kd_scale = 0.0f;
  position_cmd.watchdog_timeout = 0.2f;
  // position_cmd.maximum_torque = mot_max_torque; // moved to the loop since this value cannot be updated over I2C

  Serial.println("Waiting for config write from PI...");
  while (!config_ack) {
    if (digitalRead(STOP_LED) == LOW) digitalWrite(STOP_LED, HIGH);
    // delay(10);
    // Serial.println(i2c_error_code);
  }
  Serial.println("Config was successful!");
  digitalWrite(STOP_LED, LOW);

  // briefly pause interrupts, copy the data, and turn them back on
  noInterrupts();                       // Pauses I2C temporarily
  config_data = config_data_interrupt;  // Instant, safe memory copy
  interrupts();                         // Resume normal I2C operation

  Serial.println("Received values from Pi:");
  Serial.print("Desired torso pitch:\t");
  Serial.println(config_data.desired_torso_pitch);
  Serial.print("kp:\t");
  Serial.println(config_data.kp);
  Serial.print("ki:\t");
  Serial.println(config_data.ki);
  Serial.print("kd:\t");
  Serial.println(config_data.kd);
  Serial.print("wheel_max_torque:\t");
  Serial.println(config_data.wheel_max_torque);
  Serial.print("control_max_integral:\t");
  Serial.println(config_data.control_max_integral);
  Serial.print("controller:\t");
  Serial.println(config_data.controller);

  mot_max_torque = config_data.wheel_max_torque / GEAR_RATIO;  // set once; overwrites default value

  // waits for heatrbeat
  Serial.println("Waiting for heartbeat from the PI...");
  while (!was_heartbeat_rcvd) {
    if (digitalRead(STOP_LED) == LOW) digitalWrite(STOP_LED, HIGH);
    // delay(10);
  }
  Serial.println("Heartbeat was received!");
  digitalWrite(STOP_LED, LOW);

  // save the last counter value
  // heartbeat_last_ctr = heartbeat_ctr;
  was_heartbeat_rcvd = false;

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus.SetStop();
  Serial.println("Motor stopped!");
  delay(5);

  Serial.println("Core 0 setup complete!");

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(OK_LED, HIGH);

  // Evaluate the gravity constant for the torso
  // gravity_constant = TORSO_MASS * g * TORSO_COM_LENGTH;
  gravity_constant = 2.10;  // evaluated using three trial runs
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void loop() {

  // get encoder steps from the shared memory
  local_signed_encoder_steps = signed_encoder_steps;
  local_encoder_ang = encoder_ang;
  // memcpy(encoder_steps_data, &local_signed_encoder_steps, sizeof(int32_t));

  // get speed from core1
  local_wheel_rel_vel = wheel_rel_vel;

  // copy all values to the struct and pause interrupts!
  noInterrupts();
  state_data.torso_pitch = torso_pitch;
  state_data.torso_pitch_rate = torso_pitch_rate;
  state_data.wheel_pos = wheel_pos;
  state_data.wheel_vel = wheel_vel;
  state_data.encoder_steps = local_signed_encoder_steps;
  state_data.encoder_ang = local_encoder_ang;
  state_data.encoder_speed = local_wheel_rel_vel;
  state_data.motor_drv_mode = mot_drv_mode;
  state_data.wheel_cmd_torque = wheel_cmd_torque;
  state_data.wheel_actual_torque = wheel_torque;

  state_data.checksum = calculate_checksum<StateData>(state_data);
  interrupts();

  // checks then increments
  if (loop_count++ == 0) {
    start_time = millis();  // stores start time of the main loop function
    Serial.print("loop start time:  ");
    Serial.println(start_time);
  }

  if (!digitalRead(REED_SWITCH)) {
    Serial.println("Reed switch not detected!");
    is_reed_switch_present = false;
    emergencyStop();
  }
  is_reed_switch_present = true;

  // is_heartbeat_valid = checkHeartbeatValidity();
  millis_since_last_heartbeat = millis() - heartbeat_timer;  // heartbeat_timer is updated by the i2cRecv function if heartbeat is valid

  if (millis_since_last_heartbeat > HEARTBEAT_TIMEOUT) {
    heartbeat_timeout_exceeded = true;
  } else {
    heartbeat_timeout_exceeded = false;
  }

  // checks for 1) heartbeat validity (checksum and expected counter value) and 2) time since last heartbeat received
  if (!is_heartbeat_valid || heartbeat_timeout_exceeded) {
    Serial.print("millis_since_last_heartbeat: ");
    Serial.println(millis_since_last_heartbeat);

    Serial.print("is_heartbeat_valid?: ");
    Serial.println(is_heartbeat_valid);

    Serial.println("heartbeat not valid or heartbeat timeout reached!");

    emergencyStop();  // call the stop routine
  }

  // Check IMU
  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset during loop!");
    torso_pitch = NaN;
    torso_pitch_rate = NaN;
    setReports(reportIntervalUs);
    // resetOccurred = true;
    // memcpy(torso_pitch_data, &torso_pitch, sizeof(float));
  }

  // If IMU sensor event is true
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on BNO08X_FAST_MODE define (above)

    // Serial.print("Sensor ID:");
    // Serial.println(sensorValue.sensorId);

    switch (sensorValue.sensorId) {
        // case SH2_ARVR_STABILIZED_RV:
        //   // quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        //   // torso_pitch = degToRad(imuAngleCorrection(ypr.pitch, gravity_x, gravity_y, gravity_z)) + imu_angle_adjustment;  // adjusts sign and adds the angle between the imu axes and the COM
        //   // break;

      case SH2_GAME_ROTATION_VECTOR:
        // torso_pitch is updated by reference
        // &sensorValue.un.rotationVector uses the magnetomoeter and causes x/y values to jump due to yaw corrections
        // arvrstabilizedrv does not use magnetometer
        quaternionToPitch(&sensorValue.un.gameRotationVector, &torso_pitch);
        torso_pitch -= imu_angle_adjustment;
        // torso_pitch *= imu_scale_adjustment;
        torso_pitch = wrapTo2PI(torso_pitch);  // we need angles in the [0, 2*PI) range
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        torso_pitch_rate = sensorValue.un.gyroscope.y;
        break;

      case SH2_GRAVITY:
        gravity_x = sensorValue.un.gravity.x;
        gravity_y = sensorValue.un.gravity.y;
        gravity_z = sensorValue.un.gravity.z;
        break;
    }
  }

  // copy to buffers to send to pi
  // memcpy(torso_pitch_data, &torso_pitch, sizeof(float));
  // memcpy(torso_pitch_rate_data, &torso_pitch_rate, sizeof(float));
  // // Serial.println();

  // **************************************************************
  // Controls part

  // get these values until wait time is over
  // if we do not write to the controller, the motor sensor values don't update
  if (control_loop_ctr == 0) {
    // compute ff torque command to the motor; 0 = do nothing
    // position_cmd.maximum_torque = mot_max_torque;
    // position_cmd.feedforward_torque = 0;

    // // // **Write to the motor; kkep ccmmented out
    // // moteus.SetPosition(position_cmd, &position_fmt);

    // get values
    mot_drv_mode = static_cast<int8_t>(moteus.last_result().values.mode);
    mot_pos = revsToRad(moteus.last_result().values.position);  //radians
    mot_vel = revsToRad(moteus.last_result().values.velocity);  // radians/s

    // keep reading these values; the last value stored before control gets updated is the one that we want
    torso_pitch_init = torso_pitch;
    // local_signed_encoder_steps = signed_encoder_steps;
    wheel_rel_pos_init = encoderCountsToRad(local_signed_encoder_steps);


    // mot_pos_init = mot_pos;

    // copy into struct
    noInterrupts();
    init_data.torso_pitch_init = torso_pitch_init;
    init_data.wheel_rel_pos_init = wheel_rel_pos_init;
    init_data.checksum = calculate_checksum<InitData>(init_data);
    interrupts();

    // Serial.print("mot_pos_init:\t");
    // Serial.print(mot_pos_init);
    // Serial.print("\t");
    // Serial.println(torso_pitch_init);

    // copy to buffer to be sent to pi
    // memcpy(mot_pos_data, &mot_pos, sizeof(float));
    // memcpy(mot_vel_data, &mot_vel, sizeof(float));  //
    // memcpy(mot_drv_mode_data, &mot_drv_mode, sizeof(int8_t));
    // memcpy(torso_pitch_init_data, &torso_pitch_init, sizeof(float));
    // memcpy(mot_pos_init_data, &mot_drv_momot_pos_initde, sizeof(float));
  }

  // Do wheel calculations
  // local_signed_encoder_steps = signed_encoder_steps;                               //check sign value based on encoder channel pins
  wheel_rel_pos = encoderCountsToRad(local_signed_encoder_steps);  // radians
  wheel_rel_pos -= wheel_rel_pos_init;

  // when looked from the driver's side, motor aclc is positive, torso acls is positive
  wheel_pos = wheel_rel_pos + (torso_pitch - torso_pitch_init);

  // When the robot starts, it's wheel angle is -collision angle
  wheel_pos += wheel_pos_init;

  // wrap around [-PI/n, PI/n)
  wheel_pos = fmod(wheel_pos, 2 * PI / NUM_SPOKES);

  if (wheel_pos < -PI / NUM_SPOKES) {
    wheel_pos += 2 * PI / NUM_SPOKES;
  } else if (wheel_pos >= PI / NUM_SPOKES) {
    wheel_pos -= 2 * PI / NUM_SPOKES;
  } else {  //do nothing
  }

  // get other wheel values
  // wheel_rel_vel = (encoder.speed / ENCODER_uSPR) * 2 * PI;  // relative velocity in rad/s
  wheel_vel = torso_pitch_rate + local_wheel_rel_vel;  // does not require init values for the mot or the torso

  // memcpy(wheel_pos_data, &wheel_pos, sizeof(float));
  // memcpy(wheel_vel_data, &wheel_vel, sizeof(float));

  // We intend to send control frames every 1000/CONTROL_FREQ milliseconds
  if (control_next_send_millis >= millis()) return;  //early return >> next loop

  // update value
  control_next_send_millis += 1000 / CONTROL_FREQ;

  // Print values for debugging (not in each loop)
  // Serial.println("<<<---<<<");
  // Serial.print(desired_torso_pitch, 4);
  // Serial.print("\t");
  // Serial.print(torso_pitch, 4);
  // Serial.print("\t");
  // Serial.print(torso_pitch_rate, 4);
  // Serial.print("\t");
  // // Serial.println(isnan(torso_pitch));
  // Serial.print(wheel_pos, 4);
  // Serial.print("\t");
  // Serial.print(local_signed_encoder_steps);
  // // Serial.print("\t");
  // // Serial.print(radToDeg(wheel_pos), 4);
  // Serial.print("\t");
  // Serial.print(wheel_vel, 4);
  // // Serial.print("\t");
  // // Serial.print(TORSO_MASS, 4);
  // // Serial.print("\t");
  // // Serial.print(g, 4);
  // Serial.print("\t");
  // // Serial.print(TORSO_COM_LENGTH, 4);
  // // Serial.print("\t");
  // // Serial.print(sin(torso_pitch), 4);
  // // Serial.print("\t");
  // // Serial.print(gravity_compensation_term, 4);
  // // Serial.print("\t");
  // Serial.println(mot_cmd_torque, 4);
  // Serial.print("\t");
  // Serial.print(mot_torque, 4);
  // Serial.print("\t");
  // Serial.print(mot_drv_mode, 4);


  // // These values are received over I2C
  // Serial.print(kp);
  // Serial.print("\t");
  // Serial.print(ki);
  // Serial.print("\t");
  // Serial.print(kd);
  // Serial.print("\t");
  // Serial.print(wheel_max_torque);
  // Serial.print("\t");
  // Serial.print(mot_max_torque);
  // Serial.print("\t");
  // Serial.print(max_integral);
  // Serial.print("\t");
  // Serial.print(control_error);
  // Serial.print("\t");
  // Serial.println("---");

  // this condition allows imu values to stabilise at program start
  if (millis() - start_time < wait_time) {
    // We need to write to the motor driver, otherwise the watchdog timer gives the position timeout error
    position_cmd.maximum_torque = mot_max_torque;
    position_cmd.feedforward_torque = 0;
    // // **Write to the motor
    // moteus.SetPosition(position_cmd, &position_fmt);

    if (moteus.last_result().values.mode == mjbots::moteus::Mode::kPositionTimeout) {
      Serial.println("motor driver fault");
      moteus.SetStop();
    }

    return;  // return early >> next loop
  }

  // Fault mot_drv_mode
  // 10 is the position mode
  // 11 is position timeout error
  // !!! This might cause jitters!
  // if (moteus.last_result().values.mode != mjbots::moteus::Mode::kPosition) {
  //   Serial.println("motor driver fault");
  //   moteus.SetStop();
  // }

  time_last = time_now;  // store value from previous control loop's timer

  // If control_loop_ctr is 0 (first loop), update the time_last value, since we don't want the error to begin accumulating from the start of the program (only at the start of the control loop)
  if (control_loop_ctr != 0)
    time_last = millis();

  time_now = millis();                     // get current time
  time_since_last = time_now - time_last;  //time since last loop

  // ***Calculate torque required***
  // Calculate error
  control_error = config_data.desired_torso_pitch - torso_pitch;  // radians

  // wrap error so that it is always between -PI to PI radians
  if (control_error > PI) {
    control_error -= 2 * PI;
  } else if (control_error < -PI) {
    control_error += 2 * PI;
  }

  // Integral term
  control_error_integral += control_error * time_since_last;
  // clamp the integral to prevent integral windup
  control_error_integral = min(config_data.control_max_integral, max(-config_data.control_max_integral, control_error_integral));

  // time derivative of control error = -torso_pitch_rate ; because desired_pitch_rate is constant

  mot_cmd_torque = config_data.kp * control_error + config_data.ki * control_error_integral + config_data.kd * (-torso_pitch_rate);

  // term for integral torque
  // mot_cmd_torque += ki * control_error_integral;
  if (config_data.controller == 1) {  // term for gravity compensation
    gravity_compensation_term = gravity_constant * sin(torso_pitch);
    mot_cmd_torque -= gravity_compensation_term / GEAR_RATIO;  // add a gravity compensation term (adjusted for gear ratio) at the motor
  }

  mot_cmd_torque = min(mot_max_torque, max(-mot_max_torque, mot_cmd_torque));  // second safety net; max torque has already been defined for the motor board but this line ensures no value greater than max is written

  mot_cmd_torque = -mot_cmd_torque;  //- sign corrects for sign difference between +torque on torso vs +torso rotation; +torso rotation is anticlockwise when viwed from motor driver end

  // write to cmd
  position_cmd.maximum_torque = mot_max_torque;
  position_cmd.feedforward_torque = mot_cmd_torque;
  // position_cmd.feedforward_torque = 0;  //testing

  // // **Write to the motor only if imu values are not nan
  // // Serial.println(millis());
  // float check_pitch_nan;
  // memcpy(&check_pitch_nan, torso_pitch_data, sizeof(float));

  // moteus.SetPosition(position_cmd, &position_fmt);

  // Serial.println(millis());

  // control_loop_ctr++;
  // **************************************************************


  // **************************************************************
  // copy data from driver to buffers

  // Get values every SENSORS_UPDATE_PERIOD_LOOPS iters
  if (control_loop_ctr++ % SENSORS_UPDATE_PERIOD_LOOPS != 0) {
    return;
  }
  // digitalWrite(LED_BUILTIN, HIGH);

  // print_moteus(moteus.last_result().values);
  // last result only returns values if something is written to the board

  // uint32_t status = save_and_disable_interrupts();


  mot_drv_mode = static_cast<int8_t>(moteus.last_result().values.mode);
  // mot_pos = revsToRad(moteus.last_result().values.position);  //radians
  // mot_vel = revsToRad(moteus.last_result().values.velocity);  // radians/s
  mot_torque = moteus.last_result().values.torque;

  // get init values only in the 0th loop
  // if (control_loop_ctr == 1) {
  //   torso_pitch_init = torso_pitch;
  //   mot_pos_init = mot_pos;
  //   Serial.print("mot_pos_init");
  //   Serial.print(mot_pos_init);
  //   Serial.print("\t");
  //   Serial.println(torso_pitch_init);
  // }

  // calculate wheel position from motor and torso angles
  // mot pos is relative to the torso


  wheel_torque = mot_torque * GEAR_RATIO;
  wheel_cmd_torque = mot_cmd_torque * GEAR_RATIO;

  // Serial.print("millis_since_last_heartbeat: ");
  // Serial.println(millis_since_last_heartbeat);
  // Serial.print("mot_drv_mode: ");
  // Serial.print(mot_drv_mode);
  // Serial.print("\tcommand_torque: ");
  // Serial.print(mot_cmd_torque);
  // Serial.print("\tmot_torque: ");
  // Serial.print(mot_torque);
  // Serial.print("\tmot_pos: ");
  // Serial.print(mot_pos);
  // Serial.print("\tmot_vel: ");
  // Serial.println(mot_vel);

  // copy to buffer to send to pi
  // memcpy(mot_pos_data, &mot_pos, sizeof(float));
  // memcpy(mot_vel_data, &mot_vel, sizeof(float));                    //


  // memcpy(wheel_cmd_torque_data, &wheel_cmd_torque, sizeof(float));  //
  // memcpy(wheel_torque_data, &wheel_torque, sizeof(float));
  // memcpy(mot_drv_mode_data, &mot_drv_mode, sizeof(int8_t));

  // restore_interrupts(status);
  // memcpy(mot_drv_mode_data, &mot_drv_mode, sizeof(int8_t));

  // **************************************************************

  // Delay function messes with the IMU data and causes buffers to overflow(?);
  // bno085 goes into reset state and needs a hardwareReset() call; hardwareReset() doesn't always work
  // delay(5);
}

void setup1() {
  if (encoder.begin(encoder_pinA)) {
    // If begin() returns failure (-1), we enter this block
    encoder_initialized = false;
    core1_setup_complete = true;
    while (1) {
      // Freeze here so we don't try to run a broken robot
      delay(1);
    }
  }
  encoder_initialized = true;
  core1_setup_complete = true;
}

void loop1() {
  // update encoder value
  static float prev_encoder_ang;
  static uint32_t prev_time;
  static uint32_t loop1_ctr;

  if (loop1_ctr++ == 0) {
    prev_encoder_ang = 0;
    prev_time = micros();
  }

  // update values
  encoder.update();

  signed_encoder_steps = -static_cast<int32_t>(encoder.step);  // sign depends on pin connections for channel A and B
  encoder_ang = encoderCountsToRad(signed_encoder_steps);      //encoder angle in rad
  wheel_rel_vel = -(encoder.speed / ENCODER_uSPR) * 2 * PI;    // relative velocity in rad/s

  // // wheel vel by finite difference
  // uint32_t time_now = micros();
  // uint32_t time_diff_micros = time_now - prev_time;

  // if (time_diff_micros > 1000) {
  //   wheel_rel_vel = ((encoder_ang - prev_encoder_ang) / time_diff_micros) * 1000 * 1000;
  //   prev_encoder_ang = encoder_ang;
  //   prev_time = time_now;
  // }
}

///////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////
// Called when the I2C slave receives something from master
void i2cRecv(int len) {

  if (len < 1) return;

  i2c_cmd_id = Wire.peek();  // or the ID

  if (i2c_cmd_id == HEARTBEAT_ID) {
    if (len == sizeof(Heartbeat)) {
      was_heartbeat_rcvd = true;

      Heartbeat incoming_heartbeat;

      Wire.readBytes((uint8_t *)&incoming_heartbeat, sizeof(Heartbeat));

      uint8_t calculated_checksum = calculate_checksum<Heartbeat>(incoming_heartbeat);

      if (calculated_checksum == incoming_heartbeat.checksum) {  // && (heartbeat_ctr == (heartbeat_last_ctr + 1) % 256)) {

        is_heartbeat_valid = true;

        if (incoming_heartbeat.counter != heartbeat_last_ctr) {  //(heartbeat_ctr == (heartbeat_last_ctr + 1) % 256) {
          is_heartbeat_ctr_valid = true;
          heartbeat_last_ctr = incoming_heartbeat.counter;
          heartbeat_timer = millis();
        } else {
          is_heartbeat_ctr_valid = false;
        }

      } else {
        is_heartbeat_valid = false;
      }
    } else {
      while (Wire.available()) Wire.read();
    }
  }
  //check for config_cmd
  else if (i2c_cmd_id == MCU_CONTROLLER_CONFIG_ID) {
    i2c_error_code = 1;
    if (len == sizeof(mcuConfigData)) {
      mcuConfigData incoming_config_data;

      Wire.readBytes((uint8_t *)&incoming_config_data, sizeof(mcuConfigData));

      uint8_t calculated_checksum = calculate_checksum<mcuConfigData>(incoming_config_data);

      if (calculated_checksum == incoming_config_data.checksum) {
        config_data_interrupt = incoming_config_data;
        config_ack = 1;
      } else {
        i2c_error_code = 3;
        config_ack = 0;
      }

    } else {
      i2c_error_code = 2;
      while (Wire.available()) Wire.read();
    }

  }
  // ensures that if there are remaining bytes, they are read so that the buffer is emptied
  else {
    while (Wire.available()) Wire.read();
  }
}

// Called when the I2C slave is read from
void i2cReq() {
  i2c_read_ctr++;
  switch (i2c_cmd_id) {
    case HEARTBEAT_ID:
      Wire.write(robot_run_status);
      break;
    case MCU_CONTROLLER_CONFIG_ID:
      Wire.write(config_ack);
      break;
    case STATE_DATA_ID:
      Wire.write((uint8_t *)&state_data, sizeof(StateData));
      break;
    case INIT_DATA_ID:
      Wire.write((uint8_t *)&init_data, sizeof(InitData));
      break;
  }
}

void setReports(long report_interval) {
  Serial.println("Setting desired reports");
  // if (!bno08x.enableReport(SH2_ROTATION_VECTOR, report_interval)) {
  //   Serial.println("Could not enable rotation vector");
  // }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, report_interval)) {
    Serial.println("Could not enable rotation vector");
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
// quaternions to Euler
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

// quaternion to pitch angle using rotation matrices to bypass euler pitch singularity
// gets the angle between imu's x-axis and the global horizontal plane; same as euler pitch
void quaternionToPitch(sh2_RotationVector_t *rotational_vector, float *pitch, bool degrees) {
  float qw = rotational_vector->real;
  float qx = rotational_vector->i;
  float qy = rotational_vector->j;
  float qz = rotational_vector->k;

  // Calculate the two components of the Rotation Matrix we care about
  // These represent the relationship between the Body's X/Z axes and World Gravity.

  // R31: Projection of World Z on Body X (Sine component for Y-rotation); the world z is the gavity vector
  float R31 = 2.0f * (qx * qz - qw * qy);

  // R33: Projection of World Z on Body Z (Cosine component for Y-rotation)
  float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);

  // Calculate the full 360-degree angle (atan2 handles all 4 quadrants automatically)
  // We use -R31 because standard pitch definition often opposes the matrix index direction
  *pitch = atan2(-R31, R33);

  if (degrees) {
    *pitch *= RAD_TO_DEG;
  }
}

// quaternions to Euler Rotation Vector
void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// quaternions to Euler Gyro Integrated
void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// change from degrees to cycles/full rotations
float degToRevs(float deg) {
  return deg / 360;
}

// change from revolutions to radians
float revsToRad(float revs) {
  return revs * 2 * PI;
}

// change value from degrees to radians
float degToRad(float deg) {
  return deg / 180 * PI;
}

// change value from radians to degrees
float radToDeg(float rad) {
  return rad * 180 / PI;
}

// wrap angle from 0 to 360
float wrapTo2PI(float angle) {

  if (angle < 0) { angle += 2 * PI; }
  return angle;
}

// encoder counts (or steps) to radians
float encoderCountsToRad(int32_t local_signed_encoder_steps) {
  return static_cast<float>(local_signed_encoder_steps) / ENCODER_CPR * 2 * PI;
}

// encoder counts (or steps) to degrees
float encoderCountsToDeg(int32_t local_signed_encoder_steps) {
  return static_cast<float>(local_signed_encoder_steps) / ENCODER_CPR * 360.0;
}

// manipulates the IMU values
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

  pitch = wrapTo2PI(pitch);

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

// sign function
int signof(float num) {
  // returns +1 for pos, -1 for neg, and 0 for 0
  return (num > 0) - (num < 0);
}

// called when program has to stop
void emergencyStop(void) {
  // robot_run_status = 0xFF;  // true
  Serial.println("!!! Stopping program! :(");
  Serial.println(is_heartbeat_valid);

  if (!is_heartbeat_valid) robot_run_status = 0xAA;
  else if (!is_heartbeat_ctr_valid) robot_run_status = 0xBB;
  else if (heartbeat_timeout_exceeded) robot_run_status = 0xCC;
  else if (!is_reed_switch_present) robot_run_status = 0xDD;

  Serial.println(is_heartbeat_ctr_valid);
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

  while (1) {
    // infinite while loop
    delay(1);
  }
}
