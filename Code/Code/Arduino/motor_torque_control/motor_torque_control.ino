#include <Arduino.h>
#include <Wire.h>
#include <cstdlib>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
// Pin definition
//——————————————————————————————————————————————————————————————————————————————

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

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool deegrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees);
void setReports(long report_interval);
float degToCycles(float);
float imuAngleCorrection(float, float, float, float);
float gravityMagnitude(float, float, float);
float degToRad(float);
bool checkHeartbeatValidity(void);
void emergencyStop(void);

//——————————————————————————————————————————————————————————————————————————————
//  Variable declarations
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gNextSendMillis = 0;

const int I2C_BAUDRATE = 25 * 1000;  // I2C speed in Hz

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
const int HEARTBEAT_FREQ = 1000 / 25;                      //hz; must be same as from pi
const int HEARTBEAT_TIMEOUT = 10 * 1000 / HEARTBEAT_FREQ;  //miliseconds
volatile unsigned long long heartbeat_timer = 0;           // = millis();
long millis_since_last_heartbeat = 0;

// I2C cmd for mcu data
const uint8_t DESIRED_TORSO_PITCH_CMD = 0x00;  // desired torso_pitch
const uint8_t IMU_PITCH_CMD = 0x01;        // torso_pitch
const uint8_t IMU_PITCH_RATE_CMD = 0x02;   // torso_pitch_rate
const uint8_t MOTOR_POS_CMD = 0x03;        // motor rotor position (not wheel)
const uint8_t MOTOR_VEL_CMD = 0x04;        // motor velocity position (not wheel)
const uint8_t MOTOR_TORQUE_CMD = 0X05;     // motor torque at motor shaft (not wheel)
const uint8_t MOTOR_DRV_MODE_CMD = 0X06;   // motor driver mode

const int data_len = sizeof(float);  // 4 bytes
char desired_torso_pitch_data[data_len];
char imu_pitch_data[data_len];
char imu_pitch_rate_data[data_len];
char mot_pos_data[data_len];
char mot_vel_data[data_len];
char mot_torque_data[data_len];
char drv_mode_data[sizeof(int8_t)];

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

volatile float desired_torso_pitch = degToRad(10);

float kp = 0.5;
float kd = 0.0;

float max_torque = 0.5;

int ctr = 0;

euler_t ypr;

uint16_t gLoopCount = 3;

float ff_torque;

//——————————————————————————————————————————————————————————————————————————————
//  BNO08X object
//——————————————————————————————————————————————————————————————————————————————

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

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

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
  delay(2000);

  Serial.println("Starting torsobot ...");

  // I2C lines
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);

  Wire.begin(PICO_SLAVE_ADDRESS);
  Wire.setClock(I2C_BAUDRATE);

  // set interrup functions for I2C
  Wire.onReceive(recv);
  Wire.onRequest(req);

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
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  bno08x.hardwareReset();
  delay(1000);
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
    delay(1000);
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

  // moteus.SetPositionWaitComplete(position_cmd, 0.02, &position_fmt);

  // desired pitch angle
  memcpy(desired_torso_pitch_data, &desired_torso_pitch, sizeof((desired_torso_pitch)));

  Serial.println("Waiting for heartbeat from the PI...");

  // waits for heatrbeat
  while (!was_heartbeat_rcvd) { digitalWrite(STOP_LED, HIGH); }  //Serial.println(was_heartbeat_rcvd); }

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(OK_LED, LOW);

  // save the last counter value
  heartbeat_last_ctr = heartbeat_ctr;
  was_heartbeat_rcvd = false;

  Serial.println("Heartbeat received from PI!");
  Serial.println("Setup complete!");
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(OK_LED, HIGH);
  
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void loop() {


  // is_heartbeat_valid = checkHeartbeatValidity();
  millis_since_last_heartbeat = millis() - heartbeat_timer;

  if (!is_heartbeat_valid || (millis_since_last_heartbeat > HEARTBEAT_TIMEOUT)) {
    Serial.print("millis_since_last_heartbeat: ");
    Serial.println(millis_since_last_heartbeat);

    Serial.print("is_heartbeat_valid?: ");
    Serial.println(is_heartbeat_valid);

    Serial.println("heartbeat not valid or heartbeat timeout reached!");

    emergencyStop();  // call the stop routine
  }

  // is_heartbeat_valid = false;

  // digitalWrite(LED_BUILTIN, LOW);

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
    Serial.print(imu_pitch, 4);
    Serial.print(", ");

    // imu_pitch = imuAngleCorrection(ypr.pitch);
    Serial.print(imu_pitch_rate, 4);
    Serial.print(", ");

    Serial.println(gravity_y, 4);

    memcpy(imu_pitch_data, &imu_pitch, sizeof(float));
    memcpy(imu_pitch_rate_data, &imu_pitch_rate, sizeof(float));
  }
  // // Serial.println();

  // We intend to send control frames every 20ms.
  const auto time = millis();

  if (gNextSendMillis >= time) { return; }

  // Fault drv_mode
  if (static_cast<int>(moteus.last_result().values.mode) == 11) {
    Serial.println("motor driver fault");
    moteus.SetStop();
  }

  gNextSendMillis += 20;
  gLoopCount++;

  // calculate torque required
  ff_torque = -kp * (imu_pitch - desired_torso_pitch) - kd * (imu_pitch_rate - 0);
  ff_torque = min(max_torque, ff_torque);  // second safety net; max torque has already been defined for the board but this line esnures no value greater than max is written
  position_cmd.feedforward_torque = ff_torque;

  // Write to the motor
  Serial.println(millis());
  moteus.SetPosition(position_cmd, &position_fmt);
  Serial.println(millis());
  // Get values every 5 loop iters
  if (gLoopCount % 5 != 0) { return; }
  // digitalWrite(LED_BUILTIN, HIGH);

  // print_moteus(moteus.last_result().values);
  drv_mode = static_cast<int8_t>(moteus.last_result().values.mode);
  mot_vel = moteus.last_result().values.velocity;
  mot_pos = moteus.last_result().values.position;
  mot_torque = moteus.last_result().values.torque;

  Serial.print("millis_since_last_heartbeat: ");
  Serial.println(millis_since_last_heartbeat);
  Serial.print("drv_mode: ");
  Serial.print(drv_mode);
  Serial.print("\tcommand_torque: ");
  Serial.print(ff_torque);
  Serial.print("\tmot_torque: ");
  Serial.print(mot_torque);
  Serial.print("\tmot_pos: ");
  Serial.print(mot_pos);
  Serial.print("\tmot_vel: ");
  Serial.println(mot_vel);

  memcpy(mot_pos_data, &mot_pos, sizeof(float));
  memcpy(mot_vel_data, &mot_vel, sizeof(float));
  memcpy(mot_torque_data, &mot_torque, sizeof(float));
  memcpy(drv_mode_data, &drv_mode, sizeof(int8_t));

  // Delay function messes with the IMU data and causes buffers to overflow(?);
  // bno085 goes into reset state and needs a hardwareReset() call;
  // delay(5);
}

///////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////
// Called when the I2C slave gets written to
void recv(int len) {
  read_command = Wire.read();  // or the ID

  if (static_cast<uint8_t>(read_command) == HEARTBEAT_ID) {
    if (len == 3) {
      heartbeat_rcvd_ID = read_command;
      heartbeat_ctr = Wire.read();
      heartbeat_rcvd_checksum = Wire.read();
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
}

// Called when the I2C slave is read from
void req() {
  ctr++;
  switch (read_command) {
    case DESIRED_TORSO_PITCH_CMD:
      Wire.write(desired_torso_pitch_data, sizeof(desired_torso_pitch_data));
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
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
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

float imuAngleCorrection(float pitch, float gravity_x, float gravity_y, float gravity_z) {
  pitch = 90 - pitch;

  // pitch is always positive so no need to take abs

  // Evaluate angle from gravity vector when close to vertical
  if (abs(gravity_y) < 1.0) pitch = abs(radToDeg(asin(gravity_y / gravityMagnitude(gravity_x, gravity_y, gravity_z))));

  // Change sign of angle based on gravity vector's j component
  if (gravity_y
      < 0) pitch = -pitch;

  return pitch;
}

float gravityMagnitude(float gravity_x, float gravity_y, float gravity_z) {
  return sqrt(pow(gravity_x, 2) + pow(gravity_y, 2) + pow(gravity_z, 2));
}

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
