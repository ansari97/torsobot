#include <Arduino.h>
#include <Wire.h>
#include <cstdlib>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CAN Controller and the IMU board.
//——————————————————————————————————————————————————————————————————————————————

// For BNO085 SPI mode, we need a CS pin
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

// Defining fast mode runs the IMU at a higher frequency but causes values to drift
// #define BNO08X_FAST_MODE

// I2C slave address
#define PICO_SLAVE_ADDRESS 0x30

static uint32_t gNextSendMillis = 0;

// Sensor value cmd bytes
const int IMU_CMD = 0x1;
const int MOTOR_VAL_CMD = 0x2;

const int data_len = sizeof(float);  // 4 bytes
char imu_data[data_len];
char mot_vel_data[data_len];

// char write_buff[data_len + 10];
// char write_buff2[data_len + 10];

char read_command;

int mode;
float imu_pitch;
float mot_vel;
float pos_vel;

int ctr = 0;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool deegrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees);
void setReports(sh2_SensorId_t reportType, long report_interval);
float degToCycles(float);

euler_t ypr;

uint16_t gLoopCount = 0;

//——————————————————————————————————————————————————————————————————————————————
//  BNO08X object
//——————————————————————————————————————————————————————————————————————————————

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool resetOccurred = false;

#ifdef BNO08X_FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

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

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(5000);

  Serial.println("Staring I2C slave...IMU and motor driver test");

  // I2C lines
  Wire.setSDA(0);
  Wire.setSCL(1);

  Wire.begin(PICO_SLAVE_ADDRESS);

  Wire.onReceive(recv);
  Wire.onRequest(req);

  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);

  SPI1.setSCK(MCP2517_SCK);
  SPI1.setMOSI(MCP2517_MOSI);
  SPI1.setMISO(MCP2517_MISO);
  SPI1.setCS(MCP2517_CS);

  SPI1.begin();

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println(F("Failed to find BNO08x chip"));
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  bno08x.hardwareReset();
  delay(1000);
  Serial.print("Sensor reset?\t");
  Serial.println(bno08x.wasReset());

  setReports(reportType, reportIntervalUs);

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

  position_fmt.velocity_limit = Moteus::kFloat;
  position_fmt.accel_limit = Moteus::kFloat;

  position_cmd.velocity_limit = 2.0;
  position_cmd.accel_limit = 3.0;



  Serial.println("wait...");
  // delay(5000);

  position_cmd.position = 0.0;
  moteus.SetPositionWaitComplete(position_cmd, 0.02, &position_fmt);
  Serial.println("motor at zero!");
  delay(5000);
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void loop() {
  digitalWrite(LED_BUILTIN, LOW);

  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset during loop!");
    setReports(reportType, reportIntervalUs);
    // resetOccurred = true;
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on BNO08X_FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    imu_pitch = ypr.pitch;
    Serial.println(imu_pitch);

    memcpy(imu_data, &imu_pitch, sizeof(float));
  }
  // // Serial.println();


  // We intend to send control frames every 20ms.
  const auto time = millis();

  if (gNextSendMillis >= time) { return; }

  if (static_cast<int>(moteus.last_result().values.mode) == 11) {
    moteus.SetStop();
  }

  gNextSendMillis += 20;
  gLoopCount++;

  position_cmd.position = degToCycles(imu_pitch);
  moteus.SetPosition(position_cmd, &position_fmt);

  if (gLoopCount % 5 != 0) { return; }
  digitalWrite(LED_BUILTIN, HIGH);

  // auto print_moteus = [](const Moteus::Query::Result& query) {
  //   Serial.print("mode: ");
  //   Serial.print(static_cast<int>(query.mode));
  //   //   Serial.print("\tposition: ");
  //   //   Serial.print(query.position);
  //   //   Serial.print("\tvelocity: ");
  //   //   Serial.print(query.velocity);
  // };

  // print_moteus(moteus.last_result().values);
  mode = static_cast<int>(moteus.last_result().values.mode);
  mot_vel = moteus.last_result().values.velocity;
  pos_vel = moteus.last_result().values.position;

  Serial.print("mode: ");
  Serial.print(mode);
  Serial.print("\tpos_vel: ");
  Serial.print(pos_vel);
  Serial.print("\tmot_vel: ");
  Serial.println(mot_vel);

  memcpy(mot_vel_data, &mot_vel, sizeof(float));

  // Delay function messes with the IMU data and causes buffers to overflow(?);
  // bno085 goes into reset state and needs a hardwareReset() call;
  // delay(5);
}

///////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////
// Called when the I2C slave gets written to
void recv(int len) {
  read_command = Wire.read();
}

// Called when the I2C slave is read from
void req() {
  ctr++;
  switch (read_command) {
    case IMU_CMD:
      Wire.write(imu_data, sizeof(imu_data));
      break;
    case MOTOR_VAL_CMD:
      Wire.write(mot_vel_data, sizeof(mot_vel_data));
      break;
  }
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
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
