#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

// For BNO085 SPI mode, we need a CS pin
#define BNO08X_SCK 2
#define BNO08X_MOSI 3
#define BNO08X_MISO 4
#define BNO08X_CS 5
#define BNO08X_INT 10
// For SPI mode, we also need a RESET
#define BNO08X_RESET 11
#define BNO08X_FAST_MODE

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CANBed FD board.
//——————————————————————————————————————————————————————————————————————————————

#define MCP2517_SCK 2   // SCK input of MCP2517
#define MCP2517_MOSI 3  //  input of MCP2517
#define MCP2517_MISO 4  // SDO output of MCP2517

#define MCP2517_CS 1    // CS input of MCP2517
#define MCP2517_INT 12  // INT output of MCP2517

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool deegrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees);

euler_t ypr;

uint16_t gLoopCount = 0;
static uint32_t gNextSendMillis = 0;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef BNO08X_FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(BNO08X_CS, OUTPUT);
  // pinMode(MCP2517_CS, OUTPUT);

  // digitalWrite(BNO08X_CS, HIGH);  // Disable both devices initially
  // digitalWrite(MCP2517_CS, HIGH);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println(F("Adafruit BNO08x and BLDC motor driver test!"));

  Serial.println("here");
  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);

  Serial.println("here1");

  // SPI.setSCK(MCP2517_SCK);
  // SPI.setMOSI(MCP2517_MOSI);
  // SPI.setMISO(MCP2517_MISO);
  SPI.setCS(MCP2517_CS);

  SPI.begin();

  Serial.println("here2");

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  // AAA: changed oscillator freq to 40MHz (same as MIKROE-2379 setting)
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

  while (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(10);
  }

  // To clear any faults the controllers may have, we start by sending
  // a stop command to the motor.
  moteus1.SetStop();
  Serial.println(F("motors stopped"));

  // Try to initialize!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println(F("Failed to find BNO08x chip"));
    while (1) { delay(10); }
  }
  Serial.println(F("BNO08x Found!"));

  setReports(reportType, reportIntervalUs);

  Serial.println(F("after SPI here"));
  SPI.begin();

  Serial.println(F("Starting loop ..."));
  delay(5000);
}


void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
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
    static long last = 0;
    long now = micros();

    Serial.print(gLoopCount);
    Serial.print("\t");

    Serial.print(now - last);
    Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);
    Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);
    Serial.print("\t");
    Serial.print(ypr.pitch);
    Serial.print("\t");
    Serial.print(ypr.roll);
    // Serial.print("\t");
    // Serial.println(sensorValue.un.tiltDetector.tilt);
    Serial.println();
    // gLoopCount++;
  }

  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;

  // Moteus::CurrentMode::Command current_cmd;

  cmd.position = NaN;
  cmd.velocity = 0.2 * ::sin(time / 1000.0);

  // moteus1.SetCurrent(current_cmd);
  moteus1.SetPosition(cmd);
  // moteus2.SetBrake();
  if (gLoopCount % 5 != 0) {
    return;
  }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print(static_cast<int>(query.mode));
    Serial.print(F("  position "));
    Serial.print(query.position);
    Serial.print(F("  velocity "));
    Serial.print(query.velocity);
    Serial.print(F("  temperature "));
    Serial.print(query.temperature);
  };

  print_moteus(moteus1.last_result().values);
  // print_moteus(moteus2.last_result().values);
  Serial.println();

  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
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
