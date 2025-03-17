#include <Wire.h>
#include <cstdlib>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
// #include <ACAN2517FD.h>
// #include <Moteus.h>

// For BNO085 SPI mode, we need a CS pin
#define BNO08X_SCK 2
#define BNO08X_MOSI 3
#define BNO08X_MISO 4
#define BNO08X_CS 5
#define BNO08X_INT 10
// For SPI mode, we also need a RESET
#define BNO08X_RESET 11
#define BNO08X_FAST_MODE

// I2C
#define PICO_SLAVE_ADDRESS 0x30

// Sensor value cmd bytes
const int SENSOR_1_CMD = 0x1;
const int SENSOR_2_CMD = 0x2;

const int data_len = sizeof(float);  // 4 bytes
char sensor_data[data_len];
char sensor_data2[data_len];

char write_buff[data_len + 10];
char write_buff2[data_len + 10];

char read_command;

float sensor_val;
float sensor_val2;

float minVal = 180.0;
float maxVal = 0.0;

float minVal2 = 780.0;
float maxVal2 = 1000.0;

int i = 0;

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

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Staring I2C slave...Adafruit BNO08x Test");

  Wire.setSDA(0);
  Wire.setSCL(1);

  Wire.begin(PICO_SLAVE_ADDRESS);

  Wire.onReceive(recv);
  Wire.onRequest(req);

  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println(F("Failed to find BNO08x chip"));
    while (1) { delay(10); }
  }
  Serial.println(F("BNO08x Found!"));

  setReports(reportType, reportIntervalUs);
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

    Serial.print(gLoopCount++);
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
    Serial.print("\t");
    Serial.println(i);
    // Serial.println(sensorValue.un.tiltDetector.tilt);
    Serial.println();
    // gLoopCount++;
  }

  memcpy(sensor_data, &ypr.pitch, sizeof(float));

  delay(5);
}


// Called when the I2C slave gets written to
void recv(int len) {
  read_command = Wire.read();
}


// Called when the I2C slave is read from
void req() {
  i++;
  switch (read_command) {
    case SENSOR_1_CMD:
      Wire.write(sensor_data, sizeof(sensor_data));
      break;
    case SENSOR_2_CMD:
      Wire.write(sensor_data2, sizeof(sensor_data2));
      break;
  }
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
