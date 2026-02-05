#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include <SPI.h>

// For SPI mode, we need a CS pin
#define BNO08X_SCK 2
#define BNO08X_MOSI 3
#define BNO08X_MISO 4
#define BNO08X_CS 5
#define BNO08X_INT 6
// For SPI mode, we also need a RESET
#define BNO08X_RESET 7

// #define FAST_MODE

// For SPI mode, we also need a RESET
// #define BNO08X_RESET 7
// but not for I2C or UART
// #define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

euler_t ypr;

unsigned int loop_no = 0;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
// sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);
  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);


  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  // if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

// wrap angle from 0 to 360
float wrapTo360(float angle) {

  if (angle < 0) { angle += 360; }
  return angle;
}

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

float imu_pitch;
float imu_pitch_calibration_constant;  // = (-176.57 + 180);  // degrees

// gets the angle between imu's x-axis and the global horizontal plane
void quaternionToPitch(sh2_RotationVectorWAcc_t* rotational_vector, float* pitch, bool degrees = false) {
  float qw = rotational_vector->real;
  float qx = rotational_vector->i;
  float qy = rotational_vector->j;
  float qz = rotational_vector->k;

  float sqx = sq(qx);
  float sqy = sq(qy);
  float sqz = sq(qz);

  float xy = qx * qy;
  float xz = qx * qz;
  float zw = qz * qw;
  float yw = qy * qw;

  // Rotation matrix terms
  float R11 = 1 - 2 * sqy - 2 * sqz;
  float R21 = 2 * (xy + zw);
  float R31 = 2 * (xz - yw);
  float R33 = 1 - 2 * sqx - 2 * sqy;

  // apparently the IMU treats global Z as up
  *pitch = atan2(-R31, signof(R33) * sqrt(sq(R11) + sq(R21)));

  if (degrees) {
    *pitch *= RAD_TO_DEG;
  }
}


void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      // case SH2_ARVR_STABILIZED_RV:
      //   quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_ROTATION_VECTOR:
        // quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);
        // quaternionToPitch(&sensorValue.un.rotationVector, &imu_pitch, true);
        // rotation vector uses the magnetomoeter and causes x/y values to jump due to corrections - arvrstabilizedrv does not use magnetometer
        quaternionToPitch(&sensorValue.un.arvrStabilizedRV, &imu_pitch, true);
        // imu_pitch -= imu_pitch_calibration_constant;
        imu_pitch = wrapTo360(imu_pitch);

        // to get the correction constant, allow torso to dnagle at the lower vertical and let the readings stabilize
        imu_pitch_calibration_constant = imu_pitch - 180;
        // quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }


    static long last = 0;
    long now = micros();

    Serial.print(loop_no);
    Serial.print("\t");

    Serial.print(now - last);
    Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);
    Serial.print("\t");  // This is accuracy in the range of 0 to 3
    // Serial.print(ypr.yaw);
    // Serial.print("\t");
    // Serial.print(ypr.pitch);
    // Serial.print("\t");
    Serial.print(imu_pitch, 4);
    Serial.print("\t");
    Serial.println(imu_pitch_calibration_constant, 4);
    Serial.print("\t");
    Serial.println(imu_pitch_calibration_constant / RAD_TO_DEG, 4);
    // Serial.print(ypr.roll);
    // Serial.print("\t");
    // Serial.println(sensorValue.un.tiltDetector.tilt);
    Serial.println();
    loop_no++;
  }
}

// sign function
int signof(float num) {
  return (num > 0) - (num < 0);
}
