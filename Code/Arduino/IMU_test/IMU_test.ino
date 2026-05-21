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

// SPI lines for the IMU and the CAN controller
#define BNO08X_SCK 2
#define BNO08X_MOSI 3
#define BNO08X_MISO 4
#define BNO08X_CS 5
#define BNO08X_INT 6
#define BNO08X_RESET 7

// Defining fast mode runs the IMU at a higher frequency but causes values to drift
// #define BNO08X_FAST_MODE

float imu_pitch;
float torso_pitch;
float imu_angle_adjustment = 0.0477;  // imu angle adjustment relative to COM in radians; adjusted in torso_pitch >> from the imu angle code


int ctr = 0;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

void quaternionToPitch(sh2_RotationVectorWAcc_t*, float*, bool degrees = false);
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool deegrees);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees);
void setReports(sh2_SensorId_t reportType, long report_interval);
float degToCycles(float);
int signof(float num);
float imuAngleCorrection(float pitch, float gravity_x, float gravity_y, float gravity_z);
float wrapTo2PI(float angle);

euler_t ypr;

uint16_t gLoopCount = 0;

//——————————————————————————————————————————————————————————————————————————————
//  BNO08X object
//——————————————————————————————————————————————————————————————————————————————

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool resetOccurred = false;

sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
long reportIntervalUs = 5000;

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(2000);

  Serial.println("Staring IMU test");

  SPI.setSCK(BNO08X_SCK);
  SPI.setMOSI(BNO08X_MOSI);
  SPI.setMISO(BNO08X_MISO);
  SPI.setCS(BNO08X_CS);


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

  Serial.println("wait...");

  delay(2000);
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
      case SH2_ROTATION_VECTOR:
        // torso_pitch is updated by reference
        // &sensorValue.un.rotationVector uses the magnetomoeter and causes x/y values to jump due to yaw corrections
        // arvrstabilizedrv does not use magnetometer
        quaternionToPitch(&sensorValue.un.arvrStabilizedRV, &torso_pitch);
        torso_pitch -= imu_angle_adjustment;
        torso_pitch = wrapTo2PI(torso_pitch);  // we need angles in the [0, 2*PI) range
        break;
    }
  }
  // // Serial.println();

  Serial.print("torso pitch: ");
  Serial.println(torso_pitch);
}

///////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////

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

void quaternionToPitch(sh2_RotationVectorWAcc_t* rotational_vector, float* pitch, bool degrees) {
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

// sign function
int signof(float num) {
  // returns +1 for pos, -1 for neg, and 0 for 0
  return (num > 0) - (num < 0);
}

// wrap angle from 0 to 360
float wrapTo2PI(float angle) {

  if (angle < 0) { angle += 2 * PI; }
  return angle;
}
