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
float torso_pitch, torso_roll;
float imu_angle_adjustment = 4.0f * DEG_TO_RAD;  // imu angle adjustment relative to COM in radians; adjusted in torso_pitch >> from the imu angle code
float imu_scale_adjustment = PI / 2.9679;


int ctr = 0;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

void quaternionToPitch(sh2_RotationVector_t*, float*, float*, bool degrees = false);
void setReports(long report_interval);
void saveCalibration(void);

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

  Serial.println("Initializing IMU ...");
  // IMU spi communication
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(F("Failed to find BNO08x IMU chip"));
    while (1) delay(10);
  }
  Serial.println("BNO08x IMU Found!");
  digitalWrite(LED_BUILTIN, LOW);

  bno08x.hardwareReset();
  delay(1000);
  Serial.print("Sensor reset?\t");
  Serial.println(bno08x.wasReset());

  setReports(reportIntervalUs);

  Serial.println("wait...");
  Serial.println("Enabling Dynamic Calibration...");
  // Enable Accelerometer, Gyro, and Magnetometer calibration
  int status = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);

  if (status == SH2_OK) {
    Serial.println("Dynamic Calibration Enabled: Accel + Gyro + Mag");
  } else {
    Serial.print("Failed to enable calibration. Status: ");
    Serial.println(status);
  }

  Serial.println("CALIBRATION MODE: Move robot in full sagittal plane.");
  Serial.println("Type 's' in Serial Monitor to save calibration.");

  delay(2000);
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void loop() {

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') {
      saveCalibration();
    }
  }

  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset during loop!");
    setReports(reportIntervalUs);
    // resetOccurred = true;
  }

  if (bno08x.getSensorEvent(&sensorValue)) {

    switch (sensorValue.sensorId) {

      case SH2_GAME_ROTATION_VECTOR:
        // torso_pitch is updated by reference
        // &sensorValue.un.rotationVector uses the magnetomoeter and causes x/y values to jump due to yaw corrections
        // arvrstabilizedrv does not use magnetometer
        quaternionToPitch(&sensorValue.un.gameRotationVector, &torso_pitch, &torso_roll);
        torso_pitch -= imu_angle_adjustment;
        // torso_pitch *= imu_scale_adjustment;
        torso_pitch = wrapTo2PI(torso_pitch);  // we need angles in the [0, 2*PI) range
        break;
    }
  }

  // // Serial.println();

  Serial.print("torso pitch: ");
  Serial.print(torso_pitch, 4);
  Serial.print("\ttorso roll: ");
  Serial.println(torso_roll, 4);
}

///////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////

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

void saveCalibration() {
  Serial.println("Saving calibration to flash...");

  // sh2_saveDcdNow() commands the BNO085 to dump its RAM calibration to Flash
  int status = sh2_saveDcdNow();

  if (status == SH2_OK) {
    Serial.println("Calibration saved successfully!");
  } else {
    Serial.print("Failed to save calibration. Status: ");
    Serial.println(status);
    Serial.println("stopping program...");
    while (1) delay(10);
  }
}

void quaternionToPitch(sh2_RotationVector_t* rotational_vector, float* pitch, float* roll, bool degrees) {
  float qw = rotational_vector->real;
  float qx = rotational_vector->i;
  float qy = rotational_vector->j;
  float qz = rotational_vector->k;

  // Calculate the two components of the Rotation Matrix we care about
  // These represent the relationship between the Body's X/Z axes and World Gravity.

  // Projections of World Z (Gravity) onto Body axes
  float R31 = 2.0f * (qx * qz - qw * qy);         // Body X (Pitch sine)
  float R32 = 2.0f * (qy * qz + qw * qx);         // Body Y (Roll sine)
  float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);  // Body Z (Cosine)

  // Calculate the full 360-degree angle (atan2 handles all 4 quadrants automatically)
  // We use -R31 because standard pitch definition often opposes the matrix index direction
  *pitch = atan2(-R31, R33);
  *roll = atan2(R32, R33);

  // we could also use the atan2(-R31, sgn(R33) * sqrt(R11*R11 + R21*R21)); but this would cause jumps/singularities since R33 is changing sign and sqrt isapproacing 0 at the same time

  if (degrees) {
    *pitch *= RAD_TO_DEG;
    *roll *= RAD_TO_DEG;
  }
}


// wrap angle from 0 to 360
float wrapTo2PI(float angle) {

  if (angle < 0) { angle += 2 * PI; }
  return angle;
}
