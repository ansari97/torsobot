#include "Arduino.h"
#include "PicoEncoder.h"

// declare two encoders that will actually be reading the same pins:
// one will measure the phase sizes and compensate for differences, while
// the other will work as if the phase sizes were perfectly uniform
PicoEncoder encoder;
PicoEncoder non_calibrated_encoder;

// must be consecutive pins on the rp2040
const int encoder_pinA = 15;
const int encoder_pinB = 16;

uint32_t raw_encoder_val;
int32_t signed_encoder_val;

const float ENCODER_PPR = 2048.0;
const float ENCODER_CPR = ENCODER_PPR * 4.0;

// keep track of current time
uint period_start_us;

float angle;

void setup() {
  Serial.begin(115200);

  delay(2000);

  // use the same pins for both the calibrated and non-calibrated encoders
  if (encoder.begin(encoder_pinA)) {
    // If begin() returns FALSE (Failure), we enter this block
    Serial.println("Encoder NOT found! System Halted.");
    while (1) {
      // Freeze here so we don't try to run a broken robot
      delay(100);
    }
  }
  Serial.println("Encoder found!");
  // non_calibrated_encoder.begin(encoder_pinA);

  delay(2000);

  period_start_us = time_us_32();
}

void loop() {
  static int count;

  count++;

  // // wait for the next sample period, while calling "autoCalibratePhases"
  // // to measure the phase sizes dynamically. If we had two encoders (to
  // // read two motors, for instance), we could call both autoCalibratePhases
  // // methods here, to calibrate both encoders simultaneously
  // while ((int)(time_us_32() - period_start_us) < 10000)
  //   // encoder.autoCalibratePhases();
  // period_start_us += 10000;

  encoder.update();
  signed_encoder_val = -static_cast<int32_t>(encoder.step);
  angle = static_cast<float>(signed_encoder_val) / ENCODER_CPR * 360;

  Serial.print("count:  ");
  Serial.print(count);

  Serial.print(", step: ");
  Serial.print(signed_encoder_val);

  Serial.print(", angle: ");
  Serial.println(angle);
}
