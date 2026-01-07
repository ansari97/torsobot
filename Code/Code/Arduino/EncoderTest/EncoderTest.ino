#include "Arduino.h"
#include "PicoEncoder.h"

// declare two encoders that will actually be reading the same pins:
// one will measure the phase sizes and compensate for differences, while
// the other will work as if the phase sizes were perfectly uniform
PicoEncoder encoder;
PicoEncoder non_calibrated_encoder;

// must be consecutive pins on the rp2040
const int encoder_pinA = 16;
const int encoder_pinB = 17;

uint32_t raw_encoder_val;
int32_t signed_endoder_val;

const int ENCODER_PPR = 2048;
const int ENCODER_CPR = ENCODER_PPR * 4;

// keep track of current time
uint period_start_us;

float angle;

void setup() {
  Serial.begin(115200);

  // use the same pins for both the calibrated and non-calibrated encoders
  while (!(encoder.begin(encoder_pinA))) {
    Serial.println("Encoder not found!");
  }
  Serial.println("Encoder found!");
  // non_calibrated_encoder.begin(encoder_pinA);

  delay(2000);

  period_start_us = time_us_32();
}

void loop() {
  static int count;

  count++;

  // wait for the next sample period, while calling "autoCalibratePhases"
  // to measure the phase sizes dynamically. If we had two encoders (to
  // read two motors, for instance), we could call both autoCalibratePhases
  // methods here, to calibrate both encoders simultaneously
  while ((int)(time_us_32() - period_start_us) < 10000)
    encoder.autoCalibratePhases();
  period_start_us += 10000;

  encoder.update();
  signed_endoder_val = -static_cast<int32_t>(encoder.step);
  angle = static_cast<float>(signed_endoder_val) / ENCODER_CPR * 360;

  Serial.print("count:  ");
  Serial.print(count);

  Serial.print(", step: ");
  Serial.print(signed_endoder_val);

  Serial.print(", angle: ");
  Serial.println(angle);
}
