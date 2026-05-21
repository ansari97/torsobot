// Motor test with Pico 2W

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

#include "PicoEncoder.h"

PicoEncoder encoder;

uint32_t raw_encoder_val;
int32_t signed_encoder_val;

// Encoder variables
const float ENCODER_PPR = 2048;               // puses per rev
const float ENCODER_CPR = ENCODER_PPR * 4;    // counts per rev
const float ENCODER_uSPR = ENCODER_CPR * 64;  //usteps per rev
float angle = 0, prev_angle = 0;
float speed;

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CAN Controller board.
//——————————————————————————————————————————————————————————————————————————————

#define MCP2517_INT 9  // INT output of MCP2517
#define MCP2517_SCK 10
#define MCP2517_MOSI 11
#define MCP2517_MISO 12
#define MCP2517_CS 13  // CS input of MCP2517

// State LED
#define OK_LED 18
#define STOP_LED 19

static uint32_t gNextSendMillis = 0;

// driveshaft encoder
// must be consecutive pins on the rp2040
#define encoder_pinA 15
#define encoder_pinB 16

// Reed switch
#define REED_SWITCH 20

const float gear_ratio = (38.0 / 16.0) * (48.0 / 16.0);

// CORE 1

// dual core variables
volatile bool encoder_initialized = false;
volatile bool core1_setup_complete = false;

int32_t local_signed_encoder_steps;
volatile int32_t signed_encoder_steps;
float local_encoder_ang;
volatile float encoder_ang;

float local_wheel_rel_vel;
volatile float wheel_rel_vel;
int32_t local_encoder_speed;
volatile int32_t encoder_speed;


// FUNCTIONS
float encoderCountsToRad(int32_t local_signed_encoder_steps);
void emergencyStop(void);
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

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(STOP_LED, OUTPUT);
  pinMode(REED_SWITCH, INPUT);


  delay(500);

  // Let the world know we have begun!

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

  // while (!Serial) {}
  Serial.println("started");

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
  digitalWrite(STOP_LED, LOW);

  SPI1.setCS(MCP2517_CS);
  SPI1.setTX(MCP2517_MOSI);
  SPI1.setRX(MCP2517_MISO);
  SPI1.setSCK(MCP2517_SCK);

  SPI1.begin();

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

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus.SetStop();
  Serial.println("all stopped");

  digitalWrite(OK_LED, HIGH);

  delay(1000);
}

uint16_t gLoopCount = 0;
uint16_t current_time = 0, prev_time = 0;

void loop() {

  if (!digitalRead(REED_SWITCH)) {
    Serial.println("Reed switch not detected!");

    emergencyStop();
  }


  local_signed_encoder_steps = signed_encoder_steps;
  local_encoder_ang = encoder_ang;
  // get speed from core1
  local_encoder_speed = encoder_speed;
  local_wheel_rel_vel = wheel_rel_vel;

  current_time = millis();


  // encoder.autoCalibratePhases();
  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;
  // cmd.position = NaN;
  // cmd.velocity = 2 * ::sin(time / 1000.0);
  cmd.position = NaN;
  cmd.velocity = 1;  // (2 * (::pow(PI, 2) / 10) * ::cos(2 * PI * (time/1000) / 10) * gear_ratio) / (2 * PI);  //1 cycle in 1 minute at the wheel

  moteus.SetPosition(cmd);

  if (gLoopCount % 5 != 0) { return; }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);
  Serial.print("\t");

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print("mode: ");
    Serial.print(static_cast<int>(query.mode));
    Serial.print("\tposition: ");
    Serial.print(query.position);
    Serial.print("\tvelocity: ");
    Serial.print(query.velocity);
  };

  // // print_moteus(moteus.last_result().values);
  // signed_encoder_val = -static_cast<int32_t>(encoder.step);
  // angle = static_cast<float>(signed_encoder_val) / ENCODER_CPR * 2 * PI;

  // speed = (angle - prev_angle) * 1000 / (current_time - prev_time);
  // prev_angle = angle;
  // prev_time = current_time;

  // print_moteus(moteus.last_result().values);
  // print_moteus(moteus2.last_result().values);
  Serial.print(F(" wheel steps: "));
  Serial.print(local_signed_encoder_steps);
  Serial.print(F(" wheel angle (rad): "));
  Serial.print(local_encoder_ang);
  Serial.print(F(" encoder speed (usteps/s): "));
  Serial.print(local_encoder_speed, 10);  // / ENCODER_uSPR) * 2 * PI);
  Serial.print(F(" wheel velocity (rad/s): "));
  Serial.print(local_wheel_rel_vel, 10);  // / ENCODER_uSPR) * 2 * PI);
  // Serial.println();
  Serial.println();
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

const uint32_t encoder_update_period = 5000;

void loop1() {
  // update encoder value
  static float prev_encoder_ang;
  static uint32_t prev_time;

  static bool is_initialized = false;
  static uint32_t time_period;

  // update values
  encoder.update();

  signed_encoder_steps = -static_cast<int32_t>(encoder.step);  // sign depends on pin connections for channel A and B
  encoder_ang = encoderCountsToRad(signed_encoder_steps);      //encoder angle in rad

  if (!is_initialized) {
    prev_encoder_ang = 0;
    time_period = micros();
    prev_time = micros();
    is_initialized = true;
  }

  if (micros() - time_period < encoder_update_period) return;
  time_period += encoder_update_period;

  // wheel vel by finite difference
  uint32_t time_now = micros();
  uint32_t time_diff_micros = time_now - prev_time;

  wheel_rel_vel = ((encoder_ang - prev_encoder_ang) / time_diff_micros) * 1000 * 1000;
  prev_encoder_ang = encoder_ang;
  prev_time = time_now;
}



// encoder counts (or steps) to radians
float encoderCountsToRad(int32_t local_signed_encoder_steps) {
  return static_cast<float>(local_signed_encoder_steps) / ENCODER_CPR * 2 * PI;
}

// called when program has to stop
void emergencyStop(void) {
  // robot_run_status = 0xFF;  // true
  Serial.println("!!! Stopping program! :(");

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
