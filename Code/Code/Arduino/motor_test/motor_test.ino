// Motor test with Pico 2W

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CAN Controller board.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 10;  // SCK input of MCP2517
static const byte MCP2517_SDI = 11;  // SDI input of MCP2517
static const byte MCP2517_SDO = 12;  // SDO output of MCP2517

static const byte MCP2517_CS = 13;  // CS input of MCP2517
static const byte MCP2517_INT = 9;  // INT output of MCP2517

static uint32_t gNextSendMillis = 0;

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

  delay(5000);

  // Let the world know we have begun!
  Serial.begin(115200);
  // while (!Serial) {}
  Serial.println("started");

  SPI1.setCS(MCP2517_CS);
  SPI1.setTX(MCP2517_SDI);
  SPI1.setRX(MCP2517_SDO);
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

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus.SetStop();
  Serial.println("all stopped");

  delay(5000);
}

uint16_t gLoopCount = 0;

void loop() {
  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;
  cmd.velocity = 2 * ::sin(time / 1000.0);

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

  print_moteus(moteus.last_result().values);
  Serial.println();
}
