// Motor test with Pico 2W

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CAN Controller board.
//——————————————————————————————————————————————————————————————————————————————

#define MCP2517_INT 9  // INT output of MCP2517
#define MCP2517_SCK 10
#define MCP2517_MOSI 11   
#define MCP2517_MISO 12
#define MCP2517_CS 13  // CS input of MCP2517

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

void softwareResetMCP2517FD(void);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(MCP2517_CS, OUTPUT);

  delay(5000);

  // Let the world know we have begun!
  Serial.begin(115200);
  // while (!Serial) {}
  Serial.println("started");

  SPI1.setSCK(MCP2517_SCK);
  SPI1.setMOSI(MCP2517_MOSI);
  SPI1.setMISO(MCP2517_MISO);
  SPI1.setCS(MCP2517_CS);

  // digitalWrite(MCP2517_CS, HIGH);

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
    while(1){
      delay(10);
    }
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

  // Only print our status every 5th cycle, so every 100ms.

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
    Serial.print("\tvoltage: ");
    Serial.print(query.voltage);
  };

  print_moteus(moteus.last_result().values);
  Serial.println();
}

// --- THE FIX: Software Reset Function ---
void softwareResetMCP2517FD() {
  Serial.println("Forcing Software Reset of MCP2517FD...");
  digitalWrite(MCP2517_CS, LOW);
  SPI1.transfer(0x00); 
  SPI1.transfer(0x00);
  digitalWrite(MCP2517_CS, HIGH);
  delay(10); 
}
// ----------------------------------------
