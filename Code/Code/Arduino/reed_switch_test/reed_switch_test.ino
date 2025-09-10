#include <Arduino.h>

// Reed switch
#define REED_SWITCH 20

// State LED
#define OK_LED 18
#define STOP_LED 19

void setup() {
  pinMode(REED_SWITCH, INPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(STOP_LED, OUTPUT);

  digitalWrite(OK_LED, LOW);
  digitalWrite(STOP_LED, LOW);
}

void loop() {

  if (digitalRead(REED_SWITCH)) {
    digitalWrite(OK_LED, HIGH);
    digitalWrite(STOP_LED, LOW);
  } else {
    digitalWrite(OK_LED, LOW);
    digitalWrite(STOP_LED, HIGH);
  }
  delay(20);
}
