#define OK_LED 18
#define REED_SWITCH 20


void setup() {
  // put your setup code here, to run once:
  pinMode(OK_LED, OUTPUT);
  digitalWrite(OK_LED, LOW);
  pinMode(REED_SWITCH, INPUT);

  Serial.begin(115200);
  delay(100);

  Serial.println("Starting torsobot ...");
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Starting torsobot ...");

  if (digitalRead(REED_SWITCH) == LOW) {
    digitalWrite(OK_LED, HIGH);
  } else {
    digitalWrite(OK_LED, LOW);
  }
}
