#include <Wire.h>
#include <cstdlib>

#define PICO_SLAVE_ADDRESS 0x30

const int data_len = sizeof(float);  // 4 bytes
char sensor_data[data_len];

char write_buff[data_len];

float sensor_val;

float minVal = 180.0;
float maxVal = 0.0;

int i = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Staring I2C slave...");

  Wire.setSDA(0);
  Wire.setSCL(1);

  Wire.begin(PICO_SLAVE_ADDRESS);

  Wire.onReceive(recv);
  Wire.onRequest(req);
}

void loop() {
  //Random number
  sensor_val = minVal + ((float)rand() / RAND_MAX) * (maxVal - minVal);
  sprintf(write_buff, "%.2f", sensor_val);  // Format float to string

  Serial.println(write_buff);
  memcpy(write_buff, &sensor_val, sizeof(float));
  Serial.println(i);
  delay(1000);
}


// Called when the I2C slave gets written to
void recv(int len) {
  //Do nothing
}


// Called when the I2C slave is read from
void req() {
  i++;
  Wire.write(write_buff, sizeof(write_buff));
}
