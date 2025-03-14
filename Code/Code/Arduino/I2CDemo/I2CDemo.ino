#include <Wire.h>
#include <cstdlib>

#define PICO_SLAVE_ADDRESS 0x30
const int SENSOR_1_CMD = 0x1;
const int SENSOR_2_CMD = 0x2;

const int data_len = sizeof(float);  // 4 bytes
char sensor_data[data_len];
char sensor_data2[data_len];

char write_buff[data_len + 10];
char write_buff2[data_len + 10];

char read_command;

float sensor_val;
float sensor_val2;

float minVal = 180.0;
float maxVal = 0.0;

float minVal2 = 780.0;
float maxVal2 = 1000.0;

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
  sensor_val2 = minVal + ((float)rand() / RAND_MAX) * (maxVal - minVal);

  sprintf(write_buff, "%.2f", sensor_val);    // Format float to string
  sprintf(write_buff2, "%.2f", sensor_val2);  // Format float to string

  memcpy(sensor_data, &sensor_val, sizeof(float));
  memcpy(sensor_data2, &sensor_val2, sizeof(float));

  Serial.print(write_buff);
  Serial.print("\t");
  Serial.print(write_buff2);

  Serial.print("\t");
  Serial.println(i);
  delay(1000);
}


// Called when the I2C slave gets written to
void recv(int len) {
  read_command = Wire.read();
}


// Called when the I2C slave is read from
void req() {
  i++;
  switch (read_command) {
    case SENSOR_1_CMD:
      Wire.write(sensor_data, sizeof(sensor_data));
      break;
    case SENSOR_2_CMD:
      Wire.write(sensor_data2, sizeof(sensor_data2));
      break;
  }
}
