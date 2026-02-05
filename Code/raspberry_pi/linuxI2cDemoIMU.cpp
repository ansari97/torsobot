#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

using namespace std;

#define PICO_SLAVE_ADDRESS 0x30
const int I2C_BUS = 1;
int i2c_handle;

const int data_len = sizeof(float);
uint8_t read_buff[data_len]; // Use uint8_t for byte buffer
float sensor_val2;

float IMU_val;

const int IMU_CMD = 0x01;
const int SENSOR_2_CMD = 0x02;

int GetSensorValue(int, float *);

int main() {
    // Open I2C bus
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);

    if ((i2c_handle = open(filename, O_RDWR)) < 0) {
        cerr << "Failed to open the i2c bus" << endl;
        return 1;
    }

    // Specify slave address
    if (ioctl(i2c_handle, I2C_SLAVE, PICO_SLAVE_ADDRESS) < 0) {
        cerr << "Failed to acquire bus access and/or talk to slave" << endl;
        close(i2c_handle);
        return 1;
    }

    cout << "Successfully opened I2C: " << i2c_handle << endl;

    // Read/request sensor data from pico
    while (true) {
        int get_sensor_val_result1 = GetSensorValue(IMU_CMD, &IMU_val);

        if (get_sensor_val_result1 <= 0) {
            close(i2c_handle);
            return 0; // Return 0 on error, or an error code as you prefer
        }

        cout << "Sensor 1 data: " << IMU_val << endl;

        usleep(1000 * 1000);
    }
}

// Get sensor data over I2C
int GetSensorValue(int cmd, float *sensor_val_addr) {
    // Clear the read buffer
    memset(read_buff, 0, data_len);

    // Write command to slave
    uint8_t commandByte = static_cast<uint8_t>(cmd);
    if (write(i2c_handle, &commandByte, 1) != 1) {
        cerr << "I2C write failed! " << endl;
        return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, data_len) != data_len) {
        cerr << "I2C read failed! " << endl;
        return -2; // Indicate error
    } else {
        memcpy(sensor_val_addr, read_buff, data_len);
    }

    return 1; // Success
}