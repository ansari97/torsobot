#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <cstring>

using namespace std;

#define PICO_SLAVE_ADDRESS 0X30
const int I2C_BUS = 1;
int i2c_handle;

const int data_len = sizeof(float);
char read_buff[data_len];
float sensor_val2;

// char read_buff1[data_len];
float sensor_val1;

const int SENSOR_1_CMD = 0x01;
const int SENSOR_2_CMD = 0x02;

int GetSensorValue(int, float *);

int main()
{
    // initialize pigpio
    int pi = gpioInitialise();
    if (pi < 0)
    {
        cerr << "Error initializing pigpio" << endl;
        usleep(100);
        return -1;
    };
    cout << "Initialized pigpio" << endl;

    i2c_handle = i2cOpen(I2C_BUS, PICO_SLAVE_ADDRESS, 0);

    if (i2c_handle < 0)
    {
        cerr << "Failed to open I2C device: " << i2c_handle << endl;
        return 1;
    }
    cout << "Successfully opened I2C: " << i2c_handle << endl;

    // Read/request sensor data from pico
    while (true)
    {
        int get_sensor_val_result1 = GetSensorValue(SENSOR_1_CMD, &sensor_val1);

        if (get_sensor_val_result1 <= 0)
        {
            return 0;
        }

        int get_sensor_val_result2 = GetSensorValue(SENSOR_2_CMD, &sensor_val2);
        cout << "Sensor 1 data: " << sensor_val1 << "\tSensor 2 data: " << sensor_val2 << endl;
        if (get_sensor_val_result2 <= 0)
        {
            return 0;
        }
        usleep(1000 * 1000);
    }
}

// Get sensor data over I2C
int GetSensorValue(int cmd, float *sensor_val_addr)
{
    // Clear the read buffer
    memset(read_buff, 0, data_len);

    // Write command to slave
    int write_request = i2cWriteByte(i2c_handle, cmd);
    if (write_request != 0)
    {
        cerr << "I2C write failed! " << write_request << endl;
        return write_request;
    }

    // Rrequest sensor data from slave
    int read_result = i2cReadDevice(i2c_handle, read_buff, data_len);

    if (read_result != data_len)
    {
        cerr << "I2C read failed! " << read_result << endl;
        return read_result;
    }
    else
    {
        memcpy(sensor_val_addr, read_buff, data_len);
        // cout << "Read data: " << *sensor_val_addr << endl;
    }

    return 1;
}
