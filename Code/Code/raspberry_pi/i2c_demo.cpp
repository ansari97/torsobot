#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <cstring>

using namespace std;

#define PICO_SLAVE_ADDRESS 0X30
const int i2c_bus = 1;

const int data_len = sizeof(float);
char read_buff[data_len];
float sensor_val;

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

    int i2c_handle = i2cOpen(i2c_bus, PICO_SLAVE_ADDRESS, 0);

    if (i2c_handle < 0)
    {
        cerr << "Failed to open I2C device: " << i2c_handle << endl;
        return 1;
    }
    cout << "Successfully opened I2C: " << i2c_handle << endl;

    // Read/request sensor data from pico
    int read_result = i2cReadDevice(i2c_handle, read_buff, data_len);

    if (read_result != data_len)
    {
        cerr << "I2C read failed! " << read_result << endl;
    }
    else
    {
        memcpy(&sensor_val, read_buff, data_len);
        cout << "Read data: " << sensor_val << endl;
    }
}
