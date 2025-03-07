#include <iostream>
#include <pigpio.h>
#include <unistd.h>
// #include <wiringPi.h>
// #include <wiringPiSPI.h>

using namespace std;

const uint spiChannel = 1;
const uint spiSpeed = 500 * 1000; // hertz

const int data_len = 1;
char spiData[data_len];

int spiReturnVal;

int main()
{
    while (gpioInitialise() < 0)
    {
        cout << "Error initializing pigpio" << endl;
        usleep(100);
    };

    cout << "Initialized pigpio" << endl;
    int spiHandle;
    while (spiHandle = spiOpen(spiChannel, spiSpeed, 0) < 0)
    {
        cout << "Error initializing pigpio SPI" << endl;
        usleep(100);
    };

    cout << "Initialized SPI" << endl;

    spiData[0] = 0b11010000;
    // spiData[1] = 0;
    // spiData[2] = 0;

    // cout << static_cast<int>(spiData[0]) << endl;
    cout << static_cast<int>(spiData[0]) << endl;

    spiReturnVal = spiWrite(spiHandle, spiData, data_len);
    cout << spiReturnVal << endl;

    cout << static_cast<int>(spiData[0]) << endl;

    if (spiReturnVal != data_len)
        cout << "SPI transfer error" << endl;
    else
    {
        cout << "SPI transfer successful" << endl;
        cout << static_cast<int>(spiData[0]) << endl;
    }

    return 0;
}
