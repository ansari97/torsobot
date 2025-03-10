#include <iostream>
// #include <pigpio.h>
// #include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

using namespace std;

const uint spiChannel = 1;
const uint spiSpeed = 500 * 1000; // hertz

unsigned char spiData[1];

int spiReturnVal;

int main()
{
    while (wiringPiSetupPinType(WPI_PIN_BCM) != 0)
    {
        cout << "Error initializing wiringPi" << endl;
        delay(100);
    };

    cout << "Initialized wiringPi" << endl;

    while (wiringPiSPISetup(spiChannel, spiSpeed) < 0)
    {
        cout << "Error initializing wiringPi" << endl;
        delay(100);
    };

    cout << "Initialized SPI" << endl;

    spiData[0] = 0b11010000;
    // spiData[1] = 0;
    // spiData[2] = 0;

    // cout << static_cast<int>(spiData[0]) << endl;
    cout << static_cast<int>(spiData[0]) << endl;

    spiReturnVal = wiringPiSPIDataRW(spiChannel, spiData, 1);
    cout << spiReturnVal << endl;

    cout << static_cast<int>(spiData[0]) << endl;

    if (spiReturnVal <= 0)
        cout << "SPI transfer error" << endl;
    else
    {
        cout << "SPI transfer successful" << endl;
        cout << spiData[0] << endl;
    }

    return 0;
}
