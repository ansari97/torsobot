#include <iostream>
// #include <pigpio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

using namespace std;

int main()
{

    wiringPiSetupGpio();

    wiringPiSPISetup(0, 1000*1000);

    while (true)
    {
        cout << "Hello World" << endl;
        usleep(1000 * 1000);
    }

    return 0;
}
