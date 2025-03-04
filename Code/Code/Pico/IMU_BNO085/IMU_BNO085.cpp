#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "bno08x.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "math.h"

float rad2deg(float);

int main()
{
    cyw43_arch_init();
    stdio_init_all();

    const uint SDA_PIN = 4;
    const uint SCL_PIN = 5;

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_init(i2c0, 100*1000);

    // spi_init()

    sleep_ms(1000);

    printf("I2C initialized");

    BNO08x IMU;

    // set up IMU
    while (IMU.begin(BNO08x_DEFAULT_ADDRESS, i2c0) == false)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
        
        // scan_i2c_bus();
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(1000);
    }

    IMU.enableRotationVector();

    while (true)
    {
    
        float roll, pitch, yaw = 0.0f;
        float x_accel, y_accel, z_accel = 0.0f;

        if (IMU.getSensorEvent() == true)
        {
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
            {
                roll = rad2deg(IMU.getRoll());
                yaw = rad2deg(IMU.getYaw());
                pitch = rad2deg(IMU.getPitch());

                z_accel = IMU.getAccelZ();

            }
        }

        printf("Roll: %.2f    ", roll); 
        printf("Pitch: %.2f    ", pitch); 
        printf("Yaw: %.2f   ", yaw);

        printf("x_accel: %.2f   ", x_accel);
        printf("y_accel: %.2f   ", y_accel);
        printf("z_accel: %.2f\n", z_accel);

        sleep_ms(20);
    }
}

float rad2deg(float rad){
    return rad*180/M_PI;
}