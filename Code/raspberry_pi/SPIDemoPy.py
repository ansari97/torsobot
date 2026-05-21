import spidev
import time

#initalize SPI
spi = spidev.SpiDev()
spi.open(0,0)
spi.mode = 0
spi.max_speed_hz = 1000*1000

data_to_send = [1,2,3,4]

try:
    print("Sent: ", data_to_send)
    received_data = spi.xfer2(data_to_send)
    # print("Sent: ", data_to_send)
    print("Received: ", received_data)

except IOError as e:
    print(f"Error:{e}")

finally:
    spi.close()