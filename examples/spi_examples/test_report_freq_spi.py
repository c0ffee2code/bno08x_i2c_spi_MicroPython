# BNO08x MicroPython SPI Test
#
# This program set up an SPI connection to the BNO08x sensor
from time import sleep
from utime import sleep_ms, ticks_ms

#from bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_MAGNETOMETER
from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# spi0_RX = Pin(16, Pin.IN)  # spi0_RX, RES (MISO) - connected to BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)  # cs for SPI
# sck = Pin(18, Pin.OUT, value=0)  # sck for SPI
# spi0_TX = Pin(19, Pin.OUT, value=0)  # spi0_TX (MOSI) - connected to BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # Wakes BNO to enable INT response

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=3_000_000)

print("Start")
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)
print(spi)  # polarity=1, phase=1 for bno08x
print("====================================\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 500)
#bno.enable_feature(BNO_REPORT_MAGNETOMETER, 100)

bno.print_report_period()
print("BNO08x sensors enabled\n")

cpt = 0

start = ticks_ms()
last_acc_ms = 0
last_mag_ms = 0
running_sum = 0.0

# ignore first call timing
accel_x, accel_y, accel_z, acc, ts_us = bno.acceleration.full
last_acc_ms = ts_us/1000.0

while True:
    accel_x, accel_y, accel_z, acc, ts_us = bno.acceleration.full
#    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f} m/s², Acc={acc}, {ts_us/1000.0 - last_acc_ms:.1f} ms")
#    print(f"Accel {ts_us/1000.0 - last_acc_ms:.1f} ms")
    iter_time = ts_us/1000.0 - last_acc_ms
    running_sum += iter_time
    cpt +=1
    print(f"ave={running_sum/cpt:.1f}, current={iter_time:.1f}")
    # print(f"ave={running_sum/cpt:.1f}")


    last_acc_ms = ts_us/1000.0
    
#     mag_x, mag_y, mag_z, acc, ts_us = bno.magnetic.full
# #    print(f"Magnetometer  X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f} uT, Acc={acc}, {ts_us/1000.0 - last_mag_ms:.1f} ms")
#     print(f"Mag {ts_us/1000.0 - last_mag_ms:.1f} ms")
#     last_mag_ms = ts_us/1000.0
