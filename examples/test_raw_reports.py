# test_raw_reports.py
#
# BNO08x Micropython I2C example program
# Raw device reports: raw_accelerometer, raw_magnetic, raw_gyro
#  - raw_accelerometer, raw_magnetic -  return 3 values and time_stamp
#  - raw_gyro - returnn 3 values, Celsius, and time_stamp
#
# note: timestamp is not clearly described in Ceva documentation
#
# Enabling reports at default frequencies
# Raw sensors will automatically enable additional required reports

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

address = 0x4b
i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("I2C devices found:", [hex(d) for d in i2c0.scan()])

print("Start")
print("====================================\n")

bno.raw_acceleration.enable()
bno.raw_magnetic.enable()
bno.raw_gyro.enable()

# sensor default frequencies
bno.print_report_period()

while True:
    # Update required each loop to check if any sensor updated, print sensor data (some or all may be old data)
    # see test_reports_full_spi.py, for example of only printing a sensor when it is updated
    bno.update_sensors()
        
    accel_x, accel_y, accel_z, ts_us = bno.raw_acceleration
    print(f"\nRaw Acceleration:  X: {accel_x:#06x}  Y: {accel_y:#06x}  Z: {accel_z:#06x} {ts_us=}")

    mag_x, mag_y, mag_z, ts_us = bno.raw_magnetic
    print(f"Raw Magnetometer:  X: {mag_x:#06x}  Y: {mag_y:#06x}  Z: {mag_z:#06x} {ts_us=}")

    gyro_x, gyro_y, gyro_z, celsius, ts_us = bno.raw_gyro
    print(f"Raw Gyroscope:     X: {gyro_x:#06x}  Y: {gyro_y:#06x}  Z: {gyro_z:#06x} {celsius=} {ts_us=}")
