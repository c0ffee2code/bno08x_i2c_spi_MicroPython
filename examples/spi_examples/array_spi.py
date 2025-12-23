# BNO08x MicroPython SPI Test
#
# This program set up an SPI connection to the BNO08x sensor
from time import sleep
from utime import sleep_ms, ticks_ms, sleep_us

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

import struct

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)

print(spi)
print("Start")
print("====================================\n")

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, 1000)

bno.print_report_period()
print("BNO08x sensors enabled\n")

ROWS = 1000
BYTES_PER_ROW = 20   # 5 float32 values × 4 bytes
buffer = bytearray(ROWS * BYTES_PER_ROW)

bno.update_sensors()
quat_i, quat_j, quat_k, quat_real, acc, ts_ms = bno.quaternion.full
first_ms = ts_ms

start = ticks_ms()
for i in range(ROWS):
    bno.update_sensors()   
    quat_i, quat_j, quat_k, quat_real, acc, ts_ms = bno.quaternion.full
    print(f"I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f} {ts_ms:.1f}")

quat_i, quat_j, quat_k, quat_real, acc, ts_ms = bno.quaternion.full
last_ms = ts_ms
print(f"timestamp {last_ms} {first_ms}  {(last_ms-first_ms)=}")

print_time = ticks_diff(ticks_ms(),start)/1000.0
print(f"Print {ROWS} Seconds={print_time}")

vals = 0
_, _, _, _, _, ts_last = bno.quaternion.full
start = ticks_ms()
for i in range(ROWS):
    bno.update_sensors()    
    offset = i * BYTES_PER_ROW
    quat_i, quat_j, quat_k, quat_real, acc, ts_ms = bno.quaternion.full
    struct.pack_into("<fffff", buffer, offset, quat_i, quat_j, quat_k, quat_real, ts_ms)
    if ts_ms != ts_last:
        vals += 1
    ts_last = ts_ms
#    sleep_ms(4)
    
array_time = ticks_diff(ticks_ms(),start)/1000.0
print(f"\nArray {ROWS} Seconds={array_time}")
print(f"{vals=}")

kbytes = (BYTES_PER_ROW * ROWS) / 1024
print(f"{kbytes=}")
