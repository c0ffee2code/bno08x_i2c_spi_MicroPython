# test_tare.py
#
# BNO08x Micropython I2C example program
# measure quaternion, use euler_conversion, tare the sensor, and show new orientation
#
# Rotation Vector to use as basis for tare.
#   0: quaternion
#   1: game_quaternion
#   2: geomagnetic_quaternion
#   3: Gyro-Integrated Rotation Vector  (not implemented)
#   4: ARVR-Stabilized Rotation Vector (not implemented)
#   5: ARVR-Stabilized Game Rotation Vector  (not implemented)

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

address = 0x4b
i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("I2C devices found:", [hex(d) for d in i2c0.scan()])

print("Start")
print("===========================")

bno.quaternion.enable(100)
bno.print_report_period()

print("\n\n*** Starting Countdown timer for 10 seconds, then tare the sensor\n")
start = ticks_ms()
secs = 10
while secs > 0:
    bno.update_sensors()

    if ticks_diff(ticks_ms(), start) < 1000:
        continue

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={secs}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")

    start = ticks_ms()
    secs -= 1

# Tare the orientation
axis = 0x07  # tare all Axis (z, y, x)
basis = 0  # Quaternion
bno.tare(axis, basis)

print(f"\n\n*** Tared the sensor axis=({hex(axis)}), basis={basis})\n")

# show the new orientation based on tare for 7 seconds
start = ticks_ms()
secs = 7
while secs > 0:
    bno.update_sensors()

    if ticks_diff(ticks_ms(), start) < 1000:
        continue

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={secs}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")

    start = ticks_ms()
    secs -= 1

# Exited loop
bno.save_tare_data()
print("\n\t*** Tare saved (from 7 seconds ago)")
