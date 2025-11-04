# BNO08x MicroPython I2C Test
#
# This program set up an I2C connection to the BNO08x sensor


from time import sleep

from bno08x import *
from machine import I2C, Pin
from utime import ticks_ms, sleep_us

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)

bno = BNO08X(i2c0, debug=True)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_GYROSCOPE,20 )
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 10)
bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

while True:
    #time.sleep(0.5)
    cpt += 1
    print("cpt", cpt)
    accel_x, accel_y, accel_z = bno.acc
    print("Acceleration\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tm/s²".format(accel_x, accel_y, accel_z))
    gyro_x, gyro_y, gyro_z = bno.gyro
    print("Gyroscope\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\trads/s".format(gyro_x, gyro_y, gyro_z))
    mag_x, mag_y, mag_z = bno.mag
    print("Magnetometer\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tuT".format(mag_x, mag_y, mag_z))
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print("Rot Vect Quat\tI: {:+.3f}\tJ: {:+.3f}\tK: {:+.3f}\tReal: {:+.3f}".format(quat_i, quat_j, quat_k, quat_real))
    R, T, P = bno.euler
    print("Euler Angle\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}".format(R, T, P))
    print("===================================")
    print("average delay times (ms) :", average_delay)
    print("===================================")
    timer = ticks_ms()
    if cpt == 10 :
        bno.tare
    if cpt % 100 == 0:
        average_delay = (timer - timer_origin) / cpt