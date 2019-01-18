#!/usr/bin/env python3
import os
import sys
import time

# TODO: implement fast read/write

from ev3dev.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from ev3dev.sensor.lego import GyroSensor, TouchSensor

touch = TouchSensor()

gyro = GyroSensor()
gyro.mode = gyro.MODE_GYRO_G_A
offset = 0

left = LargeMotor(OUTPUT_D)
left.duty_cycle_sp = 0
left.command = left.COMMAND_RUN_DIRECT

right = LargeMotor(OUTPUT_A)
right.duty_cycle_sp = 0
right.command = right.COMMAND_RUN_DIRECT

alpha = 0.5     # complementary filter
p_gain = 1.0    # proportional gain
i_gain = 1.0    # integral gain
d_gain = 1.0    # derivative gain
dt = 30 / 1000  # time between updates

def angle(): return gyro.value(0)       # raw angle
def theta(): return angle() + offset    # calibrated angle
def rate(): return gyro.value(1)        # angular velocity

def calibrate():
    global offset
    n = 50
    x = 0
    for _ in range(n):
        x += angle()
    offset = x / n

def state():
    motor_rate = (left.degrees_per_second + right.degrees_per_second) / 2
    return (1 - alpha) * rate() + alpha * motor_rate

def set_dc(dc):
    dc = max(-100, dc) if dc < 0 else min(100, dc)
    left.duty_cycle_sp = dc
    right.duty_cycle_sp = dc

def balance():
    cur_state = state()
    integral = 0
    dt_sum = 0
    n_iter = 0

    while touch.is_released and theta() < 40:
        start = time.time()

        prev_state = cur_state
        cur_state = state()
        integral += cur_state * dt
        derivative = (cur_state - prev_state) / dt

        set_dc(p_gain * cur_state + i_gain * integral + d_gain * derivative)

        while time.time() - start < dt:
            time.sleep(0.001)

        n_iter += 1
        dt_sum += time.time() - start

    set_dc(0)
    print("mean dt", dt_sum / n_iter)

def main():
    os.system("setfont Lat15-Terminus24x12")
    print("calibrate ready")
    touch.wait_for_pressed()
    touch.wait_for_released()
    print("calibrate start")
    calibrate()

    print("balance ready")
    touch.wait_for_pressed()
    touch.wait_for_released()
    print("balance start")
    balance()

main()




