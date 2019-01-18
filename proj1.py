#!/usr/bin/env python3
import os
import sys
import time

# TODO: implement fast read/write

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from ev3dev2.sensor.lego import GyroSensor, TouchSensor

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

dc_gain = 1.0
p_gain = 1.0
i_gain = 0.029
d_gain = 0.049
dt = 250 / 1000

def rate(): return gyro.value(1)

def state():
    return rate()

def set_dc(dc):
    dc = dc * dc_gain
    dc = max(-99, dc) if dc < 0 else min(99, dc)
    print("dc = %2.0f%%" % (dc), file=sys.stderr)
    #left.duty_cycle_sp = dc
    #right.duty_cycle_sp = dc

def balance():
    cur_state = state()
    integral = 0
    dt_sum = 0
    n_iter = 0

    while touch.is_released:
        start = time.time()

        prev_state = cur_state
        cur_state = state()
        integral += cur_state * dt
        derivative = (cur_state - prev_state) / dt
        output = p_gain * cur_state + i_gain * integral + d_gain * derivative

        print("cur_state =", cur_state, file=sys.stderr)
        print("integral =", integral, file=sys.stderr)
        print("derivative =", derivative, file=sys.stderr)
        print("output =", output, file=sys.stderr)

        set_dc(output)

        while time.time() - start < dt:
            time.sleep(0.001)

        n_iter += 1
        dt_sum += time.time() - start

    set_dc(0)
    print("actual = %.2f, target = %.2f" % (dt_sum / n_iter, dt), file=sys.stderr)

def main():
#    os.system("setfont Lat15-Terminus24x12")
#    print("calibrate ready")
#    touch.wait_for_pressed()
#    touch.wait_for_released()
#    print("calibrate start")
#    calibrate()

#    print("balance ready")
#    touch.wait_for_pressed()
#    touch.wait_for_released()
#    print("balance start")
    balance()

main()
