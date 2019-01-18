#!/usr/bin/env python3
import os
import sys
import time

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from ev3dev2.sensor.lego import GyroSensor, TouchSensor

touch = TouchSensor()

gyro = GyroSensor()
gyro.mode = gyro.MODE_GYRO_RATE
gyro_file = open(gyro._path + "/value0", "rb")

left = LargeMotor(OUTPUT_D)
left.duty_cycle_sp = 0
left.command = left.COMMAND_RUN_DIRECT
left_rate = open(left._path + "/speed", "rb")
left_dc = open(left._path + "/duty_cycle_sp", "w")

right = LargeMotor(OUTPUT_A)
right.duty_cycle_sp = 0
right.command = right.COMMAND_RUN_DIRECT
right_rate = open(right._path + "/speed", "rb")
right_dc = open(right._path + "/duty_cycle_sp", "w")

alpha = 0.95
dc_gain = 2.0
p_gain = 1.00
i_gain = 0.03
d_gain = 0.05
target_dt = 30 / 1000

def fast_read(infile):
    infile.seek(0)
    return(int(infile.read().decode().strip()))

def fast_write(outfile, value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()

def rate():
    return fast_read(gyro_file)

def state():
    gyro_rate = -rate()
    motor_rate = -((fast_read(right_rate) + fast_read(left_rate)) / 2)
    print("g_rate=%+0.0f\tm_rate=%+0.0f" % (gyro_rate, motor_rate), file=sys.stderr, end='\t')
    return alpha * gyro_rate + (1 - alpha) * motor_rate

def set_dc(dc):
    dc = dc * dc_gain
    dc = max(-100, dc) if dc < 0 else min(100, dc)
    fast_write(left_dc, dc)
    fast_write(right_dc, dc)

def balance():
    cur_time = time.time()
    cur_state = state()
    integral = 0
    dt_sum = 0
    n_iter = 0

    while touch.is_released:
        prev_time = cur_time
        cur_time = time.time()
        actual_dt = target_dt if n_iter == 0 else cur_time - prev_time

        n_iter += 1
        dt_sum += actual_dt

        prev_state = cur_state
        cur_state = state()

        integral += cur_state * actual_dt
        derivative = (cur_state - prev_state) / actual_dt
        output = p_gain * cur_state + i_gain * integral + d_gain * derivative

        set_dc(output)

        dc = output * dc_gain
        dc = max(-100, dc) if dc < 0 else min(100, dc)
        print("st=%+0.0f\ti=%+0.1f\td=%+0.0f\tdc=%+0.2f" % (cur_state, integral, derivative, dc), file=sys.stderr)

        while time.time() - cur_time < target_dt:
            time.sleep(0.001)

    set_dc(0)
    print("actual = %.3f, target = %.3f" % (dt_sum / n_iter, target_dt), file=sys.stderr)
    gyro_file.close()
    left_rate.close()
    left_dc.close()
    right_rate.close()
    right_dc.close()

def main():
#    os.system("setfont Lat15-Terminus24x12")
#    print("calibrate ready")
#    touch.wait_for_pressed()
#    touch.wait_for_released()
#    print("calibrate start")
#    calibrate()

    print("balance ready")
    touch.wait_for_pressed()
    touch.wait_for_released()
    time.sleep(0.5)
    print("balance start")
    balance()

main()
