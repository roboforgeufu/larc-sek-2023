#!/usr/bin/env pybricks-micropython

# pylint: skip-file

WHEEL_DIAMETER = 5.5

robot = Robot(
    wheel_diameter=const.WHEEL_DIAMETER,
    wheel_distance=const.WHEEL_DIST,
    motor_r=Port.C,
    motor_l=Port.B,
    motor_claw=Port.A,
    color_l=Port.S1,
    color_r=Port.S2,
    ultra_front=Port.S4,
    color_max_value=65,
    turn_correction=0.97,
    debug=True,
)
