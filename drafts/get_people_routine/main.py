#!/usr/bin/env pybricks-micropython
# brickrun -r -- pybricks-micropython

# pylint: skip-file

import constants as const
from pybricks.parameters import Color, Port, Stop
from pybricks.tools import wait
from robot import Robot
from utils import PIDValues, ev3_print, get_hostname, wait_button_pressed


def appa_main(appa: Robot):
    # Start
    appa.stop_mail_box.send(0)
    appa.infra_side_box.wait()
    appa.pid_line_follower(
        vel=100,
        pid=PIDValues(
            target=35,
            kp=1,
            ki=0.05,
            kd=10,
        ),
        loop_condition=lambda: (
            appa.color_fl.rgb()[2] > 50 and (appa.infra_side_box.read() or 100) > 3
        ),
    )
    appa.stop_mail_box.send(1)
    # Parada 1

    appa.simple_walk(speed=30, cm=-1.5)
    appa.pid_turn(90)
    appa.simple_walk(speed=30, cm=-3)
    appa.pid_align()
    appa.simple_walk(speed=30, cm=4)
    # Parada 2
    appa.stop_mail_box.send(0)
    # Parada 3
    appa.mbox.wait()
    appa.ev3_print(appa.mbox.read())

    # Parada 4
    appa.simple_walk(speed=30, cm=-10)

    appa.pid_turn(-90)
    appa.forward_while_same_reflection(
        speed_l=-50,
        speed_r=-50,
        reflection_diff=22,
        left_reflection_function=lambda: appa.color_bl.rgb()[2],
        right_reflection_function=lambda: appa.color_br.rgb()[2],
    )


def momo_main(momo: Robot):
    momo.motor_claw.run_until_stalled(500)
    # Start
    momo.infra_side_box.send(momo.infra_side.distance())
    while momo.stop_mail_box.read() == 0:
        momo.infra_side_box.send(momo.infra_side.distance())
        wait(10)
    # Parada 1
    # Parada 2
    momo.stop_mail_box.wait_new()
    momo.motor_claw.run_until_stalled(-500, then=Stop.HOLD)
    # Parada 3

    distance = momo.ultra_front.distance()
    momo.ev3_print("DIST:", distance)
    if distance > 60:
        age = "CHILD"
    else:
        age = "ADULT"
    momo.mbox.send(str(age) + " " + str(momo.color_front.rgb()))
    # Parada 4


def test_appa_main(appa: Robot):
    appa.pid_line_follower(
        vel=100,
        pid=PIDValues(
            target=35,
            kp=1,
            ki=0.05,
            kd=10,
        ),
        loop_condition=lambda: (appa.color_fl.rgb()[2] > 50),
    )


def test_momo_main(momo: Robot):
    ...


def main():
    if get_hostname() == "appa":
        appa_main(
            Robot(
                motor_l=Port.B,
                motor_r=Port.C,
                color_fl=Port.S1,
                color_fr=Port.S2,
                color_bl=Port.S3,
                color_br=Port.S4,
                color_max_value=65,
                turn_correction=1,
                debug=True,
                is_server=True,
            )
        )
    else:
        momo_main(
            Robot(
                infra_back=Port.S1,
                infra_side=Port.S2,
                color_front=Port.S3,
                ultra_front=Port.S4,
                motor_claw=Port.A,
                color_max_value=65,
                debug=True,
            )
        )


if __name__ == "__main__":
    main()
