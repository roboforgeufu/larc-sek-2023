from pybricks.parameters import Color
from pybricks.tools import wait

from robot import Robot
from utils import PIDValues


def deliver_person_ahead(robot: Robot):
    first_seen = None
    while True:
        robot.forward_while_same_reflection(
            reflection_diff=22,
            left_reflection_function=lambda: robot.color_fl.rgb()[2],
            right_reflection_function=lambda: robot.color_fr.rgb()[2],
            fix_errors=False,
        )
        robot.pid_walk(cm=2, speed=-30)
        robot.pid_align()
        robot.pid_walk(cm=2, speed=30)

        right_color = robot.color_fr.color()
        left_color = robot.color_fl.color()

        robot.ev3_print(right_color, left_color)

        if right_color == Color.YELLOW and left_color == Color.YELLOW:
            break

        seen_on = robot.color_fl if right_color == Color.YELLOW else robot.color_fr
        if first_seen is not None:
            if seen_on != first_seen:
                break
        else:
            first_seen = seen_on

        direction_sign = 1 if right_color == Color.YELLOW else -1

        if first_seen is None:
            first_seen = (
                robot.color_fl if right_color == Color.YELLOW else robot.color_fr
            )

        robot.pid_walk(cm=15, speed=-30)
        robot.simple_turn(10 * direction_sign)

    robot.pid_walk(cm=15, speed=-30)
    robot.brick.speaker.beep(500)
    robot.pid_walk(cm=20, speed=30)
    robot.stop_mail_box.send(0)
    wait(2000)
    robot.pid_walk(cm=10, speed=-50)

    robot.forward_while_same_reflection(
        speed_l=-50,
        speed_r=-50,
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_bl.rgb()[2],
        right_reflection_function=lambda: robot.color_br.rgb()[2],
        fix_errors=False,
    )
    robot.pid_walk(speed=30, cm=1)
    robot.pid_align(
        sensor_function_l=lambda: robot.color_bl.rgb()[2],
        sensor_function_r=lambda: robot.color_br.rgb()[2],
        direction_sign=-1
    )

