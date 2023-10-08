#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor, InfraredSensor, Motor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port, Stop
from pybricks.tools import DataLog, wait
from sensor_decision_trees import (
    momo_s3_decision_tree,
    s1_decision_tree,
    s2_decision_tree,
    s3_decision_tree,
    s4_decision_tree,
)
from utils import ev3_print, wait_button_pressed

ALL_COLORS = [
    # Color.BLUE,
    # Color.RED,
    # Color.GREEN,
    Color.BROWN,
    # Color.BLACK,
    # Color.YELLOW,
    # Color.WHITE,
]


def calibrate_all_sensors():
    brick = EV3Brick()
    for port in [
        Port.S3,
    ]:
        ev3_print("Sensor:", str(port), ev3=brick)
        sensor = ColorSensor(port)
        for color in ALL_COLORS:
            logger = DataLog(
                name="calibration_sensor_" + str(port) + "_" + str(color),
                timestamp=True,
                extension="csv",
            )
            ev3_print("PROX:", color, ev3=brick)
            wait_button_pressed(ev3=brick)
            ev3_print("LENDO...", ev3=brick)

            for _ in range(100):
                logger.log(sensor.rgb())
                wait(100)

            ev3_print("DONE:", color, ev3=brick)
            wait_button_pressed(ev3=brick)
            wait(500)


def test_calibration():
    brick = EV3Brick()
    # sensor1 = ColorSensor(Port.S1)
    # sensor2 = ColorSensor(Port.S2)
    # sensor3 = ColorSensor(Port.S3)
    # sensor4 = ColorSensor(Port.S4)

    momo_sensor_3 = ColorSensor(Port.S3)
    while True:
        # ev3_print("S1:", s1_decision_tree(sensor1.rgb()), ev3=brick)
        # ev3_print("S2:", s2_decision_tree(sensor2.rgb()), ev3=brick)
        ev3_print("S3:", momo_s3_decision_tree(momo_sensor_3.rgb()), ev3=brick)
        # ev3_print("S4:", s4_decision_tree(sensor4.rgb()), ev3=brick)
        wait(100)
        brick.screen.clear()


if __name__ == "__main__":
    test_calibration()
