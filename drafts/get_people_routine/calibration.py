#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor, InfraredSensor, Motor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port, Stop
from pybricks.tools import DataLog, wait

from utils import ev3_print, wait_button_pressed

ALL_COLORS = [
    Color.BLUE,
    Color.RED,
    Color.GREEN,
    Color.BROWN,
    Color.BLACK,
    Color.YELLOW,
    Color.WHITE,
]


def calibrate_all_sensors():
    brick = EV3Brick()
    for port in [Port.S1, Port.S2, Port.S3, Port.S4]:
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

            for _ in range(200):
                logger.log(sensor.rgb())
                wait(100)

            ev3_print("DONE:", color, ev3=brick)
            wait_button_pressed(ev3=brick)
            wait(500)


if __name__ == "__main__":
    calibrate_all_sensors()
