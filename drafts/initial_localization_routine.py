#!/usr/bin/env pybricks-micropython

from pybricks.parameters import Color, Port
from pybricks.tools import wait

from robot import Robot
import constants as const
from utils import ev3_print

toph = Robot(
                wheel_diameter=const.WHEEL_DIAMETER,
                wheel_distance=const.WHEEL_DIST,
                motor_r=Port.C,
                motor_l=Port.B,
                motor_claw=Port.A,
                color_l=Port.S1,
                color_r=Port.S2,
                ultra_front=Port.S4,
                color_max_value=65,
                debug=True
            )

def chess_tower():
    i = 0
    elapsed_time = 0
    i_share = 0
    error = 0
    while True:
        while(toph.ultra_front.distance() > 200 and toph.color_l.color() == Color.WHITE):
            toph.motor_r.run(150)
            toph.motor_l.run(150)
            toph.ev3_print(toph.ultra_front.distance())
            toph.ev3_print(toph.color_l.color())
        toph.off_motors()
        last_color = toph.color_l.color()

        if(last_color == Color.WHITE):
            while(toph.ultra_front.distance() > 50):
                toph.motor_r.run(50)
                toph.motor_l.run(50)
            toph.off_motors()
            toph.simple_walk(cm=-21.5, speed=30)

        elif(last_color == Color.BLACK or last_color == Color.YELLOW):
            toph.simple_walk(cm=-15, speed=30)

        elif(last_color == Color.BLUE or last_color == Color.RED):
            break
        
        i += 1
        print(i)
        if(i % 2 == 0):
            toph.pid_turn(90)
        else:
            toph.pid_turn(180)


def main():
    chess_tower()
    

main()