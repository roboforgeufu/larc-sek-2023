#!/usr/bin/env pybricks-micropython
#brickrun -r -- pybricks-micropython


from pybricks.parameters import Color, Port
from pybricks.tools import wait


from robot import Robot
import constants as const
from utils import ev3_print, PIDValues, wait_button_pressed

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
                turn_correction=0.97,
                debug=True
            )

def chess_tower():
    i = 0
    elapsed_time = 0
    i_share = 0
    error = 0
    while True:
        has_seen_obstacle = toph.forward_while_same_reflection(
            reflection_diff=22,
            avoid_obstacles=True,
            left_reflection_function=lambda: toph.color_l.rgb()[2], 
            right_reflection_function=lambda: toph.color_r.rgb()[2],
        )
        
        toph.off_motors()
        toph.ev3_print("Saiu do loop:", has_seen_obstacle)
        wait_button_pressed(toph.brick)

        if has_seen_obstacle:
            # Aproxima do obstáculo e lê a cor do chão
            while(toph.ultra_front.distance() > 70):
                toph.ev3_print(toph.ultra_front.distance())
                toph.motor_r.run(50)
                toph.motor_l.run(50)
            toph.off_motors()
            toph.simple_walk(cm=4.0, speed=30)
            last_color = toph.color_l.color() 
            toph.ev3_print(last_color)
            toph.simple_walk(cm=-21.5, speed=30)
            wait_button_pressed(toph.brick)

        else:
            toph.off_motors()
            #wait_button_pressed(toph.brick)
            toph.simple_walk(cm=-5, speed=30)
            #wait_button_pressed(toph.brick)
            toph.pid_align(
                # PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3)
            )
            #wait_button_pressed(toph.brick)
            toph.simple_walk(cm=2, speed=30)
            
            last_color = toph.color_l.color() 
            ev3_print(last_color)
            #wait_button_pressed(toph.brick)

            toph.simple_walk(cm=-5, speed=30)
        

        toph.ev3_print(last_color)
        #wait_button_pressed(toph.brick)


        if(last_color == Color.BLACK or last_color == Color.YELLOW):
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

#while True:
    #toph.ev3_print(toph.color_l.rgb())

main()
