#!/usr/bin/env pybricks-micropython
# brickrun -r -- pybricks-micropython


from pybricks.parameters import Color, Port
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import PIDValues, ev3_print, wait_button_pressed

toph = Robot(
    wheel_diameter=const.WHEEL_DIAMETER,
    wheel_distance=const.WHEEL_DIST,
    motor_r=Port.C,
    motor_l=Port.B,
    #motor_claw=Port.A,
    color_l=Port.S1,
    color_r=Port.S2,
    ultra_front=Port.S4,
    color_max_value=65,
    turn_correction=0.97,
    debug=True,
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

        if has_seen_obstacle:
            # Aproxima do obstáculo e lê a cor do chão
            while toph.ultra_front.distance() > 70:
                toph.ev3_print(toph.ultra_front.distance())
                toph.motor_r.run(50)
                toph.motor_l.run(50)
            toph.off_motors()
            toph.simple_walk(cm=4.0, speed=30)
            last_color = toph.color_l.color()
            toph.ev3_print(last_color)
            toph.simple_walk(cm=-21.5, speed=30)
          

        else:
            toph.off_motors()
            toph.simple_walk(cm=-5, speed=30)
            toph.pid_align(PIDValues(target=50, kp=0.8, ki=0.003, kd=0.1), sensor_function_l=lambda: toph.color_l.rgb()[2],
            sensor_function_r=lambda: toph.color_r.rgb()[2])
            toph.simple_walk(cm=2, speed=30)
            last_color = toph.color_l.color()
            ev3_print(last_color)
            toph.simple_walk(cm=-5, speed=30)

        toph.ev3_print(last_color)

        if last_color == Color.BLACK or last_color == Color.YELLOW:
            toph.simple_walk(cm=-15, speed=30)

        elif last_color == Color.BLUE or last_color == Color.RED:
            if(last_color == Color.BLUE):
                break
            else:
                three_colors=[]
                three_colors.append(last_color)
                #Cria uma lista armazenando as 3 cores do caso
                toph.simple_walk(cm=-6,speed=30)
                toph.pid_turn(-90)
                toph.simple_walk(cm=6,speed=30)
                toph.pid_align(PIDValues(target=50, kp=0.8, ki=0.003, kd=0.1), sensor_function_l=lambda: toph.color_l.rgb()[2],
                sensor_function_r=lambda: toph.color_r.rgb()[2])
                toph.simple_walk(cm=1,speed=20)
                #Se movimenta para checar a cor ao lado, alinha nessa cor e logo em seguida adiciona essa cor a lista.
                three_colors.append(toph.color_l.rgb()[2])
                print(toph.color_l.rgb())
                toph.simple_walk(cm=-6,speed=30)
                toph.pid_turn(180)
                toph.simple_walk(cm=6,speed=30)
                toph.pid_align(PIDValues(target=50, kp=0.8, ki=0.003, kd=0.1), sensor_function_l=lambda: toph.color_l.rgb()[2],
                sensor_function_r=lambda: toph.color_r.rgb()[2])
                toph.simple_walk(cm=1,speed=20)
                #Volta um pouco para tras, gira 180 e checa a outra cor na frente, e em seguida adiciona na lista.
                #Apenas para a Toph que não possui sensores atras
                three_colors.append(toph.color_l.rgb()[2])
                print(toph.color_l.rgb())
                toph.simple_walk(cm=-6,speed=30)
                toph.pid_turn(90)
                #Gira 90 graus para sair do caso e ir para o espaço em aberto e seguir 
                print(three_colors)
                if((three_colors[1] > 18 and three_colors[2] > 18) and (three_colors[1] < 25 and three_colors[2] < 25)):
                    toph.simple_walk(cm=25,speed=30)
                    toph.pid_turn(-90)
                    toph.forward_while_same_reflection(reflection_diff=22,
                        avoid_obstacles=True,
                        left_reflection_function=lambda: toph.color_l.rgb()[2],
                        right_reflection_function=lambda: toph.color_r.rgb()[2],)
                if(three_colors[1] == Color.BLACK and (three_colors[2]>18 and three_colors[2]<25)):
                    toph.simple_walk(cm=-6,speed=30)
                    toph.pid_turn(90)
                    toph.forward_while_same_reflection(reflection_diff=22,
                        avoid_obstacles=True,
                        left_reflection_function=lambda: toph.color_l.rgb()[2],
                        right_reflection_function=lambda: toph.color_r.rgb()[2],)
                    if((three_colors[1] > 18 and three_colors[2] > 18) and (three_colors[1] < 25 and three_colors[2] < 25)):
                        toph.simple_walk(cm=25,speed=30)
                        toph.pid_turn(-90)
                        toph.forward_while_same_reflection(reflection_diff=22,
                            avoid_obstacles=True,
                            left_reflection_function=lambda: toph.color_l.rgb()[2],
                            right_reflection_function=lambda: toph.color_r.rgb()[2],)
                    
                break

                

        i += 1
        print(i)
        if i % 2 == 0:
            toph.pid_turn(90)
        else:
            toph.pid_turn(180)


def main():
    chess_tower()
    #while True:
       #print(toph.color_l.rgb())


main()    
#def corner_A():

def calibra_pid_align():
    while True:
        toph.pid_align(
            PIDValues(target=50, kp=0.8, ki=0.003, kd=0.1),
            sensor_function_l=lambda: toph.color_l.rgb()[2],
            sensor_function_r=lambda: toph.color_r.rgb()[2],
        )



