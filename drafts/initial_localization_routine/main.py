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
    motor_l=Port.A,
    motor_claw=Port.A,
    color_l=Port.S1,
    color_r=Port.S2,
    ultra_front=Port.S4,
    color_max_value=65,
    turn_correction=1.02,
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
            avoid_obstacles=False,
            left_reflection_function=lambda: toph.color_l.rgb()[2],
            right_reflection_function=lambda: toph.color_r.rgb()[2],
        )
        toph.ev3_print("Saiu do loop:", has_seen_obstacle)

        if has_seen_obstacle:
            # Aproxima do obstáculo e lê a cor do chão
            while toph.ultra_front.distance() > 70:
                toph.ev3_print(toph.ultra_front.distance())
                elapsed_time, i_share, error = loopless_pid_walk(
                    elapsed_time,
                    i_share,
                    error,
                )
            toph.pid_walk(cm=4.0, speed=30)
            last_color = toph.color_l.color()
            toph.ev3_print(last_color)
            toph.pid_walk(cm=25.5, speed=-80)
          
        else:
            toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
            sensor_function_r=lambda: toph.color_r.rgb()[2])
            toph.pid_walk(cm=1, speed=30) 
            last_color = toph.color_l.color()

            toph.ev3_print(last_color)
            wait_button_pressed(toph.brick)


        if last_color == Color.BLACK or last_color == Color.YELLOW:
            toph.pid_walk(cm=10, speed=-80)

        elif last_color == Color.BLUE or last_color == Color.RED:
            if(last_color == Color.BLUE):
                break
            else:
                three_colors_l=[]
                three_colors_r=[]
                three_colors_l.append(last_color)
                three_colors_r.append(last_color)
                #Cria uma lista armazenando as 3 cores do caso
                toph.pid_walk(cm=7,speed=-80)
                toph.pid_turn(-90)
                toph.forward_while_same_reflection(reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: toph.color_l.rgb()[2],
                    right_reflection_function=lambda: toph.color_r.rgb()[2],)
                toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                sensor_function_r=lambda: toph.color_r.rgb()[2])
                toph.pid_walk(cm=1,speed=20)

                toph.ev3_print(toph.color_l.rgb())
                wait_button_pressed(toph.brick)

                #Se movimenta para checar a cor ao lado, alinha nessa cor e logo em seguida adiciona essa cor a lista.
                three_colors_l.append(toph.color_l.rgb()[2])
                three_colors_r.append(toph.color_r.rgb()[2])

                # print(toph.color_l.rgb())
                # toph.ev3_print(toph.color_l.rgb())
                # wait_button_pressed(toph.brick)

                toph.pid_walk(cm=8,speed=-80)
                toph.pid_turn(180)
                toph.forward_while_same_reflection(reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: toph.color_l.rgb()[2],
                    right_reflection_function=lambda: toph.color_r.rgb()[2],)
                toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                sensor_function_r=lambda: toph.color_r.rgb()[2])
                toph.pid_walk(cm=1,speed=20)

                toph.ev3_print(toph.color_l.rgb())
                wait_button_pressed(toph.brick)

                #Volta um pouco para tras, gira 180 e checa a outra cor na frente, e em seguida adiciona na lista.
                #Apenas para a Toph que não possui sensores atras
                three_colors_l.append(toph.color_l.rgb()[2])
                three_colors_r.append(toph.color_r.rgb()[2])

                # print(toph.color_l.rgb())
                # toph.ev3_print(toph.color_l.rgb())
                # wait_button_pressed(toph.brick)

                toph.pid_walk(cm=8,speed=-80)
                toph.pid_turn(90)
                #Gira 90 graus para sair do caso e ir para o espaço em aberto e seguir 
                print(three_colors_l)
                print(three_colors_r)

                if((three_colors_l[1] >= 15 and three_colors_l[2] <= 30) or (three_colors_r[1] >= 15 and three_colors_r[2] <= 30)):
                    #Se é o case A onde vemos VERMELHO, AMARELO, AMARELO (posicao A) 
                    toph.pid_walk(cm=30,speed=80)
                    toph.pid_turn(-90)
                    toph.forward_while_same_reflection(reflection_diff=22,
                        avoid_obstacles=False,
                        left_reflection_function=lambda: toph.color_l.rgb()[2],
                        right_reflection_function=lambda: toph.color_r.rgb()[2],)
                    #Anda um pouco para frente, faz uma gira para esquerda, e anda até ver alguma cor diferente
                    toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                    sensor_function_r=lambda: toph.color_r.rgb()[2])
                    
                if((three_colors_l[1] > 2 and three_colors_l[1] < 9) and ((three_colors_r[2] >= 15 and three_colors_r[2] <= 30) or (three_colors_l[2]>= 15 and three_colors_l[2] <= 30))):
                    #Case B caso veja VERMELHO, PRETO, AMARELO.
                    toph.pid_walk(cm=30,speed=80)
                    toph.pid_turn(90)
                    # posicao H e C chegam no azul
                    toph.forward_while_same_reflection(reflection_diff=22,
                        avoid_obstacles=False,
                        left_reflection_function=lambda: toph.color_l.rgb()[2],
                        right_reflection_function=lambda: toph.color_r.rgb()[2],)
                    toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                    sensor_function_r=lambda: toph.color_r.rgb()[2])
                    # posicao F chega no preto
                    if ((toph.color_l.rgb()[2] > 2 and toph.color_l.rgb()[2] < 9) or (toph.color_r.rgb()[2] > 2 and toph.color_r.rgb()[2] < 9)):
                        toph.pid_walk(cm=10, speed=-80)
                        toph.pid_turn(180)
                        toph.forward_while_same_reflection(reflection_diff=22,
                        avoid_obstacles=False,
                        left_reflection_function=lambda: toph.color_l.rgb()[2],
                        right_reflection_function=lambda: toph.color_r.rgb()[2],)
                    toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                    sensor_function_r=lambda: toph.color_r.rgb()[2])

                # todos necessariamente no azul
                toph.pid_walk(cm=10, speed=-80)
                toph.pid_turn(90)
                toph.forward_while_same_reflection(reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: toph.color_l.rgb()[2],
                    right_reflection_function=lambda: toph.color_r.rgb()[2],)
                toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                sensor_function_r=lambda: toph.color_r.rgb()[2])
                # chega na origem
                toph.pid_walk(cm=10, speed=-80)
                toph.pid_turn(-90)
                toph.forward_while_same_reflection(reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: toph.color_l.rgb()[2],
                    right_reflection_function=lambda: toph.color_r.rgb()[2],)
                toph.pid_align(PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2), sensor_function_l=lambda: toph.color_l.rgb()[2],
                sensor_function_r=lambda: toph.color_r.rgb()[2])
                toph.pid_walk(cm=5, speed=-50)
                toph.pid_turn(-90)
                #normaliza a posicao na origem
                break

        i += 1
        # print(i)
        if i % 2 == 0:
            toph.pid_turn(90)
        else:
            toph.pid_turn(180)

def main():
    chess_tower()
     #while True:
    
        #print(toph.color_l.rgb())
        #print(toph.color_r.rgb())

def calibra_pid_align():
    while True:
        toph.pid_align(
            PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
            sensor_function_l=lambda: toph.color_l.rgb()[2],
            sensor_function_r=lambda: toph.color_r.rgb()[2],
        )
        wait_button_pressed(toph.brick)

#calibra_pid_align()  
#def corner_A():
main()
