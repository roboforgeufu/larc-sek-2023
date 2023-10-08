#!/usr/bin/env pybricks-micropython

"""
Módulo para centralização dos processos e gerência da estratégia geral.

Podem estar nesse módulo coisas como:
    - Código que controla a ordem que as rotinas serão executadas
    - Código para controle de fluxo geral do robô
    - Chamadas a rotinas mais específicas
    - Instanciação de estruturas de dados, classes, etc.
    - Códigos específicos de comunicação com o EV3 ou gerência de recursos de sistemas
        operacionais no geral
    - Várias funções "main" alternativas, inclusive para testes ou calibragem de
        motores/sensores

Não devem estar nesse módulo:
    - Definição de constantes ou variáveis globais
    - Chamadas de função fora de escopo da main do módulo
    - Execução de código manipulando motores ou sensores diretamente
"""
# brickrun -r -- pybricks-micropython

# pylint: skip-file

from pybricks.parameters import Color, Port, Stop
from pybricks.tools import wait

import constants as const
from domain.chess_tower import chess_tower
from domain.map import path_to_movement
from robot import Robot
from utils import PIDValues, ev3_print, get_hostname, wait_button_pressed


def appa_main(appa: Robot):
    #
    # Localizacao inicial
    #

    chess_tower(appa)
    # return True

    # Coleta de pessoas
    #

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

    # recebe info sobre tamanho e cor do passageiro
    passenger_info = appa.mbox.read()
    appa.ev3_print(passenger_info)

    # Parada 4
    appa.simple_walk(speed=30, cm=-10)

    # appa.pid_turn(-90)
    # appa.forward_while_same_reflection(
    #     speed_l=-50,
    #     speed_r=-50,
    #     reflection_diff=22,
    #     left_reflection_function=lambda: appa.color_bl.rgb()[2],
    #     right_reflection_function=lambda: appa.color_br.rgb()[2],
    # )

    #
    # Retorno para a origem
    #

    # alinha no azul
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.pid_align(
        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: appa.color_fl.rgb()[2],
        sensor_function_r=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)
    appa.pid_turn(90)

    # vai até a origem
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)
    appa.pid_turn(90)

    # restaura a posicao inicial
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.pid_align(
        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: appa.color_fl.rgb()[2],
        sensor_function_r=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)
    appa.pid_turn(-90)
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.pid_align(
        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: appa.color_fl.rgb()[2],
        sensor_function_r=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)

    #
    # Pathfinding e movimentacao
    #

    park_flag = 0
    passenger_info = passenger_info.split()
    if passenger_info[0] == "CHILD":
        if passenger_info[1] == "Color.BLUE":
            goal = (0, 8)  # escola
        elif passenger_info[1] == "Color.BROWN":
            goal = (8, 8)  # biblioteca
        elif passenger_info[1] == "Color.GREEN":
            if park_flag == 0:  # parque
                goal = (8, 0)
            elif park_flag == 1:
                goal = (4, 0)
            elif park_flag == 2:
                goal = (0, 0)
            park_flag += 1

    if passenger_info[0] == "ADULT":
        if passenger_info[1] == "Color.BLUE":
            goal = (8, 4)  # museu
        elif passenger_info[1] == "Color.BROWN":
            goal = (0, 4)  # padaria
        elif passenger_info[1] == "Color.GREEN":
            goal = (4, 8)  # prefeitura
        elif passenger_info[1] == "Color.RED":
            goal = (4, 4)  # farmacia

    appa.ev3_print(passenger_info)
    appa.ev3_print(goal)
    # wait_button_pressed(appa.brick)

    path_to_movement(appa, goal)

    #
    # Desembarque pessoas
    #

    #
    # Retorno a origem
    #

    path_to_movement(appa, (8, 10), start=goal)


def momo_main(momo: Robot):
    momo.motor_claw.run_until_stalled(500)

    #
    # Coleta de pessoas
    #

    while True:
        momo.stop_mail_box.wait_new()
        if momo.stop_mail_box.read() == 0:
            break

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
    momo.mbox.send(str(age) + " " + str(momo.color_front.color()))

    # Parada 4

    #
    # Retorno para a origem
    #

    #
    # Pathfinding e movimentacao
    #

    #
    # Desembarque pessoas
    #

    #
    # Retorno a origem
    #


def test_appa_main(appa: Robot):
    # appa.pid_line_follower(
    #     vel=100,
    #     pid=PIDValues(
    #         target=35,
    #         kp=1,
    #         ki=0.05,
    #         kd=10,
    #     ),
    #     loop_condition=lambda: (appa.color_fl.rgb()[2] > 50),
    # chess_tower(appa)
    # path_to_movement(appa,(8,10),start=(6,2))

    # alinha no azul

    #
    # Retorno para a origem
    #

    # alinha no azul
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-2)
    appa.pid_align(
        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: appa.color_fl.rgb()[2],
        sensor_function_r=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)
    appa.pid_turn(90)

    # vai até a origem
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)
    appa.pid_turn(90)

    # restaura a posicao inicial
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.pid_align(
        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: appa.color_fl.rgb()[2],
        sensor_function_r=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)
    appa.pid_turn(-90)
    appa.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: appa.color_fl.rgb()[2],
        right_reflection_function=lambda: appa.color_fr.rgb()[2],
    )
    appa.pid_align(
        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: appa.color_fl.rgb()[2],
        sensor_function_r=lambda: appa.color_fr.rgb()[2],
    )
    appa.simple_walk(speed=30, cm=-10)

    passenger_info = "CHILD Color.BLUE"
    passenger_info = passenger_info.split()
    if passenger_info[0] == "CHILD":
        if passenger_info[1] == "Color.BLUE":
            goal = (0, 8)  # escola
        elif passenger_info[1] == "Color.BROWN":
            goal = (8, 8)  # biblioteca
        elif passenger_info[1] == "Color.GREEN":
            if park_flag == 0:  # parque
                goal = (8, 0)
            elif park_flag == 1:
                goal = (4, 0)
            elif park_flag == 2:
                goal = (0, 0)
            park_flag += 1

    if passenger_info[0] == "ADULT":
        if passenger_info[1] == "Color.BLUE":
            goal = (8, 4)  # museu
        elif passenger_info[1] == "Color.BROWN":
            goal = (0, 4)  # padaria
        elif passenger_info[1] == "Color.GREEN":
            goal = (4, 8)  # prefeitura
        elif passenger_info[1] == "Color.RED":
            goal = (4, 4)  # farmacia

    path_to_movement(appa, goal)


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
                ultra_back=Port.S1,
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
