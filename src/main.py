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
from domain.boarding import passenger_boarding, momo_passenger_boarding
from domain.chess_tower import chess_tower
from domain.delivery import deliver_person_ahead
from domain.map import decide_passenger_goal, path_to_movement
from robot import Robot
from utils import PIDValues, ev3_print, get_hostname, wait_button_pressed


def appa_main(appa: Robot):
    #
    # Localizacao inicial
    #

    chess_tower(appa)

    park_flag = 0
    while True:
        #
        # Coleta de pessoas
        #
        passenger_info = passenger_boarding(appa)

        #
        # Pathfinding e movimentacao
        #

        goal, park_flag = decide_passenger_goal(passenger_info, park_flag)

        appa.ev3_print(passenger_info)
        appa.ev3_print(goal)

        path_to_movement(appa, goal)

        #
        # Desembarque pessoas
        #

        deliver_person_ahead(appa)

        #
        # Retorno a origem
        #

        path_to_movement(appa, const.ORIGIN_TUPLE, start=goal)
        appa.stop_mail_box.send(1)


def momo_main(momo: Robot):
    momo.motor_claw.run_until_stalled(500, duty_limit=50)

    # 
    # Chess Tower
    # 
    # while True:
    #     momo.stop_mail_box.wait_new()
    #     if momo.stop_mail_box.read() == 0:
    #         break
    
    # momo.obstacle_box.send(momo.ultra_front.distance())
    # while momo.stop_mail_box.read() == 0:
    #     distance = momo.ultra_front.distance()
    #     momo.obstacle_box.send(distance)
    #     momo.ev3_print("obs:", distance)
    #     wait(10)

    #
    # Coleta de pessoas
    #

    while True:
        momo_passenger_boarding(momo)

        #
        # Retorno para a origem
        #

        #
        # Pathfinding e movimentacao
        #

        #
        # Desembarque pessoas
        #
        momo.stop_mail_box.wait()
        momo.motor_claw.run_until_stalled(500, duty_limit=50)

        #
        # Retorno a origem
        #


def test_appa_main(appa: Robot):
    while True:
        # appa.pid_turn(90)
        # wait_button_pressed(appa.brick)
        # appa.pid_turn(-90)
        appa.pid_align()
        appa.brick.speaker.beep()
        wait_button_pressed(appa.brick)
        appa.pid_align(direction_sign=-1, sensor_function_l=appa.color_bl.rgb()[2], sensor_function_r=appa.color_br.rgb()[2])
        appa.brick.speaker.beep()
        wait_button_pressed(appa.brick)

def test_momo_main(momo: Robot):
    pass


def teste_calibra_curvas(appa: Robot):
    pid = PIDValues(
            kp=0.8,
            ki=0.01,
            kd=0.4,
        )
    while True:
        for _ in range(4):
            appa.pid_turn(90, pid=pid)
            appa.brick.speaker.beep()
            wait(500)
        wait_button_pressed(appa.brick)
        for _ in range(4):
            appa.pid_turn(-90, pid=pid)
            appa.brick.speaker.beep()
            wait(500)
        wait_button_pressed(appa.brick)
        for _ in range(2):
            appa.pid_turn(180, pid=pid)
            appa.brick.speaker.beep()
            wait(500)
        wait_button_pressed(appa.brick)
        for _ in range(2):
            appa.pid_turn(-180, pid=pid)
            appa.brick.speaker.beep()
            wait(500)



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
                turn_correction=0.97,
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
