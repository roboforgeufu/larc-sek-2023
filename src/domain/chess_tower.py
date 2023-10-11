from pybricks.parameters import Color
from robot import Robot
from utils import PIDValues, wait_button_pressed
import constants as const


def chess_tower(robot: Robot):
    i = 0
    # robot.stop_mail_box.send(0)
    while True:
        has_seen_obstacle = robot.forward_while_same_reflection(
            reflection_diff=22,
            # obstacle_function=lambda: robot.obstacle_box.read() < const.OBSTACLE_DIST,
            left_reflection_function=lambda: robot.color_fl.rgb()[2],
            right_reflection_function=lambda: robot.color_fr.rgb()[2],
        )
        robot.ev3_print("Saiu do loop:", has_seen_obstacle)

        if has_seen_obstacle:
            get_closer_to_obstacle_routine(robot)
        else:
            robot.pid_walk(cm=2, speed=-30)
            robot.pid_align()
            robot.pid_walk(cm=1, speed=30)
            last_color_l, last_color_r = robot.color_fl.color(), robot.color_fr.color()

            robot.brick.speaker.beep()
            robot.ev3_print(last_color_l, last_color_r)

            if last_color_l != last_color_r:
                last_color = (
                    last_color_l if last_color_l != Color.BLACK else last_color_r
                )
            else:
                last_color = last_color_l

            robot.ev3_print(last_color)

        if last_color == Color.BLACK or last_color == Color.YELLOW:
            robot.pid_walk(cm=10, speed=-50)

        elif last_color == Color.BLUE or last_color == Color.RED:
            if last_color == Color.BLUE:
                # Azul de frente
                robot.pid_walk(cm=10, speed=-80)
                robot.pid_turn(90)

            else:
                three_colors_l, three_colors_r = check_three_colors(robot, last_color)

                if len(three_colors_l) == 0:
                    # Azul de frente
                    robot.pid_walk(cm=10, speed=-80)
                    robot.pid_turn(90)
                elif (
                    three_colors_l[1] == Color.YELLOW
                    and three_colors_l[2] == Color.YELLOW
                ) or (
                    three_colors_r[1] == Color.YELLOW
                    and three_colors_r[2] == Color.YELLOW
                ):
                    # Se é o case A onde vemos VERMELHO, AMARELO, AMARELO (posicao A)
                    case_a_routine(robot)
                elif (three_colors_l[1] == Color.BLACK) and (
                    (three_colors_r[2] == Color.YELLOW)
                    or (three_colors_l[2] == Color.YELLOW)
                ):
                    # Case B caso veja VERMELHO, PRETO, AMARELO.
                    triple_case_routine(robot)

            # normaliza a posicao na origem
            go_to_origin_routine(robot)
            break

        i += 1
        # print(i)
        if i % 2 == 0:
            robot.pid_turn(90)
        else:
            robot.pid_turn(
                156,
                pid=PIDValues(
                    kp=3.5,
                    ki=0.01,
                    kd=10
                ),
                )
    
    # robot.stop_mail_box.send(1)


def calibra_pid_align(robot: Robot):
    while True:
        robot.pid_align()
        wait_button_pressed(robot.brick)


def case_a_routine(robot: Robot):
    """VERMELHO AMARELO AMARELO"""
    robot.pid_walk(cm=30, speed=80)
    robot.pid_turn(-90)

    robot.forward_while_same_reflection(
        speed_l=-50,
        speed_r=-50,
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_bl.rgb()[2],
        right_reflection_function=lambda: robot.color_br.rgb()[2],
        fix_errors=False
    )
    robot.pid_walk(cm=2, speed=50)
    robot.pid_align(
        sensor_function_l=lambda: robot.color_bl.rgb()[2],
        sensor_function_r=lambda: robot.color_br.rgb()[2],
        direction_sign=-1
    )
    
    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
    )
    robot.pid_align()
    # Azul de frente
    robot.pid_walk(cm=10, speed=-80)
    robot.pid_turn(90)
    # Para apontado pra origem


def triple_case_routine(robot: Robot):
    """VERMELHO PRETO AMARELO"""
    robot.pid_walk(cm=30, speed=80)
    robot.pid_turn(90)
    # posicao H e C chegam no azul
    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
    )
    robot.pid_align()
    robot.pid_walk(cm=1, speed=30)
    # posicao F chega no preto
    if (robot.color_fl.color() == Color.BLACK) or (
        robot.color_fr.color() == Color.BLACK
    ):
        robot.forward_while_same_reflection(
            speed_l=-30,
            speed_r=-30,
            reflection_diff=22,
            left_reflection_function=lambda: robot.color_bl.rgb()[2],
            right_reflection_function=lambda: robot.color_br.rgb()[2],
        )
        robot.pid_align(
            sensor_function_l=lambda: robot.color_bl.rgb()[2],
            sensor_function_r=lambda: robot.color_br.rgb()[2],
            direction_sign=-1,
        )
        # Azul de costas
        robot.pid_walk(cm=10, speed=80)
        robot.pid_turn(-90)

    else:
        robot.pid_align()
        # Azul de frente
        robot.pid_walk(cm=10, speed=-80)
        robot.pid_turn(90)


def check_three_colors(robot: Robot, last_color):
    three_colors_l = []
    three_colors_r = []
    three_colors_l.append(last_color)
    three_colors_r.append(last_color)
    # Cria uma lista armazenando as 3 cores do caso
    robot.pid_walk(cm=7, speed=-80)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
    )
    robot.pid_walk(cm=2, speed=-30)
    robot.pid_align()
    robot.pid_walk(cm=1, speed=20)

    robot.ev3_print(robot.color_fl.color())

    # Se movimenta para checar a cor ao lado, alinha nessa cor e logo em seguida adiciona essa cor a lista.
    if Color.BLUE in (robot.color_fl.color(), robot.color_fr.color()):
        return [], []
    three_colors_l.append(robot.color_fl.color())
    three_colors_r.append(robot.color_fr.color())

    robot.forward_while_same_reflection(
        speed_l=-30,
        speed_r=-30,
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_bl.rgb()[2],
        right_reflection_function=lambda: robot.color_br.rgb()[2],
    )
    robot.pid_walk(cm=2, speed=30)
    robot.pid_align(
        PIDValues(target=65, kp=0.6, ki=0.005, kd=0.2),
        sensor_function_l=lambda: robot.color_bl.rgb()[2],
        sensor_function_r=lambda: robot.color_br.rgb()[2],
        direction_sign=-1,
    )
    robot.pid_walk(cm=1, speed=-20)

    robot.ev3_print(robot.color_bl.color(), robot.color_br.color())

    # Volta um pouco para tras, gira 180 e checa a outra cor na frente, e em seguida adiciona na lista.
    # Apenas para a Toph que não possui sensores atras
    if Color.BLUE in (robot.color_fl.color(), robot.color_fr.color()):
        return [], []

    three_colors_l.append(robot.color_bl.color())
    three_colors_r.append(robot.color_br.color())
    robot.ev3_print(three_colors_l)
    robot.ev3_print(three_colors_r)

    robot.pid_walk(cm=8, speed=80)
    robot.pid_turn(-90)
    # Gira 90 graus para sair do caso e ir para o espaço em aberto e seguir
    return three_colors_l, three_colors_r


def get_closer_to_obstacle_routine(robot: Robot):
    # Aproxima do obstáculo e lê a cor do chão
    elapsed_time, i_share, error = 0,0,0
    while robot.ultra_front.distance() > 70:
        robot.ev3_print(robot.ultra_front.distance())
        elapsed_time, i_share, error = robot.loopless_pid_walk(
            elapsed_time,
            i_share,
            error,
        )
    robot.pid_walk(cm=4.0, speed=30)
    last_color = robot.color_fl.color()
    robot.ev3_print(last_color)
    robot.pid_walk(cm=21, speed=-60, fix_errors=True)


def go_to_origin_routine(robot: Robot):
    # Apontando pro vermelho
    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
    )
    robot.pid_walk(cm=2, speed=-30)
    robot.pid_align()
    # chega na origem
    robot.pid_walk(cm=3, speed=-30)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
        fix_errors=True
    )
    robot.pid_walk(cm=2, speed=-30)
    robot.pid_align()
    robot.pid_walk(cm=2, speed=-30)
    robot.pid_turn(-90)
    robot.pid_align(
        sensor_function_l=lambda: robot.color_bl.rgb()[2],
        sensor_function_r=lambda: robot.color_br.rgb()[2],
        direction_sign=-1,
    )
    #robot.pid_walk(cm=1, speed=30)
