from pybricks.parameters import Color

from robot import Robot
from utils import PIDValues, wait_button_pressed


def chess_tower(robot: Robot):
    i = 0
    elapsed_time = 0
    i_share = 0
    error = 0
    while True:
        has_seen_obstacle = robot.forward_while_same_reflection(
            reflection_diff=22,
            avoid_obstacles=False,
            left_reflection_function=lambda: robot.color_fl.rgb()[2],
            right_reflection_function=lambda: robot.color_fr.rgb()[2],
        )
        robot.ev3_print("Saiu do loop:", has_seen_obstacle)

        if has_seen_obstacle:
            # Aproxima do obstáculo e lê a cor do chão
            while robot.ultra_front.distance() > 70:
                robot.ev3_print(robot.ultra_front.distance())
                # elapsed_time, i_share, error = loopless_pid_walk(
                #     elapsed_time,
                #     i_share,
                #     error,
                # )
            robot.pid_walk(cm=4.0, speed=30)
            last_color = robot.color_fl.color()
            robot.ev3_print(last_color)
            robot.pid_walk(cm=25.5, speed=-80)

        else:
            robot.pid_align(
                PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                sensor_function_l=lambda: robot.color_fl.rgb()[2],
                sensor_function_r=lambda: robot.color_fr.rgb()[2],
            )
            robot.pid_walk(cm=1, speed=30)
            last_color = robot.color_fl.color()

            robot.ev3_print(last_color)
            wait_button_pressed(robot.brick)

        if last_color == Color.BLACK or last_color == Color.YELLOW:
            robot.pid_walk(cm=10, speed=-80)

        elif last_color == Color.BLUE or last_color == Color.RED:
            if last_color == Color.BLUE:
                break
            else:
                three_colors_l = []
                three_colors_r = []
                three_colors_l.append(last_color)
                three_colors_r.append(last_color)
                # Cria uma lista armazenando as 3 cores do caso
                robot.pid_walk(cm=7, speed=-80)
                robot.pid_turn(-90)
                robot.forward_while_same_reflection(
                    reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: robot.color_fl.rgb()[2],
                    right_reflection_function=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_align(
                    PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                    sensor_function_l=lambda: robot.color_fl.rgb()[2],
                    sensor_function_r=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_walk(cm=1, speed=20)

                robot.ev3_print(robot.color_fl.rgb())
                wait_button_pressed(robot.brick)

                # Se movimenta para checar a cor ao lado, alinha nessa cor e logo em seguida adiciona essa cor a lista.
                three_colors_l.append(robot.color_fl.rgb()[2])
                three_colors_r.append(robot.color_fr.rgb()[2])

                # print(robot.color_l.rgb())
                # robot.ev3_print(robot.color_l.rgb())
                # wait_button_pressed(robot.brick)

                robot.pid_walk(cm=8, speed=-80)
                robot.pid_turn(180)
                robot.forward_while_same_reflection(
                    reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: robot.color_fl.rgb()[2],
                    right_reflection_function=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_align(
                    PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                    sensor_function_l=lambda: robot.color_fl.rgb()[2],
                    sensor_function_r=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_walk(cm=1, speed=20)

                robot.ev3_print(robot.color_fl.rgb())
                wait_button_pressed(robot.brick)

                # Volta um pouco para tras, gira 180 e checa a outra cor na frente, e em seguida adiciona na lista.
                # Apenas para a Toph que não possui sensores atras
                three_colors_l.append(robot.color_fl.rgb()[2])
                three_colors_r.append(robot.color_fr.rgb()[2])

                # print(robot.color_l.rgb())
                # robot.ev3_print(robot.color_l.rgb())
                # wait_button_pressed(robot.brick)

                robot.pid_walk(cm=8, speed=-80)
                robot.pid_turn(90)
                # Gira 90 graus para sair do caso e ir para o espaço em aberto e seguir
                print(three_colors_l)
                print(three_colors_r)

                if (three_colors_l[1] >= 15 and three_colors_l[2] <= 30) or (
                    three_colors_r[1] >= 15 and three_colors_r[2] <= 30
                ):
                    # Se é o case A onde vemos VERMELHO, AMARELO, AMARELO (posicao A)
                    robot.pid_walk(cm=30, speed=80)
                    robot.pid_turn(-90)
                    robot.forward_while_same_reflection(
                        reflection_diff=22,
                        avoid_obstacles=False,
                        left_reflection_function=lambda: robot.color_fl.rgb()[2],
                        right_reflection_function=lambda: robot.color_fr.rgb()[2],
                    )
                    # Anda um pouco para frente, faz uma gira para esquerda, e anda até ver alguma cor diferente
                    robot.pid_align(
                        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                        sensor_function_l=lambda: robot.color_fl.rgb()[2],
                        sensor_function_r=lambda: robot.color_fr.rgb()[2],
                    )

                if (three_colors_l[1] > 2 and three_colors_l[1] < 9) and (
                    (three_colors_r[2] >= 15 and three_colors_r[2] <= 30)
                    or (three_colors_l[2] >= 15 and three_colors_l[2] <= 30)
                ):
                    # Case B caso veja VERMELHO, PRETO, AMARELO.
                    robot.pid_walk(cm=30, speed=80)
                    robot.pid_turn(90)
                    # posicao H e C chegam no azul
                    robot.forward_while_same_reflection(
                        reflection_diff=22,
                        avoid_obstacles=False,
                        left_reflection_function=lambda: robot.color_fl.rgb()[2],
                        right_reflection_function=lambda: robot.color_fr.rgb()[2],
                    )
                    robot.pid_align(
                        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                        sensor_function_l=lambda: robot.color_fl.rgb()[2],
                        sensor_function_r=lambda: robot.color_fr.rgb()[2],
                    )
                    # posicao F chega no preto
                    if (robot.color_fl.rgb()[2] > 2 and robot.color_fl.rgb()[2] < 9) or (
                        robot.color_fr.rgb()[2] > 2 and robot.color_fr.rgb()[2] < 9
                    ):
                        robot.pid_walk(cm=10, speed=-80)
                        robot.pid_turn(180)
                        robot.forward_while_same_reflection(
                            reflection_diff=22,
                            avoid_obstacles=False,
                            left_reflection_function=lambda: robot.color_fl.rgb()[2],
                            right_reflection_function=lambda: robot.color_fr.rgb()[2],
                        )
                    robot.pid_align(
                        PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                        sensor_function_l=lambda: robot.color_fl.rgb()[2],
                        sensor_function_r=lambda: robot.color_fr.rgb()[2],
                    )

                # todos necessariamente no azul
                robot.pid_walk(cm=10, speed=-80)
                robot.pid_turn(90)
                robot.forward_while_same_reflection(
                    reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: robot.color_fl.rgb()[2],
                    right_reflection_function=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_align(
                    PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                    sensor_function_l=lambda: robot.color_fl.rgb()[2],
                    sensor_function_r=lambda: robot.color_fr.rgb()[2],
                )
                # chega na origem
                robot.pid_walk(cm=10, speed=-80)
                robot.pid_turn(-90)
                robot.forward_while_same_reflection(
                    reflection_diff=22,
                    avoid_obstacles=False,
                    left_reflection_function=lambda: robot.color_fl.rgb()[2],
                    right_reflection_function=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_align(
                    PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
                    sensor_function_l=lambda: robot.color_fl.rgb()[2],
                    sensor_function_r=lambda: robot.color_fr.rgb()[2],
                )
                robot.pid_walk(cm=3, speed=-50)
                robot.pid_turn(-90)
                # normaliza a posicao na origem
                break

        i += 1
        # print(i)
        if i % 2 == 0:
            robot.pid_turn(90)
        else:
            robot.pid_turn(180)


def calibra_pid_align(robot: Robot):
    while True:
        robot.pid_align(
            PIDValues(target=50, kp=0.6, ki=0.005, kd=0.2),
            sensor_function_l=lambda: robot.color_fl.rgb()[2],
            sensor_function_r=lambda: robot.color_fr.rgb()[2],
        )
        wait_button_pressed(robot.brick)
