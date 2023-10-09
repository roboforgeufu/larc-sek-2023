from robot import Robot
from utils import PIDValues


def passenger_boarding(robot: Robot):
    robot.stop_mail_box.send(0)
    robot.infra_side_box.wait()
    robot.pid_line_follower(
        vel=100,
        pid=PIDValues(
            target=35,
            kp=1,
            ki=0.05,
            kd=10,
        ),
        loop_condition=lambda: (
            robot.color_fl.rgb()[2] > 50 and (robot.infra_side_box.read() or 100) > 3
        ),
    )
    robot.stop_mail_box.send(1)

    # Parada 1
    robot.simple_walk(speed=30, cm=-1.5)
    robot.pid_turn(90)
    robot.simple_walk(speed=30, cm=-3)
    robot.pid_align()
    robot.simple_walk(speed=30, cm=4)

    # Parada 2
    robot.stop_mail_box.send(0)

    # Parada 3
    robot.mbox.wait()

    # recebe info sobre tamanho e cor do passageiro
    passenger_info = robot.mbox.read()
    robot.ev3_print(passenger_info)

    # Parada 4
    robot.simple_walk(speed=30, cm=-10)

    # robot.pid_turn(-90)
    # robot.forward_while_same_reflection(
    #     speed_l=-50,
    #     speed_r=-50,
    #     reflection_diff=22,
    #     left_reflection_function=lambda: robot.color_bl.rgb()[2],
    #     right_reflection_function=lambda: robot.color_br.rgb()[2],
    # )

    #
    # Retorno para a origem
    #

    # alinha no azul
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
    robot.simple_walk(speed=30, cm=-10)
    robot.pid_turn(90)

    # vai até a origem
    robot.forward_while_same_reflection(
        reflection_diff=22,
        avoid_obstacles=False,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
    )
    robot.simple_walk(speed=30, cm=-10)
    robot.pid_turn(90)

    # restaura a posicao inicial
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
    robot.simple_walk(speed=30, cm=-10)
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
    robot.simple_walk(speed=30, cm=-10)
    return passenger_info
