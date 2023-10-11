from robot import Robot
from utils import PIDValues
from pybricks.tools import wait
from pybricks.parameters import Stop
import constants as const

def passenger_boarding(robot: Robot):
    robot.pid_turn(30)
    robot.pid_walk(cm=2, speed=30)
    robot.pid_line_grabber(PIDValues(target=50, kp=-3.5, ki=-0.05, kd=-10), vel=20, time=3000, sensor=robot.color_fr)
    robot.pid_walk(cm=4, speed=-30)

    robot.stop_mail_box.send(0)
    robot.infra_side_box.wait()
    robot.pid_line_follower(
        vel=60,
        pid=PIDValues(
            target=65,
            kp=0.7,
            ki=0.02,
            kd=0.2,
        ),
        loop_condition=lambda: (
            robot.color_fl.rgb()[2] > 50 and (robot.infra_side_box.read() or 100) > 15
        ),
    )
    robot.stop_mail_box.send(1)

    # Parada 1
    robot.pid_walk(speed=30, cm=1.5)
    robot.pid_turn(90)
    robot.pid_walk(speed=-30, cm=3)

    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
        fix_errors=False
    )
    robot.pid_walk(speed=-30, cm=1)
    robot.pid_align()

    # comeca multiplas leituras
    robot.stop_mail_box.send(0)

    robot.pid_walk(speed=30, cm=5)

    # termina multiplas leituras
    robot.stop_mail_box.send(1)

    # Parada 3
    robot.mbox.wait()

    # recebe info sobre tamanho e cor do passageiro
    passenger_info = robot.mbox.read()
    robot.ev3_print(passenger_info)

    # Parada 4
    robot.pid_walk(speed=-30, cm=10)

    #
    # Retorno para a origem
    #

    # alinha no azul
    robot.forward_while_same_reflection(
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_fl.rgb()[2],
        right_reflection_function=lambda: robot.color_fr.rgb()[2],
        fix_errors=False
    )
    robot.pid_walk(speed=-30, cm=1)
    robot.pid_align()
    robot.pid_walk(speed=-30, cm=10)
    robot.pid_turn(-90)

    # vai até a origem
    robot.forward_while_same_reflection(
        speed_l=-50,
        speed_r=-50,
        reflection_diff=22,
        left_reflection_function=lambda: robot.color_bl.rgb()[2],
        right_reflection_function=lambda: robot.color_br.rgb()[2],
        fix_errors=False,
    )
    robot.pid_walk(speed=30, cm=1)
    robot.pid_align(
        sensor_function_l=lambda: robot.color_bl.rgb()[2],
        sensor_function_r=lambda: robot.color_br.rgb()[2],
        direction_sign=-1
    )
    robot.pid_walk(speed=30, cm=7)

    return passenger_info


def momo_passenger_boarding(robot: Robot):
    while True:
        robot.stop_mail_box.wait_new()
        if robot.stop_mail_box.read() == 0:
            break

    robot.infra_side_box.send(robot.infra_side.distance())
    while robot.stop_mail_box.read() == 0:
        robot.infra_side_box.send(robot.infra_side.distance())
        wait(10)
    

    robot.stop_mail_box.wait_new()
    # comeca multiplas leituras
    distances = []
    while robot.stop_mail_box.read() == 0:
        distances.append(robot.ultra_front.distance())
        wait(10)

    # termina multiplas leituras

    # Parada 1

    # Parada 2
    robot.motor_claw.run_until_stalled(-500, then=Stop.HOLD, duty_limit=const.CLAW_DUTY_LIMIT)

    # Parada 3
    distances.append(robot.ultra_front.distance())
    robot.ev3_print("DIST:", min(distances))
    if min(distances) > 60:
        age = "CHILD"
    else:
        age = "ADULT"
    robot.mbox.send(str(age) + " " + str(robot.color_front.color()))

    # Parada 4