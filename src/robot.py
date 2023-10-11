"""
Módulo central pra controle do Robô.

Devem estar nesse módulo:
    - Classe 'Robot', com métodos e atributos para controle geral no robô
    - Estruturas de dados auxiliares aplicáveis a "qualquer" tipo de robô

Não devem estar nesse módulo:
    - Código específico de algum problema/desafio
"""


import math

from pybricks.ev3devices import ColorSensor, InfraredSensor, Motor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.messaging import (
    BluetoothMailboxClient,
    BluetoothMailboxServer,
    NumericMailbox,
    TextMailbox,
)
from pybricks.parameters import Color, Port
from pybricks.tools import StopWatch, wait

import constants as const
from sensor_decision_trees import (
    DecisionColorSensor,
    momo_s3_decision_tree,
    s1_decision_tree,
    s2_decision_tree,
    s3_decision_tree,
    s4_decision_tree,
)
from utils import PIDValues, between, get_hostname, normalize_color, wait_button_pressed


class Robot:
    """
    Classe que representa um robô genérico.
    """

    def __init__(
        self,
        wheel_diameter: float = const.WHEEL_DIAMETER,
        wheel_distance: float = const.WHEEL_DIST,
        motor_r: Port = None,
        motor_l: Port = None,
        motor_claw: Port = None,
        infra_side: Port = None,
        ultra_back: Port = None,
        ultra_front: Port = None,
        color_br: Port = None,
        color_bl: Port = None,
        color_fr: Port = None,
        color_fl: Port = None,
        color_front: Port = None,
        debug: bool = False,
        turn_correction: float = 1,
        color_max_value: float = 100,
        connect_to: str = "appa",
        is_server: bool = False,
    ) -> None:
        # Debug
        self.debug = debug

        # Brick EV3
        self.brick = EV3Brick()
        self.name = get_hostname()
        self.ev3_print("==", self.name.upper(), "==")

        # Medidas do robô
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance
        self.turn_correction = turn_correction
        self.color_max_value = color_max_value

        # Cronometro
        self.stopwatch = StopWatch()

        # Motores
        if motor_r is not None:
            self.motor_r = Motor(motor_r)
        if motor_l is not None:
            self.motor_l = Motor(motor_l)
        if motor_claw is not None:
            self.motor_claw = Motor(motor_claw)

        # Sensores infra vermelhos
        if infra_side is not None:
            self.infra_side = InfraredSensor(infra_side)
        if ultra_back is not None:
            self.ultra_back = UltrasonicSensor(ultra_back)

        # Sensores ultrassonicos
        if ultra_front is not None:
            self.ultra_front = UltrasonicSensor(ultra_front)

        # Sensores de cor
        if color_fl is not None:
            self.color_fl = DecisionColorSensor(color_fl, s1_decision_tree)
        if color_fr is not None:
            self.color_fr = DecisionColorSensor(color_fr, s2_decision_tree)
        if color_bl is not None:
            self.color_bl = DecisionColorSensor(color_bl, s3_decision_tree)
        if color_br is not None:
            self.color_br = DecisionColorSensor(color_br, s4_decision_tree)
        if color_front is not None:
            self.color_front = DecisionColorSensor(color_front, momo_s3_decision_tree)

        if is_server:
            # APPA
            # setup
            self.server = BluetoothMailboxServer()
            self.server.wait_for_connection()

            # mailboxes
            self.mbox = TextMailbox("text", self.server)
            self.infra_side_box = NumericMailbox("ultra", self.server)
            self.front_obstacle_box = NumericMailbox("f_obstacle", self.server)
            self.back_obstacle_box = NumericMailbox("b_obstacle", self.server)
            self.color_front_box = NumericMailbox("color", self.server)
            self.stop_mail_box = NumericMailbox("stop", self.server)

            # handshake
            self.mbox.wait()
            self.brick.speaker.beep()

            self.ev3_print(self.mbox.read())
            self.brick.speaker.beep(200)
            self.mbox.send("hello to you, client!")
        else:
            # MOMO
            # setup
            self.client = BluetoothMailboxClient()
            self.client.connect(connect_to)

            # mailboxes
            self.mbox = TextMailbox("text", self.client)
            self.infra_side_box = NumericMailbox("ultra", self.client)
            self.front_obstacle_box = NumericMailbox("f_obstacle", self.client)
            self.back_obstacle_box = NumericMailbox("b_obstacle", self.client)
            self.color_front_box = NumericMailbox("color", self.client)
            self.stop_mail_box = NumericMailbox("stop", self.client)

            # handshake
            self.brick.speaker.beep(200)
            self.mbox.send("hello server!")
            self.mbox.wait()
            self.brick.speaker.beep()
            self.ev3_print(self.mbox.read())

    def robot_axis_to_motor_degrees(self, axis_degrees: float):
        """
        Grau relativo ao eixo do robô -> grau nas rodas (motores) do robô

        Considera um possível fator de correção
        """
        return (
            axis_degrees
            * (self.wheel_distance / self.wheel_diameter)
            * self.turn_correction
        )

    def cm_to_motor_degrees(self, cm: float):
        """Distância em centímetros -> grau nas rodas (motores) do robô"""
        return cm * (360 / (math.pi * self.wheel_diameter))

    def motor_degrees_to_cm(self, degrees: float):
        """Grau nas rodas (motores) do robô -> Distância em centímetros"""
        return degrees * ((math.pi * self.wheel_diameter) / 360)

    def off_motors(self):
        """Desliga motores de locomoção."""
        self.motor_l.dc(0)
        self.motor_r.dc(0)
        self.motor_l.hold()
        self.motor_r.hold()
        self.motor_l.dc(0)
        self.motor_r.dc(0)

    def ev3_print(self, *args, clear=False, always: bool = False, **kwargs):
        """
        Métodos para logs.
        """
        if self.debug or always:
            if clear:
                wait(10)
                self.brick.screen.clear()
            self.brick.screen.print(*args, **kwargs)
            print(*args, **kwargs)

    def forward_while_same_reflection(
        self,
        speed_r=50,
        speed_l=50,
        reflection_diff=10,
        obstacle_function=None,
        pid: PIDValues = PIDValues(
            kp=1,
            ki=0.001,
            kd=1,
        ),
        left_reflection_function=None,
        right_reflection_function=None,
        fix_errors=True,
    ):
        """
        Move ambos os motores (de forma individual) até que a intensidade de reflexão
        mude o suficiente (`reflection_diff`)

        Retorna um booleano representando se parou por obstáculo
        """
        # self.ev3_print(self.forward_while_same_reflection.__name__)

        if left_reflection_function is None:
            left_reflection_function = self.color_fl.reflection
        if right_reflection_function is None:
            right_reflection_function = self.color_fr.reflection

        starting_ref_r = right_reflection_function()
        starting_ref_l = left_reflection_function()

        initial_motor_l_angle = self.motor_l.angle()
        initial_motor_r_angle = self.motor_r.angle()

        motor_error_i = 0
        prev_motor_error = 0

        has_seen_obstacle = False

        stopped_l = False
        stopped_r = False

        diff_ref_r = 0
        diff_ref_l = 0
        while abs(diff_ref_l) < reflection_diff or abs(diff_ref_r) < reflection_diff:
            if obstacle_function is not None and obstacle_function():
                has_seen_obstacle = True
                break

            diff_ref_r = right_reflection_function() - starting_ref_r
            diff_ref_l = left_reflection_function() - starting_ref_l

            # Controle PID entre os motores
            if (
                abs(diff_ref_l) < reflection_diff and abs(diff_ref_r) < reflection_diff
            ) and (speed_l == speed_r):
                motor_diff = (self.motor_r.angle() - initial_motor_r_angle) - (
                    self.motor_l.angle() - initial_motor_l_angle
                )
                motor_error = motor_diff

                motor_error_i += motor_error
                motor_error_d = motor_error - prev_motor_error
                prev_motor_error = motor_error

                # self.ev3_print(
                #     motor_error,
                #     motor_error_i,
                #     motor_error_d,
                # )
                pid_speed = (
                    pid.kp * motor_error
                    + pid.ki * motor_error_i
                    + pid.kd * motor_error_d
                )
            else:
                pid_speed = 0
            ###

            # self.ev3_print(diff_ref_l, diff_ref_r)

            if abs(diff_ref_r) < reflection_diff:
                self.motor_r.dc(speed_r - pid_speed)
            elif fix_errors:
                direction_sign =(speed_r) / abs(speed_r)
                self.motor_r.dc(
                    speed_r + pid_speed + (const.FORWARD_SPEED_CORRECTION * direction_sign)
                )
                initial_motor_r_angle = self.motor_r.angle()
                initial_motor_l_angle = self.motor_l.angle()
            else:
                if not stopped_r:
                    stopped_r = True
                self.motor_r.hold()

            if abs(diff_ref_l) < reflection_diff:
                self.motor_l.dc(speed_l + pid_speed)
            elif fix_errors:
                self.motor_l.dc(
                    ((speed_l) / abs(speed_l))
                    * (speed_l + pid_speed + const.FORWARD_SPEED_CORRECTION)
                )
                initial_motor_l_angle = self.motor_l.angle()
                initial_motor_r_angle = self.motor_r.angle()
            else:
                if not stopped_l:
                    stopped_l = True
                self.motor_l.hold()

            if stopped_r and stopped_l:
                break

        self.off_motors()
        return has_seen_obstacle

    def simple_turn(
        self, angle, speed=50, look_around_function=None, motor_correction=None
    ):
        """
        Curva simples.


        Caso look_around_function seja passado, retorna uma lista com todas
        as leituras durante a curva.

        A lista retornada contém tuplas com: (valores lidos, motor_r, motor_l).

        motor_correction é um valor de 0 a 1
        O sinal de motor_correction define em qual motor a correção será aplicada:
        - Negativo: motor direito
        - Positivo: motor esquerdo
        """
        dir_sign = 1 if angle > 0 else -1
        speed = abs(speed)

        motor_degrees = self.robot_axis_to_motor_degrees(abs(angle))
        if motor_correction is not None:
            if motor_correction > 0:
                motor_degrees_l = motor_degrees * abs(motor_correction)
                motor_degrees_r = motor_degrees
            else:
                motor_degrees_r = motor_degrees * abs(motor_correction)
                motor_degrees_l = motor_degrees
        else:
            motor_degrees_l = motor_degrees
            motor_degrees_r = motor_degrees

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        reads = []

        while (
            abs(self.motor_l.angle() - initial_angle_l) < motor_degrees_l
            or abs(self.motor_r.angle() - initial_angle_r) < motor_degrees_r
        ):
            if look_around_function is not None:
                reads.append(
                    (
                        look_around_function(),
                        self.motor_r.angle(),
                        self.motor_l.angle(),
                    )
                )

            if abs(self.motor_r.angle() - initial_angle_r) < motor_degrees:
                self.motor_r.dc(dir_sign * -speed)
            else:
                self.motor_r.dc(0)

            if abs(self.motor_l.angle() - initial_angle_l) < motor_degrees:
                self.motor_l.dc(dir_sign * speed)
            else:
                self.motor_l.dc(0)

        self.off_motors()
        return reads

    def one_wheel_turn(self, motor: Motor, angle: int, speed: int):
        """Curva com um motor"""
        dir_sign = 1 if angle > 0 else -1
        initial_angle = motor.angle()
        while abs(motor.angle() - initial_angle) < self.robot_axis_to_motor_degrees(
            abs(angle) * 2
        ):
            motor.dc(speed * dir_sign)
        self.off_motors()

    def one_wheel_turn_till_color(self, motor: Motor, sensor_color, target_color):
        """Curva com um motor até que um sensor alvo veja uma cor alvo"""
        motor.reset_angle(0)
        vel = 50
        while self.accurate_color(sensor_color.rgb()) != target_color:
            motor.dc(vel)
        self.off_motors()
        return motor.angle()

    def turn_till_color(self, direction, sensor_color, target_color):
        """Curva em torno do próprio eixo até que um sensor alvo veja uma cor alvo"""
        sign = 1 if direction == "right" else -1
        vel = 50
        while self.accurate_color(sensor_color.rgb()) != target_color:
            self.motor_l.dc(sign * vel)
            self.motor_r.dc(sign * (-vel))
        self.off_motors()

    def certify_line_alignment_routine(self, motor: Motor, sensor_color, target_color):
        degrees = self.one_wheel_turn_till_color(motor, sensor_color, target_color)
        self.pid_walk(cm=const.WHEEL_DIAMETER, speed=30)
        motor.reset_angle(0)
        target_motor = self.motor_r if motor == self.motor_l else self.motor_l
        target_motor.run_target(100, -220 + degrees)
        self.line_grabber(vel=20, time=3000, sensor=sensor_color)

    def move_both_to_target(
        self,
        target=None,
        target_l=None,
        target_r=None,
        tolerable_diff=2,
        speed=30,
    ):
        """
        Move os dois motores simultaneamente para um ângulo alvo (que pode ser
        diferente entre eles).

        Suporta curvas!
        """
        if target is not None:
            target_l = target
            target_r = target

        while (
            abs(self.motor_l.angle() - target_l) > tolerable_diff
            or abs(self.motor_r.angle() - target_r) > tolerable_diff
        ):
            if abs(self.motor_l.angle() - target_l) > tolerable_diff:
                dir_sign_l = -1 if (self.motor_l.angle() - target_l) > 0 else 1
                self.motor_l.dc(speed * dir_sign_l)
            else:
                self.motor_l.dc(0)

            if abs(self.motor_r.angle() - target_r) > tolerable_diff:
                dir_sign_r = -1 if (self.motor_r.angle() - target_r) > 0 else 1
                self.motor_r.dc(speed * dir_sign_r)
            else:
                self.motor_r.dc(0)
        self.off_motors()

    def pid_turn(
        self,
        angle,
        mode=1,
        pid: PIDValues = PIDValues(
            kp=0.8,
            ki=0.01,
            kd=0.4,
        ),
    ):
        """
        Curva com controle PID.
        - Angulo relativo ao eixo do robô.
        - Angulo negativo: curva p / esquerda
        - Angulo positivo: curva p / direita
        - Modos(mode):
            - 1: usa o valor dado como ângulo ao redor do eixo do robô
            - 2: usa o valor dado como ângulo no eixo das rodas
        """
        # self.ev3_print(self.pid_turn.__name__)

        if mode == 1:
            motor_degrees = self.robot_axis_to_motor_degrees(angle)
        if mode == 2:
            motor_degrees = angle

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        target_angle_r = initial_angle_r - motor_degrees
        target_angle_l = initial_angle_l + motor_degrees

        # self.ev3_print("INICIAL:", initial_angle_l, initial_angle_r)
        # self.ev3_print("TARGET:", target_angle_l, target_angle_r)

        left_error = target_angle_l - self.motor_l.angle()
        right_error = target_angle_r - self.motor_r.angle()

        left_error_i = 0
        right_error_i = 0

        left_prev_error = left_error
        right_prev_error = right_error

        n = 0
        while (
            int(abs(self.motor_l.angle() - target_angle_l))
            > const.PID_TURN_ACCEPTABLE_DEGREE_DIFF
            or int(abs(self.motor_r.angle() - target_angle_r))
            > const.PID_TURN_ACCEPTABLE_DEGREE_DIFF
        ):
            n += 1
            left_error = target_angle_l - self.motor_l.angle()
            right_error = target_angle_r - self.motor_r.angle()

            if abs(left_error) < 30:
                left_error_i += left_error
            if abs(right_error) < 30:
                right_error_i += right_error

            left_error_d = left_prev_error - left_error
            right_error_d = right_prev_error - right_error

            left_prev_error = left_error
            right_prev_error = right_error

            left_pid_speed = (
                pid.kp * left_error + pid.ki * left_error_i + pid.kd * left_error_d
            )
            right_pid_speed = (
                pid.kp * right_error + pid.ki * right_error_i + pid.kd * right_error_d
            )
            # self.ev3_print(
            #     self.motor_l.angle(),
            #     self.motor_r.angle(),
            #     "|",
            #     left_error,
            #     left_error_i,
            #     left_error_d,
            #     left_pid_speed,
            # )

            # Limitante de velocidade
            left_speed_sign = -1 if left_pid_speed < 0 else 1
            left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign
            right_speed_sign = -1 if right_pid_speed < 0 else 1
            right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

            self.motor_l.dc(left_pid_speed)
            self.motor_r.dc(right_pid_speed)

            left_wheel_angle_distance = self.motor_l.angle() - initial_angle_l
            right_wheel_angle_distance = self.motor_r.angle() - initial_angle_r

            # self.ev3_print("C:", self.motor_l.speed(), self.motor_r.speed())
            if (
                abs(self.motor_l.speed()) < const.PID_TURN_MIN_SPEED
                and abs(self.motor_r.speed()) < const.PID_TURN_MIN_SPEED
                and abs(left_wheel_angle_distance) > const.MIN_DEGREES_CURVE_THRESHOLD
                and abs(right_wheel_angle_distance) > const.MIN_DEGREES_CURVE_THRESHOLD
            ):
                break
        self.off_motors()
        # self.ev3_print(n, "| END:", self.motor_l.angle(), self.motor_r.angle())

    def simple_walk(self, cm, speed=50, speed_l=None, speed_r=None):
        """Movimentação simples"""
        dir_sign = 1 if cm > 0 else -1

        speed = abs(speed)
        if speed_l is None:
            speed_l = speed
        if speed_r is None:
            speed_r = speed
        speed_l = abs(speed_l)
        speed_r = abs(speed_r)

        degrees = self.cm_to_motor_degrees(cm)

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        while abs(initial_angle_l - self.motor_l.angle()) < abs(degrees) or abs(
            initial_angle_r - self.motor_r.angle()
        ) < abs(degrees):
            if abs(initial_angle_l - self.motor_l.angle()) < abs(degrees):
                self.motor_l.dc(speed_l * dir_sign)
            else:
                self.motor_l.dc(0)

            if abs(initial_angle_r - self.motor_r.angle()) < abs(degrees):
                self.motor_r.dc(speed_l * dir_sign)
            else:
                self.motor_r.dc(0)
        self.off_motors()

    def pid_walk(
        self,
        cm,
        speed=60,
        obstacle_function=None,
        fix_errors=False,
    ):
        """Anda em linha reta com controle PID entre os motores."""

        # wait_button_pressed(self.brick)
        
        degrees = self.cm_to_motor_degrees(cm)

        elapsed_time = 0
        i_share = 0
        error = 0
        motor_angle_average = 0
        initial_left_angle = self.motor_l.angle()
        imutable_initial_left_angle = self.motor_l.angle()
        initial_right_angle = self.motor_r.angle()
        imutable_initial_right_angle = self.motor_r.angle()
        self.stopwatch.reset()

        if speed < 0:
            right_sensor = self.color_bl
            left_sensor = self.color_br
            right_motor = self.motor_l
            left_motor = self.motor_r
        else:
            right_sensor = self.color_fr
            left_sensor = self.color_fl
            right_motor = self.motor_r
            left_motor = self.motor_l


        initial_left_read = left_sensor.rgb()[2]
        initial_right_read = right_sensor.rgb()[2]

        left_read_diff = lambda: abs(left_sensor.rgb()[2] - initial_left_read)
        right_read_diff = lambda: abs(right_sensor.rgb()[2] - initial_right_read)

        has_seen_obstacle = False
        while abs(motor_angle_average) < abs(degrees):
            if obstacle_function is not None and obstacle_function():
                has_seen_obstacle = True
                break
            
            motor_angle_average = ((self.motor_l.angle() - imutable_initial_left_angle) + (self.motor_r.angle() - imutable_initial_right_angle)) / 2

            if fix_errors and right_read_diff() > 22:
                # Vendo estab. com sensor direito
                # self.ev3_print(1)
                direction_sign =(speed) / abs(speed)
                right_motor.dc(speed + (const.FORWARD_SPEED_CORRECTION * direction_sign))
                initial_left_angle = self.motor_l.angle()
                initial_right_angle = self.motor_r.angle()

            if fix_errors and left_read_diff() > 22:
                # self.ev3_print(2)
                # Vendo estab. com sensor esquerdo
                direction_sign =(speed) / abs(speed)
                left_motor.dc(speed + (const.FORWARD_SPEED_CORRECTION * direction_sign))
                initial_left_angle = self.motor_l.angle()
                initial_right_angle = self.motor_r.angle()

            if (
                not fix_errors or
                (
                    (not right_read_diff() > 22) and (not left_read_diff() > 22)
                )
            ):
                # self.ev3_print(3)
                elapsed_time, i_share, error = self.loopless_pid_walk(
                    elapsed_time, i_share, error,
                    vel=speed,
                    initial_left_angle=initial_left_angle,
                    initial_right_angle=initial_right_angle,
                )

        self.off_motors()
        # wait_button_pressed(self.brick)
        return has_seen_obstacle

    def loopless_pid_walk(
        self,
        prev_elapsed_time=0,
        i_share=0,
        prev_error=0,
        vel=60,
        pid: PIDValues = PIDValues(
            kp=3,
            ki=0.2,
            kd=8,
        ),
        initial_left_angle = 0,
        initial_right_angle = 0,
    ):
        """
        Controle PID entre os motores sem um loop específico.
        Feita pra ser colocada dentro de um loop em outra função, passando os
        novos parâmetros (prev_elapsed_time, i_share, prev_error) devidamente
        inicializados a cada iteração.
        """
        error = (self.motor_r.angle() - initial_right_angle) - (
            self.motor_l.angle() - initial_left_angle
        )
        p_share = error * pid.kp

        if abs(error) < 3:
            i_share = i_share + (error * pid.ki)

        wait(1)
        elapsed_time = self.stopwatch.time()

        d_share = ((error - prev_error) * pid.kd) / (elapsed_time - prev_elapsed_time)

        pid_correction = p_share + i_share + d_share
        self.motor_r.dc(vel - pid_correction)
        self.motor_l.dc(vel + pid_correction)

        return (elapsed_time, i_share, error)

    def pid_accelerated_walk(
        self,
        time,
        mode,
        pid: PIDValues = PIDValues(
            kp=3,
            ki=0.02,
            kd=3,
        ),
    ):
        """
        Linha reta acelerada baseada em modos com correção PID entre os motores.

        - Modo 1: termina com velocidade 0
        - Modo 2: termina com velocidade máxima
        - Modo 3: começa com velocidade máxima, termina com velocidade 0
        """

        elapsed_time = 0
        i_share = 0.0
        error = 0

        self.stopwatch.reset()
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)

        while self.stopwatch.time() < abs(time):
            prev_error = error
            error = self.motor_r.angle() - self.motor_l.angle()
            p_share = error * pid.kp

            if abs(error) < 3:
                i_share = i_share + (error * pid.ki)

            prev_elapsed_time = elapsed_time
            wait(1)
            elapsed_time = self.stopwatch.time()
            d_share = ((error - prev_error) * pid.kd) / (
                elapsed_time - prev_elapsed_time
            )

            pid_correction = p_share + i_share + d_share

            if mode == 1:
                a = 1.0
                b = 1.0
                c = 0.0
            elif mode == 2:
                a = 0.5
                b = 0.5
                c = 0.0
            elif mode == 3:
                a = 0.5
                b = 0.0
                c = 80.0

            vel = (
                -((elapsed_time * a * 20 / abs(time)) ** 2)
                + elapsed_time * (b * 400 / abs(time))
                + (c + 20)
            )
            sign = -1 if time < 0 else 1
            self.motor_l.dc(sign * vel + pid_correction)
            self.motor_r.dc(sign * vel - pid_correction)

        self.off_motors()

    def pid_align(
        self,
        pid: PIDValues = PIDValues(target=65, kp=0.7, ki=0.02, kd=0.2),
        sensor_function_l=None,
        sensor_function_r=None,
        direction_sign=1,
    ):
        """
        Alinha usando os dois pares (motor - sensor de cor) e controle PID.
        """
        if sensor_function_l is None:
            sensor_function_l = lambda: self.color_fl.rgb()[2]
        if sensor_function_r is None:
            sensor_function_r = lambda: self.color_fr.rgb()[2]
        correction_factor = 0.9
        left_error_i = 0
        right_error_i = 0
        left_prev_error = 0
        right_prev_error = 0

        has_stopped_left = False
        has_stopped_right = False

        while not has_stopped_left and not has_stopped_right:
            left_error = sensor_function_l() - pid.target
            right_error = sensor_function_r() - (pid.target * correction_factor)

            left_error_i += left_error
            right_error_i += right_error

            left_error_d = left_error - left_prev_error
            right_error_d = right_error - right_prev_error

            left_prev_error = left_error
            right_prev_error = right_error

            left_pid_speed = (
                pid.kp * left_error + pid.ki * left_error_i + pid.kd * left_error_d
            )
            right_pid_speed = (
                pid.kp * right_error + pid.ki * right_error_i + pid.kd * right_error_d
            )

            # Limitante de velocidade
            left_speed_sign = -1 if left_pid_speed < 0 else 1
            left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign

            right_speed_sign = -1 if right_pid_speed < 0 else 1
            right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

            self.motor_l.dc(left_pid_speed * direction_sign)
            self.motor_r.dc(right_pid_speed * direction_sign)

            # self.ev3_print(left_error, right_error)
            if abs(left_error) <= 7 and abs(right_error) <= 7:
                break
        self.off_motors()

    def line_grabber(self, sensor, time, vel=20, multiplier=1.5):
        color_reads = []
        num_reads = 10
        wrong_read_perc = 0.5
        color_count_perc = 0.5
        self.stopwatch.reset()
        self.reset_both_motor_angles()
        while True:
            sign = 1 if sensor == self.color_fl else -1
            if (
                (self.accurate_color(sensor.rgb()) != Color.WHITE)
                and (self.accurate_color(sensor.rgb()) != Color.BLACK)
                and (self.accurate_color(sensor.rgb()) != "None")
            ):
                sign = sign * (-1)

            color_read = self.accurate_color(sensor.rgb())
            color_reads.append(color_read)
            left_multiplier = multiplier
            right_multiplier = multiplier
            if len(color_reads) == num_reads:
                black_count_perc = (color_reads.count(Color.BLACK)) / num_reads
                white_count_perc = (color_reads.count(Color.WHITE)) / num_reads
                wrong_read_perc = black_count_perc + white_count_perc
                color_count_perc = 1 - wrong_read_perc
                color_reads.clear()

            self.motor_r.dc(vel + (vel * wrong_read_perc * right_multiplier * sign))
            self.motor_l.dc(vel - (vel * color_count_perc * left_multiplier * sign))
            print(
                vel + (vel * wrong_read_perc * right_multiplier * sign),
                vel - (vel * color_count_perc * left_multiplier * sign),
            )

            motor_mean = (self.motor_l.angle() + self.motor_r.angle()) / 2

            if color_read == "None":
                self.off_motors()
                return motor_mean

            if self.stopwatch.time() > time:
                self.off_motors()
                return motor_mean

    def pid_line_grabber(  # pylint: disable=invalid-name
        self,
        pid: PIDValues = PIDValues(
            target=35,  # medir na linha toda vez
            kp=3.5,
            ki=0.05,
            kd=10,
        ),
        vel = 20,
        time = 3000,
        sensor: ColorSensor = None,
    ):
        """
        O robô usa um dos sensores de cor para encontrar a linha e entrar em posição
        ideal antes de iniciar o algoritmo de seguidor de linha.
        """

        error = 0
        i_share = 0.0
        elapsed_time = 0

        self.stopwatch.reset()
        while True:
            prev_error = error
            error = pid.target - sensor.reflection()
            p_share = error * pid.kp

            if abs(error) < 3:
                i_share = (i_share + error) * pid.ki

            prev_elapsed_time = elapsed_time
            wait(1)
            elapsed_time = self.stopwatch.time()
            d_share = ((error - prev_error) * pid.kd) / (
                elapsed_time - prev_elapsed_time
            )

            pid_correction = p_share + i_share + d_share

            pid_sign = 1 if sensor == self.color_fl else -1
            self.motor_r.run(vel + (pid_correction * pid_sign))
            self.motor_l.run(vel - (pid_correction * pid_sign))

            ref_read = sensor.reflection()

            if self.stopwatch.time() > time:
                break

            if ref_read < 5:
                break

        self.motor_l.hold()
        self.motor_r.hold()

    def pid_line_follower(
        self,
        sensor_function=None,
        vel=50,
        direction="right",
        pid=PIDValues(
            target=35,
            kp=3.5,
            ki=0.05,
            kd=10,
        ),
        loop_condition=None,
    ):
        """
        Seguidor de linha com controle PID.
        """
        error = 0
        i_share = 0.0

        if sensor_function is None:
            sensor_function = self.color_fr.reflection
        if loop_condition is None:
            loop_condition = lambda: True

        while loop_condition():
            prev_error = error
            error = pid.target - sensor_function()

            p_share = error * pid.kp
            i_share = (i_share + error) * pid.ki
            d_share = (error - prev_error) * pid.kd

            pid_correction = p_share + i_share + d_share

            pid_sign = 1 if direction == "right" else -1
            self.motor_r.dc(vel + (pid_correction * pid_sign))
            self.motor_l.dc(vel - (pid_correction * pid_sign))

        self.off_motors()

    def line_follower_color_id(self, sensor, vel=50, array=None, break_color="None"):
        color_reads = []
        all_color_reads = []
        num_reads = 10
        wrong_read_perc = 0.5
        color_count_perc = 0.5
        if array is None:
            array = []
        while True:
            sign = 1 if sensor == self.color_fl else -1
            if (
                (self.accurate_color(sensor.rgb()) != Color.WHITE)
                and (self.accurate_color(sensor.rgb()) != Color.BLACK)
                and (self.accurate_color(sensor.rgb()) != "None")
            ):
                sign = sign * (-1)

            color_read = self.accurate_color(sensor.rgb())
            color_reads.append(color_read)
            left_multiplier = 0.33
            right_multiplier = 0.33
            if len(color_reads) == num_reads:
                black_count_perc = (color_reads.count(Color.BLACK)) / num_reads
                white_count_perc = (color_reads.count(Color.WHITE)) / num_reads
                green_count_perc = (color_reads.count(Color.GREEN)) / num_reads
                wrong_read_perc = black_count_perc + white_count_perc + green_count_perc
                color_count_perc = 1 - wrong_read_perc
                all_color_reads.extend(color_reads)
                color_reads.clear()

            self.motor_r.dc(vel + (vel * wrong_read_perc * right_multiplier * sign))
            self.motor_l.dc(vel - (vel * color_count_perc * left_multiplier * sign))

            # if color_read not in array and color_read in valid_colors:
            if len(all_color_reads) > 50:
                y_count = all_color_reads.count(Color.YELLOW)
                r_count = all_color_reads.count(Color.RED)
                b_count = all_color_reads.count(Color.BLUE)
                max_count = max(y_count, r_count, b_count)
                if max_count == y_count:
                    if Color.YELLOW not in array:
                        array.append(Color.YELLOW)
                elif max_count == r_count:
                    if Color.RED not in array:
                        array.append(Color.RED)
                elif max_count == b_count:
                    if Color.BLUE not in array:
                        array.append(Color.BLUE)
                all_color_reads.clear()

            if break_color == "None" and sensor == self.color_fl:
                if len(array) >= 2:
                    break
            else:
                if color_read == break_color:
                    break

            if color_read == "None":
                break

        self.off_motors()

        return array

    def min_aligner(
        self,
        min_function,
        speed: int = 40,
        max_angle=90,
        acceptable_range=0,
        motor_correction=None,
    ):
        """
        Alinha os motores usando o mínimo de uma função como alvo.

        O argumento `min_function` deve ser a função a ser "minimizada"
        (infra_sensor.distance, por exemplo).
        """
        infra_reads = self.simple_turn(
            -(max_angle / 2),
            speed=speed,
            look_around_function=min_function,
            motor_correction=motor_correction,
        )
        second_reads = self.simple_turn(
            max_angle,
            speed=speed,
            look_around_function=min_function,
            motor_correction=motor_correction,
        )
        infra_reads.extend(second_reads)

        min_read = min(i_read for i_read, _, _ in infra_reads)
        close_reads = [
            read
            for read in infra_reads
            if read[0] in range(min_read, min_read + acceptable_range + 1)
        ]

        motor_r_mean = sum(motor_r_angle for _, motor_r_angle, _ in close_reads) / len(
            close_reads
        )
        motor_l_mean = sum(motor_l_angle for _, _, motor_l_angle in close_reads) / len(
            close_reads
        )

        self.move_both_to_target(target_l=motor_l_mean, target_r=motor_r_mean)

    def move_to_distance(
        self,
        distance: float,
        sensor: UltrasonicSensor,
        pid=PIDValues(kp=1, ki=0.0001, kd=0.01),
        turning=0,
        max_cm=None,
        safe_max_read=None,
    ):
        """
        Se move até ler determinada distância com o sensor dado.

        - turning é um valor percentual (0 a 1) positivo ou negativo que
        representa o quanto a força será diferente entre dois motores, a fim de
        fazer uma curva.
        - single_motor é um motor opcional caso queira executar o movimento apenas com o motor
        desejado. Se não for passado, os dois motores básicos são usados por padrão.
        """
        # self.ev3_print(self.move_to_distance.__name__, distance)

        if max_cm is not None:
            max_motor_degrees = self.cm_to_motor_degrees(max_cm)
        else:
            max_motor_degrees = 0

        initial_degrees_l = self.motor_l.angle()
        initial_degrees_r = self.motor_r.angle()

        initial_time = self.stopwatch.time()

        diff = sensor.distance() - distance
        diff_i = 0
        prev_diff = diff
        while abs(diff) >= 1 and (
            safe_max_read is None or abs(sensor.distance()) < safe_max_read
        ):
            diff = sensor.distance() - distance
            diff_i += diff
            diff_d = diff - prev_diff

            pid_speed = diff * pid.kp + diff_i * pid.ki + diff_d * pid.kd

            # self.ev3_print(diff, diff_i, diff_d, pid_speed)
            if abs(pid_speed) < 10:
                break

            self.motor_l.dc(pid_speed * (1 + turning))
            self.motor_r.dc(pid_speed * (1 - turning))

            # self.ev3_print(
            #     abs(initial_degrees_l - self.motor_l.angle()),
            #     abs(initial_degrees_r - self.motor_r.angle()),
            # )
            if max_cm is not None and (
                abs(initial_degrees_l - self.motor_l.angle()) > max_motor_degrees
                or abs(initial_degrees_r - self.motor_r.angle()) > max_motor_degrees
            ):
                break

            # self.ev3_print("MV_DIST:", self.motor_l.speed(), self.motor_r.speed())
            if (
                abs(self.motor_l.speed()) < const.PID_TURN_MIN_SPEED
                and abs(self.motor_r.speed()) < const.PID_TURN_MIN_SPEED
                and (
                    self.stopwatch.time() - initial_time
                    > const.MV_TO_DIST_THRESHOLD_TIME
                )
            ):
                break

        # self.ev3_print(sensor.distance())
        self.off_motors()

    def accurate_color(self, rgb_tuple):
        """
        Processamento de cor pra evitar os erros da leitura padrão.
        """
        red_value = rgb_tuple[0]
        green_value = rgb_tuple[1]
        blue_value = rgb_tuple[2]

        red_normalized_value = normalize_color(red_value, self.color_max_value)
        green_normalized_value = normalize_color(green_value, self.color_max_value)
        blue_normalized_value = normalize_color(blue_value, self.color_max_value)

        if (
            between(red_normalized_value, 0.11, 0.15)
            and between(green_normalized_value, 0.36, 0.44)
            and between(blue_normalized_value, 0.48, 0.72)
        ):
            return Color.BLUE
        elif (
            green_normalized_value != 0
            and red_normalized_value / green_normalized_value <= 0.5
            and blue_normalized_value / green_normalized_value <= 0.5
        ):
            return Color.GREEN
        elif (
            between(red_normalized_value, 0.71, 0.8)
            and between(green_normalized_value, 0.46, 0.5)
            and between(blue_normalized_value, 0.07, 0.14)
        ):
            return Color.YELLOW
        elif (
            red_normalized_value == 0
            and green_normalized_value == 0
            and blue_normalized_value == 0
        ):
            return "None"
        elif (
            between(red_normalized_value, 0.7, 1)
            and between(green_normalized_value, 0.7, 1)
            and between(blue_normalized_value, 0.7, 1)
        ):
            return Color.WHITE
        elif (
            between(red_normalized_value, 0.57, 0.73)
            and between(green_normalized_value, 0.05, 0.07)
            and between(blue_normalized_value, 0, 0.05)
        ):
            return Color.RED
        else:
            return Color.BLACK

    def get_average_reading(self, sensor_func, num_reads=100):
        measures = []
        for _ in range(num_reads):
            measures.append(sensor_func())
        mean = sum(measures) / len(measures)
        return mean

    def reset_both_motor_angles(self):
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)
