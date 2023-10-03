#!/usr/bin/env pybricks-micropython

# pylint: skip-file

import os
import time

from pybricks.ev3devices import InfraredSensor, Motor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.messaging import (
    BluetoothMailboxClient,
    BluetoothMailboxServer,
    NumericMailbox,
    TextMailbox,
)
from pybricks.parameters import Button, Port
from pybricks.tools import wait


def get_hostname() -> str:
    """
    Retorna o hostname do dispositivo. Feito pensando em verificar o nome do BRICK.
    """
    stream = os.popen("hostname")  # nosec
    return stream.read().split()[0]


def ev3_print(*args, ev3: EV3Brick = None, **kwargs):
    """
    Função para logs.
    Imprime os valores tanto na tela do EV3 (caso disponível) quanto na do terminal.
    """
    if ev3 is not None:
        ev3.screen.print(*args, **kwargs)
    print(*args, **kwargs)


def wait_button_pressed(ev3: EV3Brick, button: Button = Button.CENTER):
    """
    Trava execução até que o botão especificado seja pressionado.
    """
    ev3.speaker.beep(800)
    while True:
        if button in ev3.buttons.pressed():
            break


def momo_main():
    momo = EV3Brick()
    ev3_print("== MOMO ==", ev3=momo)

    ultra = UltrasonicSensor(Port.S4)

    client = BluetoothMailboxClient()
    client.connect("appa")

    mbox = TextMailbox("text", client)
    ultra_box = NumericMailbox("ultra", client)
    stop_mail_box = NumericMailbox("stop", client)

    momo.speaker.beep(200)
    mbox.send("hello appa!")

    momo.speaker.beep()
    mbox.wait()
    ev3_print(mbox.read(), ev3=momo)
    # wait_button_pressed(momo)

    ev3_print("Sending...", ev3=momo)
    stop_mail_box.wait()
    stop = stop_mail_box.read()

    while stop == 0:
        current_read = ultra.distance()
        ultra_box.send(current_read)
        wait(100)
        stop = stop_mail_box.read()
        ev3_print(stop, ev3=momo)


def appa_main():
    appa = EV3Brick()
    ev3_print("== APPA ==", ev3=appa)

    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)

    server = BluetoothMailboxServer()
    server.wait_for_connection()

    mbox = TextMailbox("text", server)
    ultra_box = NumericMailbox("ultra", server)
    stop_mail_box = NumericMailbox("stop", server)

    mbox.wait()
    appa.speaker.beep()

    ev3_print(mbox.read(), ev3=appa)
    appa.speaker.beep(200)
    mbox.send("hello to you!")
    # wait_button_pressed(appa)

    stop_mail_box.send(0)
    ev3_print("Receiving...", ev3=appa)
    while True:
        ultra_box.wait_new()
        distance = ultra_box.read()
        ev3_print(distance, ev3=appa)
        left_motor.run(100)
        right_motor.run(100)
        if distance < 100:
            left_motor.stop()
            right_motor.stop()
            break
    stop_mail_box.send(1)


if get_hostname() == "momo":
    momo_main()
else:
    appa_main()
