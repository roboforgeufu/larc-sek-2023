from pybricks.ev3devices import ColorSensor, InfraredSensor, Motor, UltrasonicSensor
from pybricks.parameters import Color, Port, Stop


def s1_decision_tree(rgb_tuple):
    r, g, b = rgb_tuple
    if g <= 15.5:
        if r <= 18.00:
            return Color.BLACK
        else:
            if r <= 47.00:
                return Color.BROWN
            else:
                return Color.RED
    else:
        if r <= 42.50:
            if g <= 24.50:
                return Color.BLUE
            if g > 24.50:
                return Color.GREEN
        else:
            if r <= 71.50:
                return Color.YELLOW
            else:
                return Color.WHITE


def s2_decision_tree(rgb_tuple):
    r, g, b = rgb_tuple
    if g <= 20.50:
        if r <= 20.00:
            return Color.BLACK
        else:
            if r <= 53.00:
                return Color.BROWN
            else:
                return Color.RED
    else:
        if g <= 58.50:
            if b <= 34.00:
                return Color.GREEN
            else:
                return Color.BLUE
        else:
            if g <= 88.50:
                return Color.YELLOW
            else:
                return Color.WHITE


def s3_decision_tree(rgb_tuple):
    r, g, b = rgb_tuple
    if r <= 10.00:
        if g <= 8.00:
            return Color.BLACK
        else:
            if b <= 22.00:
                return Color.GREEN
            else:
                return Color.BLUE
    else:
        if g <= 26.50:
            if r <= 30.00:
                return Color.BROWN
            else:
                return Color.RED
        else:
            if g <= 52.00:
                return Color.YELLOW
            else:
                return Color.WHITE


def s4_decision_tree(rgb_tuple):
    r, g, b = rgb_tuple
    if r <= 12.50:
        if b <= 6.00:
            return Color.BLACK
        else:
            if g <= 22.50:
                return Color.BLUE
            else:
                return Color.GREEN
    else:
        if g <= 36.00:
            if r <= 33.00:
                return Color.BROWN
            else:
                return Color.RED
        else:
            if r <= 46.00:
                return Color.YELLOW
            else:
                return Color.WHITE


class DecisionColorSensor:
    """
    Encapsula o sensor de cor numa classe que parametriza
    a determinação de cores através de uma árvore de decisão
    """

    raw_sensor: ColorSensor

    def __init__(self, port, decision_tree) -> None:
        self.raw_sensor = ColorSensor(port)
        self.decision_tree = decision_tree

    def color(self):
        return self.decision_tree(self.raw_sensor.rgb())

    def reflection(self):
        return self.raw_sensor.reflection()

    def rgb(self):
        return self.raw_sensor.rgb()
