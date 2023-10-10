from math import sqrt

from constants import ORIGIN_TUPLE
from robot import Robot
from utils import PIDValues
from domain.chess_tower import go_to_origin_routine

# 12 de largura, 9 de altura
city_map = [
    "PYOYBXOYSX E",
    "PX XYX XYX E",
    "PX  O   O  E",
    "PX XXX XYX E",
    "PYOYDYOXCX E",
    "PX XXX XYX E",
    "PX  O   O  E",
    "PX XYX XYX E",
    "PYOXMYOXLY E",
]


def reconstruct_path(came_from: dict, current: tuple[int, int]):
    total_path = [current]
    while current in came_from.keys():
        current = came_from[current]
        total_path.insert(0, current)
    return total_path


def get_neighbours(position: tuple[int, int]) -> list[tuple[int, int]]:
    x, y = position
    neighbours = [(x, y - 1), (x - 1, y), (x, y + 1), (x + 1, y)]
    return [n for n in neighbours if n[0] in range(9) and n[1] in range(12)]


def get_position_weight(position: tuple[int, int]):
    character = city_map[position[0]][position[1]]
    if character in "PBSDCML":
        return 10
    elif character == "Y":
        return 5
    elif character in "X":
        return 1000
    else:
        # " " e "O"
        return 1


def euclidian_distance(a_tuple, b_tuple):
    return sqrt((a_tuple[0] - b_tuple[0]) ** 2 + (a_tuple[1] - b_tuple[1]) ** 2)


def a_star(start: tuple[int, int], goal, heuristic):
    open_set = set([start])
    came_from = {}

    g_score = {(x, y): 1000 for x in range(9) for y in range(12)}
    g_score[start] = 0

    f_score = {(x, y): 1000 for x in range(9) for y in range(12)}
    f_score[start] = heuristic(start, goal)

    while open_set != set():
        current = sorted(open_set, key=lambda p: f_score[p])[0]
        if current == goal:
            return reconstruct_path(came_from, current)

        open_set.remove(current)
        for neighbour in get_neighbours(current):
            tentative_g_score = g_score[current] + get_position_weight(neighbour)
            if tentative_g_score < g_score[neighbour]:
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g_score
                f_score[neighbour] = tentative_g_score + heuristic(neighbour, goal)
                if neighbour not in open_set:
                    open_set.add(neighbour)
    return -1


def set_path_routine(goal, start):
    path_positions_list = a_star(start, goal, euclidian_distance)
    zero_size_elements = [
        (0, 1),
        (0, 3),
        (0, 5),
        (0, 7),
        (0, 9),
        (1, 0),
        (1, 1),
        (1, 2),
        (1, 3),
        (1, 4),
        (1, 5),
        (1, 6),
        (1, 7),
        (1, 8),
        (1, 9),
        (1, 10),
        (1, 11),
        (2, 1),
        (2, 3),
        (2, 5),
        (2, 7),
        (2, 9),
        (3, 0),
        (3, 1),
        (3, 2),
        (3, 3),
        (3, 4),
        (3, 5),
        (3, 6),
        (3, 7),
        (3, 8),
        (3, 9),
        (3, 10),
        (3, 11),
        (4, 1),
        (4, 3),
        (4, 5),
        (4, 7),
        (4, 9),
        (5, 0),
        (5, 1),
        (5, 2),
        (5, 3),
        (5, 4),
        (5, 5),
        (5, 6),
        (5, 7),
        (5, 8),
        (5, 9),
        (5, 10),
        (5, 11),
        (6, 1),
        (6, 3),
        (6, 5),
        (6, 7),
        (6, 9),
        (7, 0),
        (7, 1),
        (7, 2),
        (7, 3),
        (7, 4),
        (7, 5),
        (7, 6),
        (7, 7),
        (7, 8),
        (7, 9),
        (7, 10),
        (7, 11),
        (8, 1),
        (8, 3),
        (8, 5),
        (8, 7),
        (8, 9),
    ]

    path_with_curves = find_turns(path_positions_list)
    path_movements_list = ghost_busters(zero_size_elements, path_with_curves)

    i = 0
    for path in path_movements_list:
        if isinstance(path, tuple):
            path_movements_list[i] = 28
        i += 1

    path_movements_list.pop(0)

    if start == ORIGIN_TUPLE:
        first_left_turn_index = path_movements_list.index("curva_esquerda")
        path_movements_list.insert((first_left_turn_index + 1), "alinha_frente")
        i = 0
        for movement in path_movements_list:
            if movement == "curva_esquerda":
                break
            path_movements_list[i] = (-1) * movement
            i += 1
        path_movements_list.insert((first_left_turn_index + 2), (-5))
        path_movements_list.insert((first_left_turn_index + 3), "curva_esquerda")
        path_movements_list.insert((first_left_turn_index + 4), "curva_esquerda")

    if goal != ORIGIN_TUPLE:
        last_movement_index = len(path_movements_list) - 1
        path_movements_list.pop(last_movement_index)
        alignment_routine = [
            "alinha_frente",
            -5,
            "curva_esquerda",
            "curva_esquerda",
            "alinha_frente",
            -5,
        ]
        for movement in alignment_routine:
            path_movements_list.append(movement)
      
    if goal == ORIGIN_TUPLE:
        path_movements_list = path_movements_list[2:]
        i = (len(path_movements_list)-1)
        while True:
            if path_movements_list[i] == "curva_direita":
                break
            i -= 1
        path_movements_list.insert(i, "alinha_frente")
        path_movements_list.insert(i+1, (-5))

    return path_movements_list


def ghost_busters(ghosts, busters):
    # busters: é a lista de caminho
    # ghosts: é a lista de quadrados fantasmas
    i = 0
    while i < len(busters):
        for ghost in ghosts:
            if busters[i] == ghost:
                busters[i] = 0
        i += 1
    return busters


def find_turns(path_list):
    find_turn_list = []
    for i in range(len(path_list) - 1):
        element = (
            path_list[i + 1][0] - path_list[i][0],
            path_list[i + 1][1] - path_list[i][1],
        )
        if element == (-1, 0):
            find_turn_list.append("N")
        elif element == (1, 0):
            find_turn_list.append("S")
        elif element == (0, 1):
            find_turn_list.append("L")
        elif element == (0, -1):
            find_turn_list.append("O")
        i += 1

    i = 0
    while i < (len(find_turn_list) - 1):
        if find_turn_list[i] != find_turn_list[i + 1]:
            if find_turn_list[i] == "N":
                if find_turn_list[i + 1] == "O":
                    path_list.insert(i + 2, "curva_esquerda")
                    find_turn_list.insert(i + 1, "curva_esquerda")
                else:
                    path_list.insert(i + 2, "curva_direita")
                    find_turn_list.insert(i + 1, "curva_direita")

            elif find_turn_list[i] == "S":
                if find_turn_list[i + 1] == "L":
                    path_list.insert(i + 2, "curva_esquerda")
                    find_turn_list.insert(i + 1, "curva_esquerda")
                else:
                    path_list.insert(i + 2, "curva_direita")
                    find_turn_list.insert(i + 1, "curva_direita")

            elif find_turn_list[i] == "L":
                if find_turn_list[i + 1] == "N":
                    path_list.insert(i + 2, "curva_esquerda")
                    find_turn_list.insert(i + 1, "curva_esquerda")
                else:
                    path_list.insert(i + 2, "curva_direita")
                    find_turn_list.insert(i + 1, "curva_direita")

            elif find_turn_list[i] == "O":
                if find_turn_list[i + 1] == "S":
                    path_list.insert(i + 2, "curva_esquerda")
                    find_turn_list.insert(i + 1, "curva_esquerda")
                else:
                    path_list.insert(i + 2, "curva_direita")
                    find_turn_list.insert(i + 1, "curva_direita")
        i += 1

    return path_list


def path_to_movement(robot: Robot, goal, start=None):
    # as direcoes sao invertidas pois o robo anda de ré
    if start is None:
        start = ORIGIN_TUPLE
    movement_list = set_path_routine(goal, start)
    print(movement_list)
    for movement in movement_list:
        if isinstance(movement, int) and movement != 0:
            if movement > 0:
                robot.pid_walk(cm=movement, speed=-80)
            else:
                robot.pid_walk(cm=movement, speed=80)
        elif movement == "curva_direita":
            robot.pid_turn(90)
        elif movement == "curva_esquerda":
            robot.pid_turn(-90)
        elif movement == "alinha_atras":
            robot.forward_while_same_reflection(
                reflection_diff=22,
                left_reflection_function=lambda: robot.color_fl.rgb()[2],
                right_reflection_function=lambda: robot.color_fr.rgb()[2],
            )
            robot.pid_walk(cm=2, speed=-30)
            robot.pid_align()
        elif movement == "alinha_frente":
            robot.forward_while_same_reflection(
                speed_l=-30,
                speed_r=-30,
                reflection_diff=22,
                left_reflection_function=lambda: robot.color_bl.rgb()[2],
                right_reflection_function=lambda: robot.color_br.rgb()[2],
            )
            robot.pid_walk(cm=2, speed=30)
            robot.pid_align(
                sensor_function_l=lambda: robot.color_bl.rgb()[2],
                sensor_function_r=lambda: robot.color_br.rgb()[2],
                direction_sign=-1,
            )

    if goal == ORIGIN_TUPLE:
        # no quadrado da origem apontando as costas para a linha vermelha
        robot.forward_while_same_reflection(
            speed_l=-30,
            speed_r=-30,
            reflection_diff=22,
            left_reflection_function=lambda: robot.color_bl.rgb()[2],
            right_reflection_function=lambda: robot.color_br.rgb()[2],
        )
        robot.pid_walk(cm=2, speed=30)
        robot.pid_align(
            sensor_function_l=lambda: robot.color_bl.rgb()[2],
            sensor_function_r=lambda: robot.color_br.rgb()[2],
            direction_sign=-1,
        )
        # alinha no azul de ré
        robot.pid_walk(cm=3, speed=30)
        robot.pid_turn(-90)
        robot.forward_while_same_reflection(
            speed_l=-30,
            speed_r=-30,
            reflection_diff=22,
            left_reflection_function=lambda: robot.color_bl.rgb()[2],
            right_reflection_function=lambda: robot.color_br.rgb()[2],
        )
        robot.pid_walk(cm=2, speed=30)
        robot.pid_align(
            sensor_function_l=lambda: robot.color_bl.rgb()[2],
            sensor_function_r=lambda: robot.color_br.rgb()[2],
            direction_sign=-1,
        )
        # alinha no vermelho de ré
        # robot.pid_walk(cm=3, speed=30)
        robot.pid_turn(90)
        robot.forward_while_same_reflection(
            speed_l=-30,
            speed_r=-30,
            reflection_diff=22,
            left_reflection_function=lambda: robot.color_bl.rgb()[2],
            right_reflection_function=lambda: robot.color_br.rgb()[2],
        )
        robot.pid_walk(cm=2, speed=30)
        robot.pid_align(
            sensor_function_l=lambda: robot.color_bl.rgb()[2],
            sensor_function_r=lambda: robot.color_br.rgb()[2],
            direction_sign=-1,
        )


def decide_passenger_goal(passenger_info, park_flag):
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

    return goal, park_flag

# path_to_movement((4,8))
# path_to_movement(ORIGIN_TUPLE, start=(4,8))