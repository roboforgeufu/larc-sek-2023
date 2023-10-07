from math import sqrt

# 12 de largura, 9 de altura
city_map = [
    "PYOYBXOYSX E",
    "PX XYX XYX E",
    "PX  O   O  E",
    "PX XXX XYX E",
    "PYOYDYXXCX E",
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
    neighbours = [(x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)]
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

def set_path_routine(goal):

    initial_position = (8,10)
    path_positions_list = a_star(initial_position, goal, euclidian_distance)
    zero_size_elements = [(0,1),(0,3),(0,5),(0,7),(0,9),
                          (1,0),(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(1,7),(1,8),(1,9),(1,10),(1,11),
                          (2,1),(2,3),(2,5),(2,7),(2,9),
                          (3,0),(3,1),(3,2),(3,3),(3,4),(3,5),(3,6),(3,7),(3,8),(3,9),(3,10),(3,11),
                          (4,1),(4,3),(4,5),(4,7),(4,9),
                          (5,0),(5,1),(5,2),(5,3),(5,4),(5,5),(5,6),(5,7),(5,8),(5,9),(5,10),(5,11),
                          (6,1),(6,3),(6,5),(6,7),(6,9),
                          (7,0),(7,1),(7,2),(7,3),(7,4),(7,5),(7,6),(7,7),(7,8),(7,9),(7,10),(7,11),
                          (8,1),(8,3),(8,5),(8,7),(8,9)]
    
    path_with_curves = find_turns(path_positions_list)
    path_movements_list = ghost_busters(zero_size_elements, path_with_curves)

    i = 0
    for path in path_movements_list:
        if(isinstance(path, tuple)):
            path_movements_list[i] = 30
        i += 1

    return path_movements_list

def ghost_busters(ghosts, busters):
#busters: é a lista de caminho
#ghosts: é a lista de quadrados fantasmas
    i = 0
    while(i < len(busters)):
        for ghost in ghosts:
            if (busters[i] == ghost):
                busters[i] = 0
        i += 1
    return busters

def find_turns(path_list):
    find_turn_list = []
    for i in range(len(path_list)-1):
        element = (path_list[i+1][0]-path_list[i][0], path_list[i+1][1]-path_list[i][1])
        if element == (-1,0):
            find_turn_list.append("N")
        elif element == (1,0):
            find_turn_list.append("S")
        elif element == (0,1):
            find_turn_list.append("L")
        elif element == (0,-1):
            find_turn_list.append("O")
        i += 1

    for i in range(len(find_turn_list)-1):
        if (find_turn_list[i] != find_turn_list[i+1]):

            if find_turn_list[i] == "N":
                if find_turn_list[i+1] == "O":
                    path_list.insert(i+2, "curva_esquerda")
                    find_turn_list.insert(i+1, "curva_esquerda")
                else:
                    path_list.insert(i+2, "curva_direita")
                    find_turn_list.insert(i+1, "curva_direita")

            elif find_turn_list[i] == "S":
                if find_turn_list[i+1] == "L":
                    path_list.insert(i+2, "curva_esquerda")
                    find_turn_list.insert(i+1, "curva_esquerda")
                else:
                    path_list.insert(i+2, "curva_direita")
                    find_turn_list.insert(i+1, "curva_direita")

            elif find_turn_list[i] == "L":
                if find_turn_list[i+1] == "N":
                    path_list.insert(i+2, "curva_esquerda")
                    find_turn_list.insert(i+1, "curva_esquerda")
                else:
                    path_list.insert(i+2, "curva_direita")
                    find_turn_list.insert(i+1, "curva_direita")

            elif find_turn_list[i] == "O":
                if find_turn_list[i+1] == "S":
                    path_list.insert(i+2, "curva_esquerda")
                    find_turn_list.insert(i+1, "curva_esquerda")
                else:
                    path_list.insert(i+2, "curva_direita")
                    find_turn_list.insert(i+1, "curva_direita")

    return path_list

def path_to_movement():
    movement_list = set_path_routine((0,2))
    for movement in movement_list:
        if (movement == 30):
            robot.pid_walk(cm=30, vel=80)
        elif (movement == "curva_direita"):
            robot.pid_turn(90)
        elif (movement == "curva_esquerda"):
            robot.pid_turn(-90)

path_to_movement()
# set_path_routine((6,2))