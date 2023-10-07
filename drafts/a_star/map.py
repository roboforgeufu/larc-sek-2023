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
