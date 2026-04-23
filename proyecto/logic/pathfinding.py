import heapq
import math

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def vecinos(node, grid):
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    res = []

    for d in dirs:
        ni = node[0] + d[0]
        nj = node[1] + d[1]

        if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1]:
            if grid[ni, nj] == "white":  # evita obstáculos
                res.append((ni, nj))

    return res


def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for v in vecinos(current, grid):
            tentative = g_score[current] + 1

            if v not in g_score or tentative < g_score[v]:
                came_from[v] = current
                g_score[v] = tentative
                f = tentative + heuristic(v, goal)
                heapq.heappush(open_set, (f, v))

    return None