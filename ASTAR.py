#!/usr/bin/env python3
import requests
import json


def printMap(grid):
    for a in grid:
        for b in a:
            print("{} ".format(b))

def loadInMapData():
    response = requests.get('http://10.7.201.15:5140/map/')
    response_dict = json.loads(response.text)
    grid = response_dict['message']['map']['pgm']
    grid = grid[160:220]
    k = 0
    for g in grid:
        g = g[160:220]
        grid[k] = g
        k+=1
    i = 0
    for row in grid:
        row = list(map(lambda c: 0 if int(c) == 0 or int(c) == 205 else 1, row))
        grid[i] = row
        i+= 1
    print(grid)
    return grid

def search(grid, init, goal, cost, delta, heuristic):
    """
    :param grid: 2D matrix of the world, with the obstacles marked as '1', rest as '0'
    :param init: list of x and y co-ordinates of the robot's initial position
    :param goal: list of x and y co-ordinates of the robot's intended final position
    :param cost: cost of moving one position in the grid
    :param delta: list of all the possible movements
    :param heuristic: 2D matrix of same size as grid, giving the cost of reaching the goal from each cell
    :return: path: list of the cost of the minimum path, and the goal position
    :return: extended: 2D matrix of same size as grid, for each element, the count when it was expanded or -1 if
             the element was never expanded.
    """
    path = []
    val = 1

    visited = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    visited[init[0]][init[1]] = 1

    expand = [[-1 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    expand[init[0]][init[1]] = 0

    x = init[0]
    y = init[1]
    g = 0
    f = g + 0#heuristic[x][y]

    minList = [f, g, x, y]

    while minList[2:] != goal:
        for i in range(len(delta)):
            x2 = x + delta[i][0]
            y2 = y + delta[i][1]
            if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                if visited[x2][y2] == 0 and grid[x2][y2] == 0:
                    g2 = g + cost
                    f2 = g2 + 0#heuristic[x2][y2]
                    path.append([f2, g2, x2, y2])
                    visited[x2][y2] = 1

        if not path:
            return 'fail', expand

        del minList[:]
        minList = min(path)
        path.remove(minList)
        x = minList[2]
        y = minList[3]
        g = minList[1]
        expand[x][y] = val
        val += 1

    return minList, expand

if __name__ == '__main__':
    '''grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]]
    '''
    heuristic = [[9, 8, 7, 6, 5, 4],
                 [8, 7, 6, 5, 4, 3],
                 [7, 6, 5, 4, 3, 2],
                 [6, 5, 4, 3, 2, 1],
                 [5, 4, 3, 2, 1, 0]]

    init = [182, 182]
    cost = 1

    delta = [[-1, 0],  # go up
             [0, -1],  # go left
             [1, 0],  # go down
             [0, 1]]  # go right

    delta_name = ['^', '<', 'v', '>']
    grid = loadInMapData()
    goal = [len(grid) - 1, len(grid[0]) - 1]
    #path, expand = search(grid, init, goal, cost, delta, heuristic)

    #print(path)

    #print(expand)

