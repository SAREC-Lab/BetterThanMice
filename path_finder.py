#!/usr/bin/env python3
import requests
import json
from collections import deque

RUN_WITH_LOCAL=True

def findFirstFreeSpace(grid):
    ind = 0
    for row in grid:
        if 0 in row:
            return (ind,row.index(0))
        ind += 1

def findLastFreeSpace(grid):
    ind = 0
    for row in grid[::-1]:
        if 0 in row:
            return (ind,len(row) - row[::-1].index(0))
        ind += 1

def outputMap(grid, filename):
    with open(filename, 'w') as f:
        for row in grid:
            row_as_str = [ str(i) for i in row ]
            f.write(''.join(row_as_str))
            f.write('\n')

def saveMapWithPath(grid, path, filename):
    for point in path:
        col, row = point
        grid[row][col] = 7
    outputMap(grid, filename)

def readInMapLocal():
    grid = []
    with open('map.txt', 'r') as f:
            Lines = f.readlines()
            for line in Lines:
                line = [ int(s) for s in [ char for char in line.strip()]]
                grid.append(line)
    return grid

def scaleMap(grid):
    while 0 not in grid[0] and 0 not in grid[1]:
        grid = grid[1:]
    while 0 not in grid[-1] and 0 not in grid[-2]:
        grid = grid[0:len(grid)-1]
    leftMostFreeCol  = 999
    rightMostFreeCol = -1
    for row in grid:
        if 0 in row:
            leftMostFreeCol = min(row.index(0), leftMostFreeCol)
            rightMostFreeCol = max(len(row)-row[::-1].index(0), rightMostFreeCol)
    rowNum = 0
    print("{} {}".format(leftMostFreeCol, rightMostFreeCol))
    for row in grid:
        row = row[leftMostFreeCol-1:rightMostFreeCol+1]
        grid[rowNum] = row
        rowNum += 1

    return grid


def saveMapFromRequest():
    grid = requestMapData()
    outputMap(grid, 'map.txt') 

def requestMapData():
    response = requests.get('http://10.7.201.15:5140/map/')
    response_dict = json.loads(response.text)
    grid = response_dict['message']['map']['pgm']
    i = 0
    for row in grid:
        row = list(map(lambda c: 1 if int(c) == 0 or int(c) == 205 else 0, row))
        grid[i] = row
        i+= 1
    return grid

def shortest_path(grid, start_pt, end_pt):
    ''' grid Properties'''
    '''BFS'''
    visited = {end_pt: None}
    queue = [end_pt]
    while queue:
        current = queue.pop(0)
        if current == start_pt:
            shortest_path = []
            while current:
                shortest_path.append(current)
                current = visited[current]
            return shortest_path
        adj_points = []
        '''FIND ADJACENT POINTS'''
        current_col, current_row = current
        #UP
        if current_row > 0:
            if grid[current_row - 1][current_col] == 0:
                adj_points.append((current_col, current_row - 1))
        #RIGHT
        if current_col < (len(grid[0])-1):
            if grid[current_row][current_col + 1] == 0: ## There was an error here!
                adj_points.append((current_col + 1, current_row))
        #DOWN
        if current_row < (len(grid) - 1):
            if grid[current_row + 1][current_col] == 0:
                adj_points.append((current_col, current_row + 1))
        #LEFT
        if current_col > 0:
            if grid[current_row][current_col - 1] == 0:
                adj_points.append((current_col - 1, current_row))

        '''LOOP THROUGH ADJACENT PTS'''
        for point in adj_points:
            if point not in visited:
                visited[point] = current
                queue.append(point)
 

if __name__ == '__main__':
    #saveMapFromRequest()
    if RUN_WITH_LOCAL:
        grid = readInMapLocal()
    else:
        grid = requestMapData()
    grid = scaleMap(grid)
    #outputMap(grid, 'maze.txt')
    path = shortest_path(grid,(4,2),(3,19))
    saveMapWithPath(grid, path, 'path.txt')
    print(grid)
