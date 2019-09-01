import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from collections import defaultdict


def reconstruct_path(came_from, current, pathMap):
    final_path = [current]
    while current in came_from:
        current = came_from[current]
        final_path.append(current)
    final_path = final_path[::-1]
    for x, y in final_path:
        pathMap[x][y] = 150
        plt.subplot(2, 2, 3)
        plt.xticks([]), plt.yticks([])
        plt.imshow(pathMap)
        plt.title('A* final path')
        plt.pause(0.1)
    return final_path


def heauristic(cell, goal):
    x1, y1 = cell
    x2, y2 = goal

    dist = ((x2-x1)**2 + (y2-y1)**2)**0.5
    return dist


def A_star(graph, start, goal, pathMap):
    pathMap[start[0]][start[1]] = 25

    closed_set = []  # nodes already evaluated

    open_set = [start]  # nodes discovered but not yet evaluated

    came_from = {}  # most efficient path to reach from

    gscore = {}  # cost to get to that node from start

    for key in graph:
        gscore[key] = 100  # intialize cost for every node to inf

    gscore[start] = 0

    fscore = {}  # cost to get to goal from start node via that node

    for key in graph:
        fscore[key] = 100

    fscore[start] = heauristic(start, goal)  # cost for start is only h(x)

    while open_set:
        min_val = 1000  # find node in openset with lowest fscore value
        for node in open_set:
            if fscore[node] < min_val:
                min_val = fscore[node]
                min_node = node

        current = min_node  # set that node to current
        if current == goal:
            return reconstruct_path(came_from, current, pathMap)
        open_set.remove(current)  # remove node from set to be evaluated and
        closed_set.append(current)  # add it to set of evaluated nodes

        for neighbor in graph[current]:  # check neighbors of current node
            if neighbor in closed_set:  # ignore neighbor node if its already evaluated
                continue
            if neighbor not in open_set:  # else add it to set of nodes to be evaluated
                open_set.append(neighbor)

            # dist from start to neighbor through current
            tentative_gscore = gscore[current] + 1

            # not a better path to reach neighbor
            if tentative_gscore >= gscore[neighbor]:
                continue
            came_from[neighbor] = current  # record the best path untill now
            gscore[neighbor] = tentative_gscore
            fscore[neighbor] = gscore[neighbor] + heauristic(neighbor, goal)
            pathMap[neighbor[0]][neighbor[1]] = 25

        img = pathMap                 # Display path while building
        plt.subplot(2, 2, 2)
        plt.xticks([]), plt.yticks([])
        plt.imshow(img)
        plt.title('A* search')
        plt.pause(0.1)
    return False


def mat2graph(mat):
    rows = len(mat)
    cols = len(mat[0])
    graph = defaultdict(list)
    for x in range(rows):
        for y in range(cols):
            if mat[x][y] == True:
                for dx, dy in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
                    if 0 <= x+dx < rows and 0 <= y+dy < cols and mat[x+dx][y+dy] == True:
                        graph[(x, y)].append((x+dx, y+dy))
    return graph


if __name__ == "__main__":

    from maze_gen_prims import PrimsMaze
    # size = 11
    size = int(input("Enter size of maze/graph: "))
    start = (0, 0)                        # start <= (size, size)
    destination = (size, size)

    print("Generating random maze...")
    obj = PrimsMaze(size, False)
    mat = obj.create_maze(start).tolist()

    # plt.figure(figsize=(15, 10))
    # plt.subplots()
    f, axs = plt.subplots(2,2,figsize=(15,15))

    plt.subplot(2, 2, 1)
    plt.title('Maze')
    plt.imshow(mat, interpolation='nearest')
    plt.xticks([]), plt.yticks([])

    graph = mat2graph(mat)

    start = (0, 0)
    destination = (size-1, size-1)

    pathMap = mat
    shortest_route = A_star(graph, start, destination, pathMap)
    print(shortest_route)
    for x, y in shortest_route:
        mat[x][y] = -150

    plt.subplot(2, 2, 2)
    plt.imshow(mat)
    plt.show()
