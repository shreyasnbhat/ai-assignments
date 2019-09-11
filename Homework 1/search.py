from collections import defaultdict
import math
import heapq as hq
import sys


# A Basic Priority Queue implementation using heapq
class PriorityQueue:
    pq = []
    fastSearch = False
    stateMap = defaultdict(list)
    counterToState = {}

    def __init__(self, fastSearch=False):
        hq.heapify(self.pq)
        self.fastSearch = fastSearch

    def push(self, element):
        hq.heappush(self.pq, element)
        self.counterToState[(element[0], element[2][0], element[2][1])] = element[1]
        if self.fastSearch:
            self.stateMap[element[2]].append(element[0])

    def pop(self):
        element = self.top()
        if self.fastSearch:
            self.stateMap[element[2]].remove(element[0])
            if len(self.stateMap[element[2]]) == 0:
                del self.stateMap[element[2]]

        hq.heappop(self.pq)
        del self.counterToState[(element[0], element[2][0], element[2][1])]

    # Element format is (pathcost, x, y)
    def delete(self, element):
        counter = self.counterToState[element]
        self.pq.remove((element[0], counter, element[1], element[2]))
        del self.counterToState[element]
        hq.heapify(self.pq)

        if self.fastSearch:
            self.stateMap[element[1]].remove(element[0])

    def top(self):
        return self.pq[0]

    def size(self):
        return len(self.pq)

    def exist(self, element):
        if len(self.stateMap[element]) == 0:
            del self.stateMap[element]
            return False, -1

        existence = element in self.stateMap
        if existence:
            idx = self.stateMap[element].index(min(self.stateMap[element]))
            return existence, self.stateMap[element][idx]
        else:
            return existence, -1

    def printPQ(self):
        copied = [i for i in self.pq]
        print([hq.heappop(copied) for i in range(len(self.pq))])


class PathFinder:
    terrainMap = None
    targets = []
    targetIdxMap = {}
    targetPathMap = {}
    targetSites = 0
    maxElevation = None
    w = 0
    h = 0
    lx = 0
    ly = 0

    neighbours = [[-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]

    def __init__(self, terrainMap, targets, lx, ly, maxElevation):
        self.terrainMap = terrainMap
        self.targets = targets
        self.targetSites = len(self.targets)

        for i in range(self.targetSites):
            self.targetIdxMap[tuple(targets[i])] = i

        self.h = len(self.terrainMap)
        self.w = len(self.terrainMap[0])
        self.lx = lx
        self.ly = ly
        self.maxElevation = maxElevation

    def legalMove(self, x, y):
        if 0 <= x < self.w and 0 <= y < self.h:
            return True

    def shouldMove(self, x, y, x_n, y_n):
        if abs(self.terrainMap[y][x] - self.terrainMap[y_n][x_n]) <= self.maxElevation:
            return True
        else:
            return False

    def cost2D(self, x, y, x_n, y_n):
        return math.floor(math.sqrt(pow(abs(x - x_n), 2) + pow(abs(y - y_n), 2)) * 10)

    def cost3D(self, x, y, x_n, y_n):
        return self.cost2D(x, y, x_n, y_n) + abs(self.terrainMap[y][x] - self.terrainMap[y_n][x_n])

    # Heuristic till target
    def heuristic(self, x, y, x_t, y_t):
        X = math.sqrt(pow(abs(x - x_t), 2) + pow(abs(y - y_t), 2)) * 10
        Y = abs(self.terrainMap[y_t][x_t] - self.terrainMap[y][x])
        return math.floor(math.sqrt(pow(X, 2) + pow(Y, 2)))

    def setPath(self, parent, x_n, y_n):

        path = list()

        goalX = x_n
        goalY = y_n

        while not (x_n == self.lx and y_n == self.ly):
            path.append([x_n, y_n])
            x_n, y_n = parent[y_n][x_n]

        path.append([self.lx, self.ly])
        path.reverse()
        self.targetPathMap[tuple([goalX, goalY])] = path

    def printTargetPaths(self):

        f = open("output.txt", "w+")

        for i in range(self.targetSites):
            if tuple(self.targets[i]) in self.targetPathMap:
                size = len(self.targetPathMap[tuple(self.targets[i])])
                for idx, j in enumerate(self.targetPathMap[tuple(self.targets[i])]):
                    temp = str(j[0]) + ',' + str(j[1])
                    if idx < size - 1:
                        f.write(temp + ' ')
                    else:
                        f.write(temp + '\n')
                    print(temp, end=' ')
            else:
                f.write('FAIL\n')
                print('FAIL', end="")
            print()

        f.close()
        self.targetPathMap = {}

    def BreadthFirst(self, target):
        visited = [[False for i in range(self.w)] for j in range(self.h)]
        parent = [[[0, 0] for i in range(self.w)] for j in range(self.h)]

        q = list()
        q.append([self.lx, self.ly])

        while len(q) is not 0:
            top = q[0]
            q.pop(0)

            for i in self.neighbours:
                x, y = top
                x_n, y_n = x + i[0], y + i[1]

                if not visited[y_n][x_n] and self.legalMove(x_n, y_n) and self.shouldMove(x, y, x_n, y_n):
                    visited[y_n][x_n] = True
                    parent[y_n][x_n] = [x, y]
                    if x_n == target[0] and y_n == target[1]:
                        self.setPath(parent, x_n, y_n)
                        if len(self.targetPathMap) == self.targetSites:
                            return
                    else:
                        q.append([x_n, y_n])

    def findInList(self, l, x, y):

        mini = 10000000
        flag = False

        for i, j in l:
            if i < mini and j[0] == x and j[1] == y:
                mini = i
                flag = True

        if flag:
            return True, mini
        else:
            return False, -1

    def UniformCost(self, target):

        open = PriorityQueue(True)
        closed = []
        parent = [[[0, 0] for i in range(self.w)] for j in range(self.h)]

        globalCounter = 0

        # Push initial state
        open.push([0, globalCounter, (self.lx, self.ly)])
        globalCounter += 1

        while True:
            if open.size() == 0:
                return

            current = open.top()
            open.pop()

            x, y = current[2][0], current[2][1]

            if x == target[0] and y == target[1]:
                self.setPath(parent, x, y)
                if len(self.targetPathMap) == self.targetSites:
                    break

            for i in self.neighbours:
                x_n, y_n = x + i[0], y + i[1]
                if self.legalMove(x_n, y_n) and self.shouldMove(x, y, x_n, y_n):
                    openExistence, openCost = open.exist((x_n, y_n))
                    closedExistence, closedCost = self.findInList(closed, x_n, y_n)

                    pathCost = current[0] + self.cost2D(x, y, x_n, y_n)

                    if not openExistence and not closedExistence:
                        open.push([pathCost, globalCounter, (x_n, y_n)])
                        parent[y_n][x_n] = [x, y]
                        globalCounter += 1
                    elif openExistence:
                        if pathCost < openCost:
                            open.delete((openCost, (x_n, y_n)))
                            open.push([pathCost, globalCounter, (x_n, y_n)])
                            parent[y_n][x_n] = [x, y]
                            globalCounter += 1
                    elif closedExistence:
                        if pathCost < closedCost:
                            closed.remove((closedCost, (x_n, y_n)))
                            open.push([pathCost, globalCounter, (x_n, y_n)])
                            parent[y_n][x_n] = [x, y]
                            globalCounter += 1

            closed.append((current[0], current[2]))

    def AStar(self, target):
        open = PriorityQueue(True)
        closed = []
        parent = [[[0, 0] for i in range(self.w)] for j in range(self.h)]

        globalCounter = 0

        # f = g + h
        f = {}
        g = defaultdict(lambda: sys.maxsize)

        g[(self.lx, self.ly)] = 0
        f[(self.lx, self.ly)] = self.heuristic(self.lx, self.ly, target[0], target[1])

        # Push initial state
        open.push([0, globalCounter, (self.lx, self.ly)])
        globalCounter += 1

        while open.size() is not 0:
            # open.printPQ()

            current = open.top()
            open.pop()

            closed.append((current[0], current[2]))

            x, y = current[2][0], current[2][1]

            if x == target[0] and y == target[1]:
                self.setPath(parent, x, y)
                break

            for i in self.neighbours:

                x_n, y_n = x + i[0], y + i[1]
                if self.legalMove(x_n, y_n) and self.shouldMove(x, y, x_n, y_n):
                    openExistence, openCost = open.exist((x_n, y_n))
                    closedExistence, closedCost = self.findInList(closed, x_n, y_n)

                    pathCost = g[(x, y)] + self.cost3D(x, y, x_n, y_n)
                    heuristicCost = self.heuristic(x_n, y_n, target[0], target[1])
                    estimatedCost = pathCost + heuristicCost

                    if closedExistence:
                        continue

                    if pathCost < g[(x_n, y_n)]:
                        parent[y_n][x_n] = [x, y]
                        g[(x_n, y_n)] = pathCost
                        f[(x_n, y_n)] = estimatedCost
                        if not openExistence:
                            open.push([f[(x_n, y_n)], globalCounter, (x_n, y_n)])
                            globalCounter += 1

    def BreadthFirstComplete(self):
        for i in targets:
            self.BreadthFirst(i)

    def UniformCostComplete(self):
        for i in targets:
            self.UniformCost(i)

    def AStarComplete(self):
        for i in self.targets:
            self.AStar(i)


if __name__ == '__main__':

    f = open("input3.txt", "r")

    lines = f.readlines()

    algorithm = lines[0].strip()
    w, h = map(int, lines[1].strip().split(" "))
    lx, ly = map(int, lines[2].strip().split(" "))
    maxElevation = int(lines[3].strip())
    targetSites = int(lines[4].strip())

    targets = []

    for i in range(targetSites):
        targets.append(list(map(int, lines[5 + i].strip().split(" "))))

    eM = []
    for i in range(h):
        eM.append(list(map(int, lines[5 + targetSites + i].strip().split(" "))))

    p = PathFinder(eM, targets, lx, ly, maxElevation)

    if algorithm == 'BFS':
        p.BreadthFirstComplete()
    elif algorithm == 'UCS':
        p.UniformCostComplete()
    else:
        p.AStarComplete()

    p.printTargetPaths()
