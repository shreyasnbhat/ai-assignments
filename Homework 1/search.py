from collections import defaultdict
import math
import heapq as hq


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

    def goalTest(self, x, y):
        return [x, y] in self.targets

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

    def setPath(self, parent, x_n, y_n):

        path = list()
        path.append([x_n, y_n])

        goalX = x_n
        goalY = y_n

        while x_n != self.lx and y_n != self.ly:
            path.append(parent[y_n][x_n])
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

    def BreadthFirst(self):
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
                    if self.goalTest(x_n, y_n):
                        self.setPath(parent, x_n, y_n)
                        if len(self.targetPathMap) == self.targetSites:
                            return
                    else:
                        q.append([x_n, y_n])

    def findInList(self, l, x, y):

        mini = 10000000
        flag = False

        for i, k, j in l:
            if i < mini and j[0] == x and j[1] == y:
                mini = i
                flag = True

        if flag:
            return True, mini
        else:
            return False, -1

    def UniformCost(self):

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

            if self.goalTest(x, y):
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

            closed.append(current)

    def AStar(self):
        pass


if __name__ == '__main__':

    f = open("input.txt", "r")

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
        p.BreadthFirst()
    elif algorithm == 'UCS':
        p.UniformCost()
    else:
        p.AStar()

    p.printTargetPaths()
