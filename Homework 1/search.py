from collections import defaultdict


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
        return [x, y] in targets

    def legalMove(self, x, y):
        if 0 <= x < self.w and 0 <= y < self.h:
            return True

    def shouldMove(self, x, y, x_n, y_n):
        if abs(self.terrainMap[y][x] - self.terrainMap[y_n][x_n]) <= maxElevation:
            return True
        else:
            return False

    def setPath(self, parent, x_n, y_n):

        path = list()
        path.append([x_n, y_n])

        goalX = x_n
        goalY = y_n

        while x_n != lx and y_n != ly:
            path.append(parent[y_n][x_n])
            x_n, y_n = parent[y_n][x_n]

        path.append([lx, ly])
        path.reverse()
        self.targetPathMap[tuple([goalX, goalY])] = path

    def printTargetPaths(self):
        for i in range(self.targetSites):
            for j in self.targetPathMap[tuple(targets[i])]:
                print(str(j[0]) + ',' + str(j[1]), end=' ')
            print()

    def BFS(self):
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


if __name__ == '__main__':
    algorithm = input().strip()
    w, h = map(int, input().strip().split(" "))
    lx, ly = map(int, input().strip().split(" "))
    maxElevation = int(input().strip())
    targetSites = int(input().strip())

    targets = []

    for i in range(targetSites):
        targets.append(list(map(int, input().strip().split(" "))))

    eM = []
    for i in range(h):
        eM.append(list(map(int, input().strip().split(" "))))

    p = PathFinder(eM, targets, lx, ly, maxElevation)
    p.BFS()
    p.printTargetPaths()
