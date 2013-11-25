
from heapdict import heapdict

class GridState(object):

    MAX_SIZE = 20

    def __init__(self,x,y):
        self.x = x
        self.y = y

    def isLegal(self):
        x_legal = (self.x < GridState.MAX_SIZE) and (self.x >= 0)
        y_legal = (self.y < GridState.MAX_SIZE) and (self.y >= 0)
        return x_legal and y_legal

    def left(self):
        return GridState(self.x - 1,self.y)

    def right(self):
        return GridState(self.x + 1,self.y)

    def up(self):
        return GridState(self.x,self.y + 1)

    def down(self):
        return GridState(self.x,self.y -1)

    def neighbours(self):
        n = [self.left(),self.right(),self.up(),self.down()]
        return [i for i in n if i.isLegal()]

    def __str__(self):
        return '({x},{y})'.format(x = self.x, y = self.y)

    def __eq__(self,other):
        return self.x == other.x and self.y == other.y

    def heuristic(self,other):
        dx = self.x - other.x
        dy = self.y - other.y
        return abs(dx) + abs(dy)

    def edgeLength(self,other):
        return 1

def aStar(start,goal):
    visited = set()
    parents = {}
    queue = heapdict()
    g_score = {}
    f_score = {}

    g_score[start] = 0
    f_score[start] = g_score[start] + start.heuristic(goal)
    queue[start] = f_score[start]

    while queue:
        current = queue.popitem()[0] 
        visited.add(current)

        if current == goal:
            res = []
            curr = current
            while parents.get(curr):
                res.append(curr)
                curr = parents[curr]

            res.reverse()
            return res

        for neigh in current.neighbours():
            tentative_g_score = g_score[current] + current.edgeLength(neigh)
            tentative_f_score = tentative_g_score + neigh.heuristic(goal)
            if (neigh in visited) and (tentative_f_score >= f_score[neigh]):
                continue

            if (not (neigh in visited)) or (tentative_f_score < f_score[neigh]):
                parents[neigh] = current
                g_score[neigh] = tentative_g_score
                f_score[neigh] = tentative_f_score
                queue[neigh] = f_score[neigh]

    return None

