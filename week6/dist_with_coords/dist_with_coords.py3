#!/usr/bin/python3

import sys
import heapq
import math
from collections import defaultdict

class AStar:
    def __init__(self, n, adj, cost, x, y):
        # See the explanations of these fields in the starter for friend_suggestion        
        self.n = n;
        self.adj = adj
        self.cost = cost
        self.inf = n*10**6
        self.d = defaultdict(lambda: self.inf)
        self.p = {}
        self.visited = [False]*n
        self.workset = []
        # Coordinates of the nodes
        self.x = x
        self.y = y

    # See the explanation of this method in the starter for friend_suggestion
    def clear(self):
        for v in self.workset:
            self.visited[v] = False;
        self.d.clear()
        self.p.clear()
        del self.workset[0:len(self.workset)]

    # See the explanation of this method in the starter for friend_suggestion
    def visit(self, q, v, dist, measure):

        if self.d[v] > dist:
            self.d[v] = dist
            heapq.heappush(q, (self.d[v] + measure, v))

    def potential(self, v, t):
        try:
            return self.p[v]
        except:
            self.p[v] = ((self.x[v] - self.x[t])**2 + (self.y[v] - self.y[t])**2)**0.5
            return self.p[v]

    def process(self, u, t, adj, cost, q):

        for i, v in enumerate(adj[u]):
            if not self.visited[v] == True:
                self.visit(q, v, self.d[u] + cost[u][i], self.potential(v, t))    
        self.workset.append(u)

    # Returns the distance from s to t in the graph
    def query(self, s, t):
        self.clear()
        q = []

        self.visit(q, s, 0, self.potential(s, t))

        if s == t:
            return 0

        while q:
            u = heapq.heappop(q)[1]

            if u == t:
                return self.d[t] if self.d[t] < self.inf else -1

            if not self.visited[u] == True:

                self.process(u, t, adj, cost, q)
                self.visited[u] = True
                
        return -1

def readl():
    return map(int, sys.stdin.readline().split())

if __name__ == '__main__':
    n,m = readl()
    x = [0 for _ in range(n)]
    y = [0 for _ in range(n)]
    adj = [[] for _ in range(n)]
    cost = [[] for _ in range(n)]
    for i in range(n):
        a, b = readl()
        x[i] = a
        y[i] = b
    for e in range(m):
        u,v,c = readl()
        adj[u-1].append(v-1)
        cost[u-1].append(c)
    t, = readl()
    astar = AStar(n, adj, cost, x, y)
    for i in range(t):
        s, t = readl()
        print(astar.query(s-1, t-1))
