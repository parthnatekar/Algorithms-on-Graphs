#!/usr/bin/python3

import sys
import queue
from collections import defaultdict
import heapq
import time

class BiDij:
    def __init__(self, n):
        self.n = n;                             # Number of nodes
        self.inf = n*10**6                      # All distances in the graph are smaller
        self.d = [defaultdict(lambda: self.inf), defaultdict(lambda: self.inf)]   # Initialize distances for forward and backward searches
        self.visited = [False]*n                # visited[v] == True iff v was visited by forward or backward search
        self.workset = []                       # All the nodes visited by forward or backward search

    def clear(self):
        """Reinitialize the data structures for the next query after the previous query."""
        for v in self.workset:
            self.visited[v] = False
        self.d[0].clear()
        self.d[1].clear()
        del self.workset[0:len(self.workset)]

    def visit(self, q, side, v, dist):
        """Try to relax the distance to node v from direction side by value dist."""
        if self.d[side][v] > dist:
            self.d[side][v] = dist
            heapq.heappush(q[side], (self.d[side][v], v))

    def process(self, u, adj, cost, side, q):

        for i, v in enumerate(adj[side][u]):
            self.visit(q, side, v, self.d[side][u] + cost[side][u][i])              
        self.workset.append(u)
            
    def query(self, adj, cost, s, t):
        self.clear()
        q = [[], []]

        self.visit(q, 0, s, 0)
        self.visit(q, 1, t, 0)

        if s == t:
            return 0

        while q[0] and q[1]:

            u = heapq.heappop(q[0])[1]
            self.process(u, adj, cost, 0, q)
            
            if self.visited[u] == True:
                return self.ShortestPath() 
            self.visited[u] = True

            u = heapq.heappop(q[1])[1]
            self.process(u, adj, cost, 1, q)

            if self.visited[u] == True:
                return self.ShortestPath() 
            self.visited[u] = True

        return -1

    def ShortestPath(self):
        distance = self.inf
        u_best = None

        for u in self.workset:
            if self.d[0][u] + self.d[1][u] < distance:
                u_best = u
                distance = self.d[0][u] + self.d[1][u]

        return distance if distance < self.inf else -1
        
        # if u_best == None:
        #     return (-1, None)

        # path = []
        # last = u_best

        # while last != s:
        #     path.append(last)
        #     last = self.prev[0][last]

        # path = path[::-1]
        # last = u_best

        # while last != t:
        #     last = self.prev[1][last]
        #     path.append(last)

        # return (distance, path)


def readl():
    return map(int, sys.stdin.readline().split())


if __name__ == '__main__':
    n,m = readl()
    adj = [[[] for _ in range(n)], [[] for _ in range(n)]]
    cost = [[[] for _ in range(n)], [[] for _ in range(n)]]
    for e in range(m):
        u,v,c = readl()
        adj[0][u-1].append(v-1)
        cost[0][u-1].append(c)
        adj[1][v-1].append(u-1)
        cost[1][v-1].append(c)
    k, = readl()
    bidij = BiDij(n)
    start = time.time()
    for i in range(k):
        s, t = readl()
        print(bidij.query(adj, cost, s-1, t-1))

    end = time.time()
        
