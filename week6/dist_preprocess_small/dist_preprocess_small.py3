#!/usr/bin/python3


import sys
import queue
from collections import defaultdict
import heapq

# Maximum allowed edge length
maxlen = 2 * 10**6


class DistPreprocessSmall:
    def __init__(self, n, adj, cost):
        # See description of these parameters in the starter for friend_suggestion
        self.n = n
        self.INFINITY = n * maxlen
        self.adj = adj
        self.cost = cost
        self.bidistance = [defaultdict(lambda: self.INFINITY), defaultdict(lambda: self.INFINITY)]
        self.visited = [False] * n
        self.workset = []
        self.q = []
        # Levels of nodes for node ordering heuristics
        self.level = [0] * n
        # Positions of nodes in the node ordering
        self.rank = [0] * n
        self.neighbors = [0] * n

    def mark_visited(self, x):
        if not self.visited[x]:
            self.visited[x] = True
            self.workset.append(x)

    def add_arc(self, u, v, c):
        def update(adj, cost, u, v, c):
            for i in range(len(adj[u])):
                if adj[u][i] == v:
                    cost[u][i] = min(cost[u][i], c)
                    return
            adj[u].append(v)
            cost[u].append(c)

        update(self.adj[0], self.cost[0], u, v, c)
        update(self.adj[1], self.cost[1], v, u, c)

    # Makes shortcuts for contracting node v
    def shortcut(self, v):
        # Implement this method yourself
        shortcut_count = 0
        shortcut_cover = 0
        neighbors = self.neighbors[v]
        level = self.level[v]
        shortcuts = []
        
        maxOutgoing = max(self.adj[0][v]) if len(self.adj[0][v]) else 0
        maxIncoming = max(self.adj[1][v]) if len(self.adj[1][v]) else 0
        for i, s in enumerate(self.adj[1][v]):
            if self.rank[s] < self.rank[v]:
                continue
            dist = self.dijkstra(s, v, maxIncoming + maxOutgoing)
            for j, t in enumerate(self.adj[0][v]):
                if self.rank[t] < self.rank[v]:
                    continue
                if self.cost[1][v][i] + self.cost[0][v][j] < dist[t]:
                    shortcut_count += 1
                    shortcut_cover += 2
                    shortcuts.append((s, t, self.cost[1][v][i] + self.cost[0][v][j]))

        # Compute correctly the values for the above heuristics before computing the node importance
        importance = (shortcut_count - len(self.adj[0][v]) - len(self.adj[1][v])) + neighbors + shortcut_cover + level
        return importance, shortcuts

    def recalibrateNeighbours(self, v):
        for i, s in enumerate(self.adj[1][v]):
                self.neighbors[s] += 1
                self.level[s] = max(self.level[s], self.level[v] + 1)
        for i, t in enumerate(self.adj[0][v]):
                self.neighbors[t] += 1
                self.level[t] = max(self.level[t], self.level[v] + 1)


    def preImportance(self):
        for v in range(n):
            edge_diff = (len(self.adj[0][v]) * len(self.adj[1][v])  - len(self.adj[0][v]) - len(self.adj[1][v]))
            neighbors = len(self.adj[0][v]) + len(self.adj[1][v])
            shortcut_cover = 0 
            level = 0
            heapq.heappush(self.q, (edge_diff + neighbors + shortcut_cover + level, v))

    def dijkstra(self, s, c, max_):
        q = []
        dist = defaultdict(lambda: self.INFINITY)
        dist[s] = 0
        heapq.heappush(q, (0, s))

        while q:
            u = heapq.heappop(q)[1]

            for i, v in enumerate(adj[0][u]):
                if v == c:
                    continue
                if dist[v] >= max_:
                    return dist
                if dist[v] > dist[u] + self.cost[0][u][i]:
                    dist[v] = dist[u] + self.cost[0][u][i]
                    heapq.heappush(q, (dist[v], v))
        return dist

    # See description of this method in the starter for friend_suggestion
    def clear(self):
        for v in self.workset:
            self.bidistance[0][v] = self.bidistance[1][v] = self.INFINITY
            self.visited[v] = False;
        del self.workset[:]

    # See description of this method in the starter for friend_suggestion
    def visit(self, q, side, v, dist):
        """Try to relax the distance to node v from direction side by value dist."""
        if self.bidistance[side][v] > dist:
            self.bidistance[side][v] = dist
            heapq.heappush(q[side], (self.bidistance[side][v], v))

    def process(self, u, adj, cost, side, q):
        for i, v in enumerate(adj[side][u]):
            if self.rank[v] < self.rank[u]:
                continue
            self.visit(q, side, v, self.bidistance[side][u] + cost[side][u][i])

    def remove_edges(self):
        for node in range(len(self.adj[0])):
            for i, v in enumerate(adj[0][node]):
                if self.rank[v] < self.rank[node]:
                    self.cost[0][node][i] = self.INFINITY
        for node in range(len(self.adj[1])):
            for i, v in enumerate(adj[1][node]):
                if self.rank[v] < self.rank[node]:
                    self.cost[1][node][i] = self.INFINITY

    def preprocess(self):

        rank = 0

        self.preImportance()
        
        while len(self.q):

            u = heapq.heappop(self.q)[1]

            try:
                second = heapq.heappop(self.q)

                importance, shortcuts = self.shortcut(u)

                if importance <= second[0]:
                    for item in shortcuts:
                        self.add_arc(*item)
                    self.rank[u] = rank
                    self.recalibrateNeighbours(u)
                else:
                    heapq.heappush(self.q, (importance, u))

                rank += 1
                heapq.heappush(self.q, (second[0], second[1]))
            except:
                self.rank[u] = max(self.rank) + 1

        # self.remove_edges()

    # Returns the distance from s to t in the graph
    def query(self, s, t):
        self.clear()
        estimate = self.INFINITY
        q = [[], []]

        self.visit(q, 0, s, 0)
        self.visit(q, 1, t, 0)
        self.mark_visited(s)
        self.mark_visited(t)

        if s == t:
            return 0

        while q[0] or q[1]:
            try:
                u = heapq.heappop(q[0])[1]
                if self.bidistance[0][u] <= estimate:
                    self.process(u, adj, cost, 0, q)

                if self.visited[u] == True and self.bidistance[0][u] + self.bidistance[1][u] < estimate:
                    estimate = self.bidistance[0][u] + self.bidistance[1][u]
                self.mark_visited(u)
            except:
                pass

            try:
                u = heapq.heappop(q[1])[1]
                if self.bidistance[0][u] <= estimate:
                    self.process(u, adj, cost, 1, q)

                if self.visited[u] == True and self.bidistance[0][u] + self.bidistance[1][u] < estimate:
                    estimate = self.bidistance[0][u] + self.bidistance[1][u]
                self.mark_visited(u)
            except:
                pass

        return -1 if estimate == self.INFINITY else estimate


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

    ch = DistPreprocessSmall(n, adj, cost)
    ch.preprocess()
    print("Ready")
    sys.stdout.flush()
    t, = readl()
    for i in range(t):
        s, t = readl()
        print(ch.query(s-1, t-1))