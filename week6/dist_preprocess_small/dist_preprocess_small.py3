#!/usr/bin/python3


import sys
import queue


# Maximum allowed edge length
maxlen = 2 * 10**6


class DistPreprocessSmall:
    def __init__(self, n, adj, cost):
        # See description of these parameters in the starter for friend_suggestion
        self.n = n
        self.INFINITY = n * maxlen
        self.adj = adj
        self.cost = cost
        self.bidistance = [[self.INFINITY] * n, [self.INFINITY] * n]
        self.visited = [False] * n
        self.visited = []
        self.q = []
        # Levels of nodes for node ordering heuristics
        self.level = [0] * n
        # Positions of nodes in the node ordering
        self.rank = [0] * n

        self.dist = [self.INFINITY] * n
        self.neighbors = [0] * n

        estimate = self.INFINITY
        rank = 0

        self.preImportance()
        
        while self.q:

            u = heapq.heappop(self.q)[1]

            second = heapq.heappop(self.q)

            importance, shortcuts = self.shortcut(self, v)

            if importance <= second[0]:
                for item in shortcuts:
                    self.add_arc(*item)
                self.rank[u] = rank
                self.recalibrateNeighbours(u)
            else:
                heapq.heappush(q, (importance, u))

            rank += 1
            heapq.heappush(q, (isecond[0], second[1]))


    def mark_visited(self, x):
        if not self.visited[x]:
            self.visited[x] = True
            self.visited.append(x)

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
        neighbors = self.neighbours[v]
        level = self.level[v]
        shortcuts = []
        
        maxDist = max(self.adj[1][v]) + max(self.adj[0][v])
        for i, s in enumerate(self.adj[1][v]):
            if self.rank[s] < self.rank[v]:
                continue
            self.dijkstra(s, maxDist)
            for j, t in enumerate(self.adj[0][v]):
                if self.rank[t] < self.rank[v] or t == v:
                    continue
                if self.cost[s][i] + self.cost[t][j] < self.dist[t]:
                    shortcut_count += 1
                    shortcut_cover += 2
                    shortcuts.append((s, t, self.cost[1][v][i] + self.cost[0][v][j]))

        # Compute correctly the values for the above heuristics before computing the node importance
        importance = (shortcut_count - len(self.adj[0][v]) - len(self.adj[1][v])) + neighbors + shortcut_cover + level
        return importance, shortcuts

    def recalibrateNeighbours(self, v, preCompute):
        for i, s in enumerate(self.adj[1][v]):
                self.neighbors[s] += 1
                self.level[s] = max(self.level[s], self.level[v] + 1)
        for i, t in enumerate(self.adj[0][v]):
                self.neighbors[t] += 1
                self.level[t] = max(self.level[s], self.level[v] + 1)
        return neighbors


    def preImportance(self):
        for v in range(n):
            edge_diff = (len(self.adj[0][v]) * len(self.adj[1][v])  - len(self.adj[0][v]) - len(self.adj[1][v]))
            neighbors = len(self.adj[0][v]) + len(self.adj[1][v])
            shortcut_cover = 0 
            level = 0
            heapq.heappush(self.q, (edge_diff + neighbours + shortcut_cover + level, v))

    def dijkstra(self, s, max):
        q = []
        heapq.heappush(q, (0, s))

        while q:
            u = heapq.heappop(q)[1]

            if self.visited[u] == True:
                continue

            for i, v in enumerate(adj[u]):
                if self.dist[v] > max:
                    return
                if self.dist[v] > self.dist[u] + self.cost[0][u][i]:
                    self.dist[v] = self.dist[u] + self.cost[0][u][i]
                    heapq.heappush(q, (self.dist[v], v))

    # See description of this method in the starter for friend_suggestion
    def clear():
        for v in self.visited:
            self.bidistance[0][v] = self.bidistance[1][v] = self.INFINITY
            self.visited[v] = False;
        del self.visited[:]

    # See description of this method in the starter for friend_suggestion
    def visit(side, v, dist):
        # Implement this method yourself
        pass

    # Returns the distance from s to t in the graph
    def query(self, s, t):
        q = [[], []]

        visit(0, s, 0) #Visit is redundant, just put s in queue with dist 0 and t in q_r with dist 0
        visit(1, t, 0)

        #JUST RUN DIJKSTRA ON AUGMENTED GRAPH

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
    print("Ready")
    sys.stdout.flush()
    t, = readl()
    for i in range(t):
        s, t = readl()
        print(ch.query(s-1, t-1))