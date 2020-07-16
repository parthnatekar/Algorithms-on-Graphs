#Uses python3

import sys


def negative_cycle(adj, cost):

    longest = []
    for item in cost:
        longest.extend(item)

    dist = [sum(longest)+1 for i in adj]
    prev = [-1  for i in adj]

    dist[0] = 0

    def relax(u, v, i):
        if dist[v] > dist[u] + cost[u][i]:
            dist[v] = dist[u] + cost[u][i]
            prev[v] = u
            return 1
        return 0

    for i in range(len(adj)):
        for s, item in enumerate(adj):
            for index, t in enumerate(item):
                r = relax(s, t, index)
                if r ==1 and i == len(adj) - 1:
                    return 1

    return 0


if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(zip(zip(data[0:(3 * m):3], data[1:(3 * m):3]), data[2:(3 * m):3]))
    data = data[3 * m:]
    adj = [[] for _ in range(n)]
    cost = [[] for _ in range(n)]
    for ((a, b), w) in edges:
        adj[a - 1].append(b - 1)
        cost[a - 1].append(w)
    print(negative_cycle(adj, cost))
