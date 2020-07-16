#Uses python3

import sys
import queue

def distance(adj, cost, s, t):

    longest = []
    for item in cost:
        longest.extend(item)

    dist = [sum(longest)+1 for i in adj]
    prev = [-1  for i in adj]

    dist[s] = 0

    def ChangePriority(H, v):
        H = sorted(H, key=lambda x:dist[x])
        return H 

    H = [i[0] for i in sorted(enumerate(dist), key=lambda x:x[1])]

    while H:
        u = H[0]
        H.remove(u)
        for i, v in enumerate(adj[u]):
            if dist[v] > dist[u] + cost[u][i]:
                dist[v] = dist[u] + cost[u][i]
                prev[v] = u
                H = ChangePriority(H, v)

    if dist[t] == sum(longest)+1:
        return -1
    return dist[t]


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
    s, t = data[0] - 1, data[1] - 1
    print(distance(adj, cost, s, t))
