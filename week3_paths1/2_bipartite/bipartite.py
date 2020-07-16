#Uses python3

import sys
import queue

def bipartite(adj):
    
    dist = [-1 for item in adj]
    CClevel = [0 for item in adj]

    cc = 0

    dist[0] = 0
    CClevel[0] = 0

    queue = [0]

    while queue:
        # Process phase
        cc = 1 - cc
        s = queue[0]
        for n in adj[s]:
            # Discover phase
            if dist[n] == -1:
                queue.append(n)
                dist[n] = dist[s] + 1
                CClevel[n] = cc
        queue.pop(0)

    for n, item in enumerate(adj):
        for nbr in item:
            if CClevel[n] == CClevel[nbr]:
                return 0
    return 1

if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(zip(data[0:(2 * m):2], data[1:(2 * m):2]))
    adj = [[] for _ in range(n)]
    for (a, b) in edges:
        adj[a - 1].append(b - 1)
        adj[b - 1].append(a - 1)
    print(bipartite(adj))
