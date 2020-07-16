#Uses python3

import sys

def reach(adj, x, y):
    visited = [0 for item in adj]
    CCnum = [0 for item in adj]

    def explore(v):
        visited[v] = 1
        CCnum[v] = 1
        for n in adj[v]:
            if visited[n] == 0:
                explore(n)

    explore(x)

    return(CCnum[y])

if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(zip(data[0:(2 * m):2], data[1:(2 * m):2]))
    x, y = data[2 * m:]
    adj = [[] for _ in range(n)]
    x, y = x - 1, y - 1
    for (a, b) in edges:
        adj[a - 1].append(b - 1)
        adj[b - 1].append(a - 1)
    print(reach(adj, x, y))
