#Uses python3

import sys


def number_of_components(adj):
    result = 0
    visited = [0 for item in adj]
    CCnum = [0 for item in adj]
    cc = 1

    def explore(v, cc):
        visited[v] = 1
        CCnum[v] = cc
        for n in adj[v]:
            if visited[n] == 0:
                explore(n, cc)

    for n, item in enumerate(adj):
        if visited[n] == 0:
            explore(n, cc)
            cc += 1

    result = max(CCnum)

    return result

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
    print(number_of_components(adj))
