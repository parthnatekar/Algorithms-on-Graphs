#Uses python3

import sys

def dfs(adj, used, order, x):

    # Note that there are two ways to make cc global. You can declare 'nonlocal cc' inside explore 
    # or pass cc as an argument to explore
    def explore(v):
        used[v] = 1  
        for n in adj[v]:
            if used[n] == 0:
                explore(n)
        order.append(v)

    for n, item in enumerate(adj):
        if used[n] == 0:
            explore(n)

    return order


def toposort(adj):
    used = [0] * len(adj)
    order = []

    order = dfs(adj, used, order, 0)

    order.reverse()

    return order

if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(zip(data[0:(2 * m):2], data[1:(2 * m):2]))
    adj = [[] for _ in range(n)]
    for (a, b) in edges:
        adj[a - 1].append(b - 1)
    order = toposort(adj)
    for x in order:
        print(x + 1, end=' ')

