#Uses python3

import sys


def acyclic(adj):

    # for n,item in enumerate(adj):

    visited = [0 for item in adj]
    preorder = [0 for item in adj]
    postorder = [0 for item in adj]

    cc = 1

    # Note that there are two ways to make cc global. You can declare 'nonlocal cc' inside explore 
    # or pass cc as an argument to explore
    def explore(v, cc):
        # nonlocal cc
        visited[v] = 1
        preorder[v] = cc    
        cc += 1
        for n in adj[v]:
            if visited[n] == 0:
                cc = explore(n, cc)
        postorder[v] = cc
        cc += 1
        return cc

    for n, item in enumerate(adj):
        if visited[n] == 0:
            cc = explore(n, cc)

    sort = [i[0] for i in sorted(enumerate(preorder), reverse=True, key=lambda x:x[1])]

    reverse_visited = [0 for item in sort]
    cycle = 0

    def reverse_explore(v):
        nonlocal cycle
        reverse_visited[v] = 1
        for n in adj[v]:
            if reverse_visited[n] == 0: 
                if preorder[n] < preorder[v] and postorder[n] > postorder[v]:
                    cycle = 1
                 
    for n in sort:
        if reverse_visited[n] == 0:
            reverse_explore(n)

    return cycle

if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(zip(data[0:(2 * m):2], data[1:(2 * m):2]))
    adj = [[] for _ in range(n)]
    for (a, b) in edges:
        adj[a - 1].append(b - 1)
    print(acyclic(adj))
