#Uses python3

import sys

sys.setrecursionlimit(200000)

def dfs(adj):

    visited = [0 for item in adj]
    preorder = [0 for item in adj]
    postorder = [0 for item in adj]

    cc = 1

    # Note that there are two ways to make cc global. You can declare 'nonlocal cc' inside explore 
    # or pass cc as an argument to explore
    def explore(v):
        nonlocal cc
        visited[v] = 1
        preorder[v] = cc    
        cc += 1
        for n in adj[v]:
            if visited[n] == 0:
                explore(n)
        postorder[v] = cc
        cc += 1

    for n, item in enumerate(adj):
        if visited[n] == 0:
            explore(n)

    return(postorder)

def number_of_strongly_connected_components(adj):

    adj_r = [[] for item in adj]

    for n,item in enumerate(adj):
        for node in item:
            adj_r[node].append(n)

    postorder = dfs(adj_r)

    sort = [i[0] for i in sorted(enumerate(postorder), reverse=True, key=lambda x:x[1])]

    visited = [0 for item in adj]

    def explore(v):
        visited[v] = 1
        for n in adj[v]:
            if visited[n] == 0:
                explore(n)
        component[v] = cc
        sort.remove(v)

    component = [0 for item in adj]
    cc = 0
    while sort:
        if visited[sort[0]] == 0:
            explore(sort[0])
        cc +=1

    return max(component)+1

if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n, m = data[0:2]
    data = data[2:]
    edges = list(zip(data[0:(2 * m):2], data[1:(2 * m):2]))
    adj = [[] for _ in range(n)]
    for (a, b) in edges:
        adj[a - 1].append(b - 1)
    print(number_of_strongly_connected_components(adj))
