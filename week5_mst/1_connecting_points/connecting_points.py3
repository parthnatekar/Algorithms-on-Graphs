#Uses python3
import sys
import math
import collections

class DisjointSet():

    def __init__(self, vertices):
        self.vertices = vertices
        self.parent = {i:i for i in self.vertices}

    def find(self, u):
        if self.parent[u] != u:
            return(self.find(self.parent[u]))
        else:
            return self.parent[u]

    def union(self, set1, set2):
        self.parent[self.find(set2)] = self.parent[self.find(set1)]

def minimum_distance(x, y):
    result = 0.
    
    def distance(s, t):
        return(math.sqrt((s[0]-t[0])**2 + (s[1] - t[1])**2))

    dist = {}  

    for i, source in enumerate(list(zip(x,y))):
        for j, target in enumerate(list(zip(x,y))):
            if (i, j) in dist.keys() or (j, i) in dist.keys() or i == j:
                continue
            else:
                dist[(i, j)] = distance(source, target)
            
    dist = collections.OrderedDict(sorted(dist.items(), key=lambda x: x[1]))

    D = DisjointSet([p for p in range(len(x))])

    for item in list(dist.keys()):
        if D.find(item[0]) != D.find(item[1]):
            dist.pop(item)
            D.union(item[0], item[1])
            result += distance((x[item[0]], y[item[0]]), (x[item[1]], y[item[1]]))

    return result


if __name__ == '__main__':
    input = sys.stdin.read()
    data = list(map(int, input.split()))
    n = data[0]
    x = data[1::2]
    y = data[2::2]
    print("{0:.9f}".format(minimum_distance(x, y)))
