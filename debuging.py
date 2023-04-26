#u = min(G2, key=lambda k: distance.get(k, float('inf')))


G = {'A': {'B': 1, 'C': 4}, 'B': {'C': 2}, 'C': {'D': 3}}
S = {'E'}
V = set(G.keys())

distance = {'A': 1, 'B': 2, 'C': 3}

G2 = set(G.keys()) - S
u = min(G2, key=lambda k: distance.get(k, float('inf')))

V.remove('B')
print(u)

# graph = {'A': 4, 'B': 2, 'C': 7}

# min_key = min(graph, key=lambda k: graph[k])

# print("Key with minimum value:", min_key)
