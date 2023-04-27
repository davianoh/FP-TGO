import math

# EXPERIMENTAL PARAMETERS ------------
eks_a = 0.25
eks_Frl = 0.02
eks_Magv = 5
eks_g = 9.8
eks_Vmax = 2
# ------------

# GLOBAL PARAMETERS ---------------------
G = {'A': {'B': 5, 'C': 3}, 'B': {'A': 5, 'C': 1, 'D': 4}, 'C': {'A': 3, 'B': 1, 'D': 2}, 'D': {'B': 4, 'C': 2}}
S = set()
AVG_Load = {'A':5, 'B':5, 'C':5, 'D':5}
# ------------

def Load(v):
    global AVG_Load
    return AVG_Load[v]

def Weight_equation(M, a, g, theta, d, v, f_rl):
    weight = (M*a + f_rl*M*g + M*g*math.sin(theta)) * d
    return weight

def Calculate_weight(M, a, g, theta, v, f_rl, G):
    # Initialize the weight matrix to infinity for all vertex pairs
    weight = {}
    for i in G:
        weight[i] = {}
        for j in G:
            weight[i][j] = float('inf')
    
    # Set the weight of self-loops to zero
    for i in G:
        for j in G:
            if(i == j):
                weight[i][j] = 0
    
    # Calculate the weight for all edges in the graph
    for i in G:
        for j in G:
            if j in G[i]:
                weight[i][j] = Weight_equation(M, a, g, theta, G[i][j], v, f_rl)
                #weight[j][i] = Weight_equation(M, a, g, -theta, G[i][j], v, f_rl)
    
    return weight


def Dijkstra_shortestPath(G, v, M):
    global S
    if v not in S:
        S.add(v)

    M = M - Load(v)
    weight = Calculate_weight(M, eks_a, eks_g, 1, eks_Vmax, eks_Frl, G)
    distance = {v: 0}
    for i in G:
        distance[i] = weight[v][i]

    # for i in G:
        

    # Among vertices not belonging to S, vertex whose distance [] is minimum
    G2 = set(G.keys()) - S
    u = min(G2, key=lambda k: distance.get(k, float('inf')))

    if u not in S:
        S.add(u)

    for j in G:
        if j not in S:
            distance[j] = min(distance[j], distance[u] + weight[u][j])
                
    return u


def Main(G):
    # Initialize the set M and the starting vertex u
    M = eks_Magv
    for i in G:
        M = M + Load(i)
    
    
    V = set(G.keys())
    u = 'A'
    E = 0

    path = []

    path.append(u)
    print('vertex=' + u)
    V.remove(u)
    
    # Loop until all vertices have been visited
    while V:
        # Find the shortest path from u to all other vertices using Dijkstra's algorithm
        u = Dijkstra_shortestPath(G, u, M)
        
        #Sum of the weight starting vertex and arrival vertex
        #E = E + G[u]

        path.append(u)
        print('vertex=' + u)
        # Remove the visited vertices from the set M
        V.remove(u)
        
    return path

result = Main(G)
print(result)

