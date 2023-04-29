import math

# EXPERIMENTAL PARAMETERS ------------
eks_a = 0.25
eks_Frl = 0.02
eks_Magv = 5
eks_g = 9.8
eks_Vmax = 2
# ------------

# GLOBAL PARAMETERS ---------------------
a = 8.0
b = 8.05
c = 8.1

w = 0.0
x = 0.5
y = 1.0
z = 1.5

G = {'A': {'E': [b, y], 'F': [a, x], 'B': [c, -x]}, 'B': {'A': [c, x], 'E': [b, z], 'F': [a, y], 'G': [a, z], 'C': [c, w]}, 
     'C': {'B': [c, w], 'F': [b, y], 'G': [c, z], 'H': [b, x], 'D': [c, x]}, 'D': {'C': [c, -x], 'G': [a, y], 'H': [c, w]}, 
     'E': {'A': [b, -y], 'B': [b, -z], 'F': [a, -x], 'J': [b, -x], 'I': [c, -y]}, 'F': {'B': [a, -y], 'A': [a, -x], 'E': [a, x], 'I': [a, -x], 'J': [c, w], 'K': [a, -y], 'G': [c, x], 'C': [b, -y]}, 
     'G': {'C': [c, -z], 'B': [a, -z], 'F': [c, -x], 'J': [b, -x], 'K': [a, -z], 'L': [c, -z], 'H': [a, -y], 'D': [a, -y]}, 'H': {'D': [c, -w], 'C': [b, -x], 'G': [a, y], 'K': [b, -x], 'L': [a, -x]}, 
     'I': {'E': [c, y], 'F': [a, x], 'J': [b, x]}, 'J': {'I': [b, -x], 'E': [b, x], 'F': [c, -w], 'G': [b, x], 'K': [c, -y]}, 
     'K': {'J': [c, y], 'F': [a, y], 'G': [a, z], 'H': [b, x], 'L': [b, w]}, 'L': {'K': [b, -w], 'G': [c, z], 'H': [a, x]}}

S = set()
AVG_Load = {'A':5, 'B':5, 'C':2, 'D':2, 'E':4, 'F':2, 'G':5, 'H':2, 'I':1, 'J':2, 'K':4, 'L':2}
distance = {}
M = 0
# ------------

def Load(v):
    global AVG_Load
    return AVG_Load[v]

def Weight_equation(M, a, g, theta, d, v, f_rl):
    weight = (M*a + f_rl*M*g + M*g*math.sin(math.radians(theta))) * d
    return weight

def Calculate_weight(M, a, g, v, f_rl, G):
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
                weight[i][j] = Weight_equation(M, a, g, G[i][j][1], G[i][j][0], v, f_rl)
                #weight[j][i] = Weight_equation(M, a, g, -theta, G[i][j], v, f_rl)
    
    return weight


def Dijkstra_shortestPath(G, v):
    global S
    global M
    global distance
    if v not in S:
        S.add(v)

    M = M - Load(v)
    weight = Calculate_weight(M, eks_a, eks_g, eks_Vmax, eks_Frl, G)
    distance[v] = 0
    for i in G:
        distance[i] = weight[v][i]

    # for i in G:
        

    # Among vertices not belonging to S, vertex whose distance [] is minimum
    G2 = set(G.keys()) - S
    u = min(G2, key=lambda k: distance.get(k, float('inf')))
    
    for i in G2:
        print(distance[i])

    if u not in S:
        S.add(u)

    for j in G:
        if j not in S:
            distance[j] = min(distance[j], distance[u] + weight[u][j])
                
    return u


def Main(G):
    # Initialize the set M and the starting vertex u
    global M
    M = M + eks_Magv
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
        u = Dijkstra_shortestPath(G, u)
        
        #Sum of the weight starting vertex and arrival vertex
        #E = E + G[u]

        path.append(u)
        print('vertex=' + u)
        # Remove the visited vertices from the set M
        V.remove(u)
        
    return path

result = Main(G)
print(result)

