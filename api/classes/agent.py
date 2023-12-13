from itertools import permutations

class Agent():
    def __init__(self, coin_distance):
        self.coin_distance = coin_distance
    
    def getPath(self, coin_distance):
        pass


class Aki(Agent):
    def __init__(self, coin_distance):
        super().__init__(coin_distance)
    
    def getPath(self, coin_distance):
        path = greedy_dfs(coin_distance)

        return path
    

class Jocke(Agent):
    def __init__(self, coin_distance):
        super().__init__(coin_distance)
    
    def getPath(self, coin_distance):
        path = brute_force(coin_distance)

        return path
    
#Aki - dfs
def dfs(coin_distance, node, visited, path):
    path.append(node)
    visited[node] = True

    if all(visited):
        path.append(path[0])
        return path

    min_distance = float('inf')
    next_node = -1
    for i in range(len(coin_distance)):
        if not visited[i] and coin_distance[node][i] != 0 and coin_distance[node][i] < min_distance:
            min_distance = coin_distance[node][i]
            next_node = i

    if next_node != -1:
        new_path = dfs(coin_distance, next_node, visited, path)
        if all(visited):
            return new_path

    return path

def greedy_dfs(coin_distance):
    start = 0
    visited = [False] * len(coin_distance)
    path = dfs(coin_distance, start, visited, [])
    return path


#Jocke - brute force
def brute_force(coin_distance):
        paths = []
        min_path_cost = float('inf')
        min_path = []
        perm_set = [i for i in range(1, len(coin_distance))]
        perm_arr = permutations(perm_set)
        # Add zeros to the start and end of the path and then calcuate the path
        for permutation in perm_arr:
            paths.append([0] + list(permutation) + [0])
        for path in paths:
            currSum = 0
            for i in range(len(path) - 1):
                currSum += coin_distance[path[i]][path[i + 1]]
            if currSum < min_path_cost:
                min_path_cost = currSum
                min_path = path
        return min_path

