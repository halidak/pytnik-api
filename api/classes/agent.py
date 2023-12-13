
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