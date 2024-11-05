import heapq
import math

# 定义节点的坐标，用于计算直线距离（启发式）
node_positions = {
    'A': (10, 7), 'B': (8, 9), 'C': (7, 7), 'D': (7, 5), 'E': (4, 7),
    'F': (6, 5), 'G': (4, 5), 'H': (3, 5), 'I': (2, 3), 'J': (2, 4)
}

# 定义图的邻接列表，去掉C和D之间的边
graph = {
    'A': {'B': 5, 'C': 4, 'D': 6},
    'B': {'A': 5, 'E': 8},
    'C': {'A': 4, 'G': 5},
    'D': {'A': 6, 'F': 2},
    'E': {'B': 8, 'H': 3},
    'F': {'D': 2, 'G': 2},
    'G': {'F': 2, 'H': 1,'C':5},
    'H': {'E': 3, 'G': 1, 'I': 4},
    'I': {'H': 4, 'J': 1},
    'J': {'I': 1}
}

# 计算两个节点之间的曼哈顿距离（作为启发式）
def heuristic(node1, node2):
    x1, y1 = node_positions[node1]
    x2, y2 = node_positions[node2]
    return abs(x1 - x2) + abs(y1 - y2)

# A*算法实现
def a_star_algorithm(start, goal):
    # 优先队列，存储待访问的节点及其优先级
    open_list = []
    heapq.heappush(open_list, (0, start))

    # 跟踪从起点到每个节点的最小代价
    g_costs = {node: float('inf') for node in graph}
    g_costs[start] = 0

    # 跟踪最佳路径
    came_from = {}

    while open_list:
        # 取出优先级最低的节点
        current_cost, current_node = heapq.heappop(open_list)

        # 如果到达目标节点，返回路径
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            return path[::-1]  # 返回从起点到目标节点的路径

        # 遍历邻接节点
        for neighbor, weight in graph[current_node].items():
            tentative_g_cost = g_costs[current_node] + weight

            # 如果找到更短的路径
            if tentative_g_cost < g_costs[neighbor]:
                came_from[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_cost = tentative_g_cost + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_cost, neighbor))

    return None  # 如果找不到路径

# 执行A*算法从A到J
path = a_star_algorithm('A', 'J')
print("最短路径:", path)