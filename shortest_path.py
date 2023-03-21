import heapq

def astar(start, end, grid):
    """
    Find the shortest path from start to end on the given grid using the A* algorithm.
    """
    rows = len(grid)
    cols = len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == end:
            break
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            next_node = (current[0] + dx, current[1] + dy)
            if next_node[0] < 0 or next_node[0] >= rows or next_node[1] < 0 or next_node[1] >= cols:
                continue
            new_cost = cost_so_far[current] + grid[next_node[0]][next_node[1]]
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(end, next_node)
                heapq.heappush(open_set, (priority, next_node))
                came_from[next_node] = current
    
    path = []
    current = end
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def heuristic(a, b):
    """
    Calculate the heuristic distance between nodes a and b.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])



"""
Create a 3x3 grid
"""
grid = [[1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]]


start = (0, 0)
end = (2, 2)
path = astar(start, end, grid)