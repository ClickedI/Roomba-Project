# Class to represent a node in the grid
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')  # g value (cost from start)
        self.h = 0  # h value (heuristic cost to end)
        self.parent = None  # parent node


# Function to calculate the Manhattan distance between two nodes
def manhattan_distance(node1, node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)


# Function to generate neighbors of a node using JPS
def get_neighbors(grid, current, end):
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up

    for dx, dy in directions:
        x, y = current.x + dx, current.y + dy
        while 0 <= x < len(grid) and 0 <= y < len(grid[0]) and not grid[x][y]:
            if x == end.x and y == end.y:
                return [(x, y)]
            neighbors.append((x, y))
            x, y = x + dx, y + dy

    return neighbors


# JPS algorithm implementation
def jps(grid, start, end):
    open_list = []  # list of open nodes
    closed_list = set()  # set of closed nodes

    start_node = Node(start[0], start[1])
    end_node = Node(end[0], end[1])

    start_node.g = 0
    start_node.h = manhattan_distance(start_node, end_node)

    open_list.append(start_node)

    while open_list:
        current = min(open_list, key=lambda node: node.g + node.h)
        open_list.remove(current)
        closed_list.add((current.x, current.y))

        if current.x == end_node.x and current.y == end_node.y:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        for neighbor in get_neighbors(grid, current, end_node):
            x, y = neighbor
            if (x, y) not in closed_list:
                neighbor_node = Node(x, y)
                neighbor_node.g = current.g + 1
                neighbor_node.h = manhattan_distance(neighbor_node, end_node)
                neighbor_node.parent = current
                open_list.append(neighbor_node)

    return None  # No path found


def print_grid_with_path(grid, path, start, end):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if (i, j) == start:
                print("&", end=" ")
            elif (i, j) == end:
                print("&", end=" ")
            elif grid[i][j] == 1:
                print("$", end=" ")
            elif (i, j) in path:
                print("*", end=" ")
            else:
                print(".", end=" ")
        print()


def main():
    # Example usage
    grid = [
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 0, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    start = (0, 0)
    end = (4, 4)

    path = jps(grid, start, end)
    if path:
        print("Path found:")
        print_grid_with_path(grid, path, start, end)
    else:
        print("No path found")


if __name__ == "__main__":
    main()
