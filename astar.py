import heapq


class Cell:
    def __init__(self):
        self.parent_r = 0
        self.parent_c = 0
        self.cell_cost = float('inf')
        self.start_cost = float('inf')
        self.heuristic_cost = 0


max_row = 9
max_col = 10
path = []


def cell_isvalid(row, col):
    return (row >= 0) and (row < max_row) and (col >= 0) and (col < max_col)


def cell_is_unblocked(grid, row, col):
    return grid[row][col] == 1


def cell_is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]


def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5


def print_grid_with_path(grid, path, dest, src):
    src_row, src_col = src
    dest_row, dest_col = dest
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if (i, j) == src:
                print("&", end=" ")
            elif (i, j) == dest:
                print("&", end=" ")
            elif (i, j) in path:
                print("*", end=" ")
            elif grid[i][j] == 0:
                print("#", end=" ")
            else:
                print(".", end=" ")
        print()


def trace_path(cell_details, dest, src):

    print("The path is ")
    row = dest[0]
    col = dest[1]

    while not (cell_details[row][col].parent_r == row and cell_details[row][col].parent_c == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_r
        temp_col = cell_details[row][col].parent_c
        row = temp_row
        col = temp_col

    path.append((row, col))
    path.reverse()


def astar_search(grid, src, dest):

    if not cell_isvalid(src[0], src[1]):
        print("Source is invalid")
        return

    if not cell_isvalid(dest[0], dest[1]):
        print("Destination is invalid")
        return

    if not cell_is_unblocked(grid, src[0], src[1]):
        print("Source is blocked")
        return

    if not cell_is_unblocked(grid, dest[0], dest[1]):
        print("Destination is blocked")
        return

    if cell_is_destination(src[0], src[1], dest):
        print("Arrived at destination")
        return

    visited_list = [[False for _ in range(max_col)] for _ in range(max_row)]

    cell_list = [[Cell() for _ in range(max_col)] for _ in range(max_row)]

    r = src[0]
    c = src[1]
    cell_list[r][c].cell_cost = 0
    cell_list[r][c].start_cost = 0
    cell_list[r][c].heuristic_cost = 0
    cell_list[r][c].parent_r = r
    cell_list[r][c].parent_c = c

    open_list = []
    heapq.heappush(open_list, (0.0, r, c))

    found_dest = False

    while len(open_list) > 0:
        p = heapq.heappop(open_list)
        r = p[1]
        c = p[2]
        visited_list[r][c] = True

        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1),  (1, -1), (-1, 1), (-1, -1)]

        for dir in directions:
            new_r = r + dir[0]
            new_c = c + dir[1]

            if cell_isvalid(new_r, new_c) and cell_is_unblocked(grid, new_r, new_c) and not visited_list[new_r][new_c]:
                if cell_is_destination(new_r, new_c, dest):
                    cell_list[new_r][new_c].parent_r = r
                    cell_list[new_r][new_c].parent_c = c
                    print("Destination cell found!")
                    trace_path(cell_list, dest, src)
                    return
                else:
                    g_new = cell_list[r][c].cell_cost + 1.0
                    h_new = calculate_h_value(new_r, new_c, dest)
                    f_new = g_new + h_new

                    if cell_list[new_r][new_c].start_cost == float('inf'):
                        heapq.heappush(open_list, (f_new, new_r, new_c))
                        cell_list[new_r][new_c].start_cost = f_new
                        cell_list[new_r][new_c].cell_cost = g_new
                        cell_list[new_r][new_c].heuristic_cost = h_new
                        cell_list[new_r][new_c].parent_r = r
                        cell_list[new_r][new_c].parent_c = c

    if not found_dest:
        print("Failed to find destination cell!")


def main():

    grid = [
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
    ]

    src = [2, 7]
    dest = [0, 0]
    astar_search(grid, src, dest)
    print_grid_with_path(grid, path, dest, src)


if __name__ == "__main__":
    main()
