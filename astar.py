import math
import heapq


class Cell:
    def __init__(self):
        self.parent_r = 0
        self.parent_c = 0
        self.cell_cost = float('inf')
        self.start_cost = float('inf')


max_row = occupancyGrid.x
max_col = occupancyGrid.y


def cell_isvalid(row, col):
    return (row>=0) and (row < max_row) and (col >= 0) and (col < max_col)


def cell_is_unblocked(grid, row, col):
    return grid[row][col] == 1


def cell_is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]


def astar_search(grid, src, dest):

    #check if cell is valid

    if not cell_isvalid(src[0], src[1]):
        print ("Source is invalid")
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

    cell_list = [[Cell() for _ in range(max_col)] for _ in range(max_row)]

    visited_list = [[False for _ in range(max_col)] for _ in range(max_row)]

    closed





def main():
    grid = Grid()