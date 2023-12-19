# Basic searching algorithms
from util import Stack, Queue, PriorityQueueWithFunction, PriorityQueue
import numpy as np
from collections import deque

# Class for each node in the grid
class Node:
    def __init__(self, row, col, cost, parent, goal):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.cost = cost      # total cost (depend on the algorithm)
        self.goal = goal      # Goal   
        self.parent = parent  # previous node

def manhattan_distance(node, goal):
    return abs(node.row - goal.row) + abs(node.col - goal.col)

def reconstruct_path(node):
    path = []
    while node is not None:
        path.append([node.row, node.col])
        node = node.parent
    path.reverse()
    return path


def is_valid_move(grid, row, col, visited):
    rows, cols = len(grid), len(grid[0])
    return 0 <= row < rows and 0 <= col < cols and grid[row][col] == 0 and (row, col) not in visited

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. 
            If no path exists return an empty list [].
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    start_node = Node(start[0], start[1], 0, None, goal)
    goal_node = Node(goal[0], goal[1], 0, None, goal)

    if start_node == goal_node:
        return [[start_node.row, start_node.col]], 1
    
    queue = Queue()
    queue.push(start_node)
    visited = set()
    visited.add((start_node.row, start_node.col))
    # steps = 0
    
    while not queue.isEmpty():
        current_node = queue.pop()
        steps += 1

        if current_node.row == goal_node.row and current_node.col == goal_node.col:
            found = True
            path = reconstruct_path(current_node)
            break

        for d_row, d_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # right, down, left, up
            neighbor_row, neighbor_col = current_node.row + d_row, current_node.col + d_col
            if is_valid_move(grid, neighbor_row, neighbor_col, visited):
                neighbor_node = Node(neighbor_row, neighbor_col, current_node.cost + 1, current_node, goal)
                queue.push(neighbor_node)
                visited.add((neighbor_row, neighbor_col))



    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    start_node = Node(start[0], start[1], 0, None, goal)
    goal_node = Node(goal[0], goal[1], 0, None, goal)

    if start_node == goal_node:
        return [[start_node.row, start_node.col]], 1
    
    stack = Stack()
    stack.push(start_node)
    visited = set()
    visited.add((start_node.row, start_node.col))
    steps = 0


    while not stack.isEmpty():
        current_node = stack.pop()
        steps += 1

        if current_node.row == goal_node.row and current_node.col == goal_node.col:
            found = True
            path = reconstruct_path(current_node)
            break

        for d_row, d_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # right, down, left, up
            neighbor_row, neighbor_col = current_node.row + d_row, current_node.col + d_col
            if is_valid_move(grid, neighbor_row, neighbor_col, visited):
                neighbor_node = Node(neighbor_row, neighbor_col, current_node.cost + 1, current_node, goal)
                stack.push(neighbor_node)
                visited.add((neighbor_row, neighbor_col))

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps

def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    start_node = Node(start[0], start[1], 0, None, goal)
    goal_node = Node(goal[0], goal[1], 0, None, goal)

    if start_node == goal_node:
        return [[start_node.row, start_node.col]], 1
    
    def heuristic(node):
        return abs(node.row - goal_node.row) + abs(node.col - goal_node.col)
    
    priority_queue = PriorityQueue()
    priority_queue.push(start_node, start_node.cost + heuristic(start_node))
    visited = set()
    visited.add((start_node.row, start_node.col))


    while not priority_queue.isEmpty():
        current_node = priority_queue.pop()
        steps += 1

        if current_node.row == goal_node.row and current_node.col == goal_node.col:
            found = True
            path = reconstruct_path(current_node)
            break

        for d_row, d_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # right, down, left, up
            neighbor_row, neighbor_col = current_node.row + d_row, current_node.col + d_col
            if is_valid_move(grid, neighbor_row, neighbor_col, visited):
                neighbor_node = Node(neighbor_row, neighbor_col, current_node.cost + 1, current_node, goal)
                if (neighbor_row, neighbor_col) not in visited or \
                   neighbor_node.cost < grid[neighbor_row][neighbor_col]:
                    priority_queue.push(neighbor_node, neighbor_node.cost + heuristic(neighbor_node))
                    visited.add((neighbor_row, neighbor_col))

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
