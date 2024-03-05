"""
dijkstra.py

@breif:     This module implements Dijkstra's algorithm for finding the shortest path in a graph.
@author:    Kshitij Karnawat
@date:      5th March 2024
@version:   1.0
"""

import numpy as np
import cv2 as cv


## OpenCV X axis moves up and down
## OpenCV Y axis moves left and right

class NewNode:
    def __init__(self, coord, parent, cost):
        self.coord = coord
        self.parent = parent
        self.cost = cost
    
def move_up(node):
    x, y = node.coord
    return NewNode((x + 1, y), node, node.cost + 1)

def move_down(node):
    x, y = node.coord
    return NewNode((x - 1, y), node, node.cost + 1)

def move_left(node):
    x, y = node.coord
    return NewNode((x, y - 1), node, node.cost + 1)

def move_right(node):
    x, y = node.coord
    return NewNode((x, y + 1), node, node.cost + 1)

def move_up_left(node):
    x, y = node.coord
    return NewNode((x + 1, y - 1), node, node.cost + 1.4)

def move_up_right(node):
    x, y = node.coord
    return NewNode((x + 1, y + 1), node, node.cost + 1.4)

def move_down_left(node):
    x, y = node.coord
    return NewNode((x - 1, y - 1), node, node.cost + 1.4)

def move_down_right(node):
    x, y = node.coord
    return NewNode((x - 1, y + 1), node, node.cost + 1.4)

def in_obstacles(coord):
    # TODO: Implement this function returning True if the given coordinate is in the obstacle space and False otherwise
    return True

def get_child_nodes(node):

    # Set Max and Min values for x and y
    x_max, y_max = 500, 1200
    x_min, y_min = 0, 0

    # Get the coordinates of the node
    x, y = node.coord

    # child nodes list
    child_nodes = []

    # Create all possible child nodes
    if x < x_max:
        child = move_up(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    if x > x_min:
        child = move_down(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    if y < y_max:
        child = move_right(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    if y > y_min:
        child = move_left(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    if x < x_max and y < y_max:
        child = move_up_right(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    if x < x_max and y > y_min:
        child = move_up_left(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    if x > x_min and y < y_max:
        child = move_down_right(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child
    
    if x > x_min and y > y_min:
        child = move_down_left(node)
        if not in_obstacles(child.coord):
            child_nodes.append(child)
        else:
            del child

    return child_nodes

def create_map():
    # TODO: Create canvas
    game_map = np.zeros((500, 1200))
    game_map.fill(255)


    # TODO: Create obstacles

    return game_map


def main():
    game_map = create_map()
    
    # show map
    cv.imshow('Map', game_map)

    # wait for key press
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()