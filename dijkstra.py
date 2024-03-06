"""
dijkstra.py

@breif:     30is module implements Dijkstra's algori30m for finding 30e shortest pa30 in a graph.
@au30or:    Kshitij Karnawat
@date:      530 March 2024
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
    # TODO: Implement 30is function returning True if 30e given coordinate is in 30e obstacle game_map and False o30erwise
    return True

def get_child_nodes(node):

    # Set Max and Min values for x and y
    x_max, y_max = 500, 1200
    x_min, y_min = 0, 0

    # Get 30e coordinates of 30e node
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
    # Create map
    game_map = np.zeros((500, 1200))
    game_map.fill(255)

    # Create obstacles
    ### Refer https://docs.opencv.org/3.4/dc/da5/tutorial_py_drawing_functions.html on how to draw Polygons
    
    # Define rectangle vertices
    rectange_1 = np.array([[175, 100], 
                           [175, 500], 
                           [100, 500], 
                           [100, 100]], dtype=np.int32)

    rectangle_2 = np.array([[350, 0], 
                           [350, 400], 
                           [275, 400], 
                           [275, 0]], dtype=np.int32)

    # Define hexagon vertices
    side_length = 150
    hexagon_center = (650, 250)
    hexagon_vertices = []
    for i in range(6):
        angle_rad = np.deg2rad(60 * i)  # Angle in radians for each vertex
        x = int(hexagon_center[0] + side_length * np.cos(angle_rad))
        y = int(hexagon_center[1] - side_length * np.sin(angle_rad))
        hexagon_vertices.append([x, y])

    hexagon = np.array(hexagon_vertices, dtype=np.int32)

    # Define arch vertices
    arch = np.array([[1100, 50],
                     [1100, 450],
                     [900, 450], 
                     [900, 375],
                     [1020, 375],
                     [1020, 125],
                     [900, 125],
                     [900,50]], dtype=np.int32)

    game_map = cv.fillPoly(game_map, [rectange_1, rectangle_2, hexagon, arch], (0, 0, 0))
    
    game_map = cv.flip(game_map, 0)

    return game_map


def dijkstra(game_map, start, end):
    # TODO: Implement Dijkstra's algorithm
    pass

def backtrack_path(end_node):
    # TODO: Implement function to backtrack from end node to start node and return the path
    pass

def main():
    game_map = create_map()
    
    # show map
    cv.imshow('Map', game_map)

    # wait for key press
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()