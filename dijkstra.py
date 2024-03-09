"""
dijkstra.py

@breif:     this module implements Dijkstra's algorithm for finding the shortest path in a graph.
@author:    Kshitij Karnawat
@date:      6th March 2024
@version:   1.0
"""

import numpy as np
import cv2 as cv
import time


## OpenCV X axis moves up and down
## OpenCV Y axis moves left and right

class NewNode:
    def __init__(self, coord, parent, cost):
        self.coord = coord
        self.parent = parent
        self.cost = cost
    
def move_up(node):
    x, y = node.coord
    return NewNode((x + 1, y), node, node.cost + 1), 1

def move_down(node):
    x, y = node.coord
    return NewNode((x - 1, y), node, node.cost + 1), 1

def move_left(node):
    x, y = node.coord
    return NewNode((x, y - 1), node, node.cost + 1), 1

def move_right(node):
    x, y = node.coord
    return NewNode((x, y + 1), node, node.cost + 1), 1

def move_up_left(node):
    x, y = node.coord
    return NewNode((x + 1, y - 1), node, node.cost + 1.4), 1.4

def move_up_right(node):
    x, y = node.coord
    return NewNode((x + 1, y + 1), node, node.cost + 1.4), 1.4

def move_down_left(node):
    x, y = node.coord
    return NewNode((x - 1, y - 1), node, node.cost + 1.4), 1.4

def move_down_right(node):
    x, y = node.coord
    return NewNode((x - 1, y + 1), node, node.cost + 1.4), 1.4

def in_obstacles(coord):
    # Set Max and Min values for x and y
    x_max, y_max = 500, 1200
    x_min, y_min = 0, 0

    x, y = coord

    bloat = 5

    if (x < x_min + bloat) or (x > x_max - bloat) or (y < y_min + bloat) or (y > y_max - bloat):
        return True
    
    # Rectangle 1
    elif (y >= 100 - bloat and y <= 175 + bloat) and (x >= 100 - bloat and x <= 500):
        return True
    
    # Rectangle 2
    elif (y >= 275 - bloat and y <= 350 + bloat) and (x >= 0 and x <= 400 + bloat):
        return True
    
    # Hexagon
    # TODO: Implement check for hexagon obstacle

    # Arch
    # Divide the arch into 3 parts and check for each part
    
    # Part 1
    elif (x >= 50 - bloat and x <= 450 + bloat) and (y >= 1020 - bloat and y <= 1100 + bloat):
        return True
    
    # Part 2
    elif (x >= 375 - bloat and x <= 450 + bloat) and (y >= 900 - bloat and y <= 1100 + bloat):
        return True
    
    # Part 3
    elif (x >= 50 - bloat and x <= 125 + bloat) and (y >= 900 - bloat and y <= 1100 + bloat):
        return True
    
    return False

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
        child, child_cost = move_up(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child

    if x > x_min:
        child, child_cost = move_down(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child

    if y < y_max:
        child, child_cost = move_right(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child

    if y > y_min:
        child, child_cost = move_left(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child

    if x < x_max and y < y_max:
        child, child_cost = move_up_right(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child

    if x < x_max and y > y_min:
        child, child_cost = move_up_left(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child

    if x > x_min and y < y_max:
        child, child_cost = move_down_right(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
        else:
            del child
    
    if x > x_min and y > y_min:
        child, child_cost = move_down_left(node)
        if not in_obstacles(child.coord):
            child_nodes.append((child, child_cost))
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
        angle_rad = np.deg2rad(90) + np.deg2rad(60 * i)  # Angle in radians for each vertex + 90 for rotating the hexagon
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

def dijkstra(game_map, start, goal):
    open_list = []
    open_list_info = {}
    closed_list = []
    closed_list_info = {}
    path = []

    # Create start node and add it to open list
    start_node = NewNode(start, None, 0)
    open_list.append((start_node, start_node.cost))
    open_list_info[start_node.coord] = start_node

    start_time = time.time()
    while open_list:

        # Get the node with the minimum cost and add to closed list
        open_list.sort(key=lambda x: x[1]) # sort open list based on cost
        node, node_cost = open_list.pop(0)
        open_list_info.pop(node.coord)
        closed_list.append(node)
        closed_list_info[node.coord] = node

        # Check if goal reached
        if node.coord == goal:
            end_time = time.time()
            print("Time taken by Dijkstra: ", end_time - start_time, " seconds")
            path = backtrack_path(node)

            return path
        
        else:
            children = get_child_nodes(node)
            for child, child_cost in children:
                if child.coord in closed_list_info.keys():
                    del child
                    continue
                else:
                    if child.coord in open_list_info.keys():
                        if child_cost + node_cost < open_list_info[child.coord].cost:
                            open_list_info[child.coord].cost = child_cost + node_cost
                            open_list_info[child.coord].parent = node
                    else:
                        child.cost = child_cost + node_cost
                        child.parent = node
                        open_list.append((child, child.cost))
                        open_list_info[child.coord] = child

    end_time = time.time()
    print("Time taken by Dijkstra: ", end_time - start_time, " seconds")
    return None

def backtrack_path(goal_node):
    # TODO: Implement function to backtrack from goal node to start node and return the path
    path = []
    parent = goal_node
    while parent!= None:
        path.append(parent.coord)
        parent = parent.parent
    return path[::-1]

def main():
    game_map = create_map()
    
    # get start and end points from user
    start_point = (int(input("Enter x coordinate of start point: ")), int(input("Enter y coordinate of start point: ")))
    goal_point = (int(input("Enter x coordinate of goal point: ")), int(input("Enter y coordinate of goal point: ")))

    # Check if start and goal points are in obstacles
    if in_obstacles(start_point):
        print("Start point is in obstacle")
        return 

    # find shortest path
    shortest_path = dijkstra(game_map, start_point, goal_point)
    if shortest_path == None:
        print("No path found")
    else:
        print("Shortest path: ", shortest_path)

    # show map
    # cv.imshow('Map', game_map)

    # wait for key press
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()