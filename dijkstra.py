"""
dijkstra.py

@breif:     This module implements Dijkstra's algorithm for finding the shortest path in a graph.
@author:    Kshitij Karnawat
@date:      5th March 2024
@version:   1.0
"""

class NewNode:
    def __init__(self, coord, parent, cost):
        self.coord = coord
        self.parent = parent
        self.cost = cost
    
def move_up(node):
    x, y = node.coord
    return NewNode((x, y + 1), node, node.cost + 1)

def move_down(node):
    x, y = node.coord
    return NewNode((x, y - 1), node, node.cost + 1)

def move_left(node):
    x, y = node.coord
    return NewNode((x - 1, y), node, node.cost + 1)

def move_right(node):
    x, y = node.coord
    return NewNode((x + 1, y), node, node.cost + 1)

def move_up_left(node):
    x, y = node.coord
    return NewNode((x - 1, y + 1), node, node.cost + 1.4)

def move_up_right(node):
    x, y = node.coord
    return NewNode((x + 1, y + 1), node, node.cost + 1.4)

def move_down_left(node):
    x, y = node.coord
    return NewNode((x - 1, y - 1), node, node.cost + 1.4)

def move_down_right(node):
    x, y = node.coord
    return NewNode((x + 1, y - 1), node, node.cost + 1.4)

