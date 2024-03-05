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
    
