import numpy as np
from graph import Graph

class Ant:
    def __init__(self, graph=Graph,start_node=0):
        self.graph = graph
        self.vehicle_load = 0
        self.current_node = start_node
        self.visited_nodes = [start_node] 
        self.route = [start_node]

    def route_construction(self):
        # Implement the route construction logic here
        pass

    def move_to_next_node(self,next_node):
        self.route.append(next_node)
        self.visited_nodes.append(next_node)
        
        distance = self.graph.distanceMatrix[self.current_node][next_node]
        
        self.vehicle_load += self.graph.nodes[next_node].demand
        
        #How do we know if we are at the depot/end of path?
        
                
        # Implement the move to next node logic here
        self.current_node = next_node

    def insert_on_path(self, node):
        # Implement the insertion logic here
        pass