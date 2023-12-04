import numpy as np
from graph import Graph

class Ant:

    # an ant is a collection of solutions (multiple vehicles)

    def __init__(
            self,
            graph:Graph): # graph composed of nodes and their infos
        
        self.graph = graph
        self.startNode = self.graph.startNode
        self.vehicleLoads = [0 for _ in range(self.graph.vehicleNumber)]
        
        # the matrix is (k, n, n) where k-th (n,n) mat is the path for truck k
        self.routes = np.zeros((self.graph.vehicleNumber,self.graph.nodeNumber,self.graph.nodeNumber))
        
        self.P = self.graph.probabilityMatrix
        self.score = 1_000_000_000

        self.rho = self.graph.rho

    def routeConstruction(self):
        # left over nodes
        nodesAvailable = [i for i in range(self.graph.nodeNumber)]
        # tI is the i-th truck, tLoad refoers to its load
        for tI, _ in enumerate(self.vehicleLoads):
            # iterate trough its path (0->...->0)
            # memory wise inefficient
            currentNode = self.startNode
            # let the current node not be choosable
            nodesAvailable.pop(np.argmax([currentNode==node for node in self.graph.nodes]))
            print(f"Truck {tI}")

            for nodeIndex in [np.argmax([currentNode==node for node in self.graph.nodes]) in range(self.graph.nodeNumber)]:
                # get the current nodeIndex and check if capacity is ok
                if self.vehicleLoads[tI]+self.graph.nodes[nodeIndex].demand > self.graph.capacity:
                    # choose the next one
                    nextNode = np.random.choice(
                        a = nodesAvailable,
                        p = self.P[tI][nodeIndex] 
                    )
                    # add the currentNode demand
                    self.vehicleLoads[tI] += self.graph.nodes[nodeIndex].demand
                    # write the path
                    self.routes[tI][currentNode][nextNode] = 1
                    # change to the next node
                    currentNode = nextNode
                    print(f"Going to: {nodeIndex}")

            # ugly code
            self.routes[tI][currentNode][np.argmax([currentNode==node for node in self.graph.nodes])]

        return 0
    
    def computeDeltaPheromons(self):
        dP = np.zeros_like(self.P)
        for route in self.routes:
            dP += np.multiply(route, self.graph.pheromonsMatrix)
        dP *= self.rho
        return dP
    
    def computePheromons(self):
        dP = self.computeDeltaPheromons()
        self.graph.pheromonsMatrix = (1-self.graph.rho)*self.graph.pheromonsMatrix + self.graph.rho*dP
        print(np.sum(dP))
    
    def update(self):
        # the order does not really matter has it is a +/-1 from the real sol.
        # just the score should relate to the sol and be calculated last
        self.computePheromons()
        self.routeConstruction()
        self.score = np.sum(self.routes*self.graph.distanceMatrix)
    # TEMA LA TAILLE DU PYTHON