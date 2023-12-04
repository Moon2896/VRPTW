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
        nodesAvailable, nodesIndexAvailable, node2Col = self.initializeNodes()

        P = self.P.copy()

        for tI, tLoad in enumerate(self.vehicleLoads):
            currentNode = nodesIndexAvailable.index(self.startNode)
            toVisit, toVisitIndex = nodesAvailable.copy(), nodesIndexAvailable.copy()

            while tLoad < self.graph.capacity or len(toVisit) == 0:
                currentNodeIndex = node2Col[currentNode]
                self.printCurrentNodeInfo(currentNode, currentNodeIndex)

                nextNodeIndex = self.chooseNextNodeIndex(currentNodeIndex, toVisitIndex, P)
                nextNodeIndexInList = toVisitIndex.index(nextNodeIndex)
                nextNode, nextNodeDemand = toVisit[nextNodeIndexInList], nextNodeIndex.demand

                if self.isDemandSatisfied(nextNodeDemand, tLoad):
                    self.handleSatisfiedDemand(tI, node2Col, nextNodeDemand, currentNode, tLoad, P, toVisitIndex, toVisit, nextNodeIndex)
                else:
                    self.handleUnsatisfiedDemand(toVisitIndex, toVisit, nextNodeIndex)

        return 0

    def initializeNodes(self):
        nodesAvailable = self.graph.nodes.copy()
        nodesIndexAvailable = [i for i in range(self.graph.nodeNumber)]
        node2Col = {node_idx: col_idx for col_idx, node_idx in enumerate(nodesIndexAvailable)}
        return nodesAvailable, nodesIndexAvailable, node2Col

    def printCurrentNodeInfo(self, currentNode, currentNodeIndex):
        print("currentNode:", currentNode)
        print("currentNodeIndex:", currentNodeIndex)

    def chooseNextNodeIndex(self, currentNodeIndex, toVisitIndex, P):
        print(toVisitIndex)
        print(currentNodeIndex)
        nextNodeIndex = np.random.choice(
            a=toVisitIndex,
            p=P[currentNodeIndex] / np.sum(P[currentNodeIndex])
        )
        return toVisitIndex.index(nextNodeIndex)

    def isDemandSatisfied(self, nextNodeDemand, tLoad):
        return nextNodeDemand + tLoad < self.graph.capacity

    def handleSatisfiedDemand(self, tI, node2Col, nextNodeDemand, currentNode, tLoad, P, toVisitIndex, toVisit, nextNodeIndex):
        del node2Col[currentNode]
        tLoad += nextNodeDemand

        currentNodeIndex = node2Col[currentNode]
        P = np.delete(P, currentNodeIndex, axis=1)

        nextNodeIndexInList = toVisitIndex.index(nextNodeIndex)
        toVisitIndex.pop(nextNodeIndexInList)
        toVisit.pop(nextNodeIndexInList)

        self.routes[tI][currentNodeIndex][nextNodeIndex] = 1
        currentNode = nextNodeIndex  # Fix: Use nextNodeIndex instead of nextNode
        print(f"Switch to new node {nextNodeIndex}" + '\n')

    def handleUnsatisfiedDemand(self, toVisitIndex, toVisit, nextNodeIndex):
        nextNodeIndexInList = toVisitIndex.index(nextNodeIndex)
        toVisitIndex.pop(nextNodeIndexInList)
        toVisit.pop(nextNodeIndexInList)
        print('Trying another node' + '\n')
    
    def computeDeltaPheromons(self):
        dPh = np.zeros_like(self.P)
        for route in self.routes:
            dPh += np.multiply(route, self.graph.pheromonsMatrix)
        dPh *= self.rho
        return dPh
    
    def computePheromons(self):
        dPh = self.computeDeltaPheromons()
        self.graph.pheromonsMatrix = (1-self.graph.rho)*self.graph.pheromonsMatrix + self.graph.rho*dPh
        self.P = self.graph.computeProbabilityMatrix()
        print(f"dPh={np.sum(dPh)}")
    
    def update(self):
        # the order does not really matter has it is a +/-1 from the real sol.
        # just the score should relate to the sol and be calculated last
        self.computePheromons()
        self.routeConstruction()
        self.score = np.sum(self.routes*self.graph.distanceMatrix)
    # TEMA LA TAILLE DU PYTHON