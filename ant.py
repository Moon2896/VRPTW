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
        nodesAvailable, nodesIndexAvailable, node2Col, col2Node = self.initializeNodes()

        P = self.P.copy()

        for tI, tLoad in enumerate(self.vehicleLoads):
            currentNode = self.graph.nodes[0]
            toVisit, toVisitIndex = nodesAvailable.copy(), nodesIndexAvailable.copy()

            while tLoad < self.graph.capacity or len(toVisit) != 0:
                currentNodeIndex = node2Col[currentNode]
                self.printCurrentNodeInfo(currentNode, currentNodeIndex)

                nextNodeIndex = self.chooseNextNodeIndex(currentNodeIndex, toVisitIndex, P)
                nextNode = col2Node[nextNodeIndex]
                nextNodeDemand = nextNode.demand

                if self.isDemandSatisfied(nextNodeDemand, tLoad):
                    P, currentNode, currentNodeIndex, tLoad, toVisit, toVisitIndex = self.handleSatisfiedDemand(tI, currentNodeIndex, node2Col,col2Node, nextNode, nextNodeDemand, currentNode, tLoad, P, toVisitIndex, toVisit, nextNodeIndex)
                else:
                    self.handleUnsatisfiedDemand(toVisitIndex, toVisit, nextNodeIndex)

        return 0

    def initializeNodes(self):
        nodesAvailable = self.graph.nodes
        nodesIndexAvailable = [i for i in range(self.graph.nodeNumber)]
        node2Col = {node: col_idx for col_idx, node in enumerate(nodesAvailable)}
        col2Node = {v:k for k, v in node2Col.items()}
        
        # print(node2Col)
        return nodesAvailable, nodesIndexAvailable, node2Col, col2Node

    def printCurrentNodeInfo(self, currentNode, currentNodeIndex):
        print("currentNode:", currentNode)
        print("currentNodeIndex:", currentNodeIndex)

    def chooseNextNodeIndex(self, currentNodeIndex, toVisitIndex, P):
        print("toVisitIndex:",toVisitIndex)
        # print("currentNodeIndex:",currentNodeIndex)
        print("P:",P[currentNodeIndex].shape)
        nextNodeIndex = np.random.choice(
            a=toVisitIndex,
            p=P[currentNodeIndex] / np.sum(P[currentNodeIndex])
        )
        return nextNodeIndex

    def isDemandSatisfied(self, nextNodeDemand, tLoad):
        return nextNodeDemand + tLoad < self.graph.capacity

    def handleSatisfiedDemand(self, tI, currentNodeIndex, node2Col, col2Node, nextNode, nextNodeDemand, currentNode, tLoad, P, toVisitIndex, toVisit, nextNodeIndex):
        # Add the path to routes[tI][currentNode][nextNode]
        # remove inf that we don't need anymore

        tLoad += nextNodeDemand
        print("P.shape before:",P.shape)
        print("removing column:",currentNodeIndex)
        P = np.delete(P, currentNodeIndex, axis=1)
        print("P.shape after:",P.shape)

        tv = []
        for node in toVisit:
            if node != nextNode:
                tv.append(node)
        toVisit = tv
        
        tvi = []
        for nodeI in toVisitIndex:
            if nodeI != nextNodeIndex:
                tvi.append(nodeI)
        toVisitIndex = tvi
        # P = np.delete(P, nextNodeIndex, axis=1)

        currentNode = col2Node[nextNodeIndex]
        currentNodeIndex = nextNodeIndex
        
        self.routes[tI][currentNodeIndex][nextNodeIndex] = 1
        print(f"Switch to new node {nextNodeIndex}" + '\n')

        return P, currentNode, currentNodeIndex, tLoad, toVisit, toVisitIndex

    def handleUnsatisfiedDemand(self, toVisitIndex, toVisit, nextNodeIndex):
        nextNodeIndexInList = toVisitIndex.index(nextNodeIndex)
        toVisitIndex.pop(nextNodeIndexInList)
        toVisit.pop(nextNodeIndexInList)
        print('Trying another node ############################################' + '\n')
    
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