import numpy as np

class Node:

    def __init__(
            self,
            x:float,            # position
            y:float, 
            demand:float,       # cargo
            readyTime:float,    # time at which the delivery can be done
            dueTime:float,      # time at which the delivery cannot be done anymore
            serviceTime:float): # time the delivery takes
        
        self.x = x
        self.y = y
        self.demand = demand
        self.readyTime = readyTime
        self.dueTime = dueTime
        self.serviceTime = serviceTime
        self.startNode = self.demand == 0

class Graph:

    def __init__(self, filePath:str) -> None:
        # load nodes and params from the files
        self.vehicleNumber, self.capacity, self.nodes \
            = self.fromFile(filePath) 
        
        self.startNode = self.getStartNode(self.nodes)
        
        self.nodeNumber = len(self.nodes)
        
        # get the distance matrix
        self.distanceMatrix = self.computeDistanceMatrix(self.nodes)
        # print(self.distanceMatrix)

        # get a first pass on pheromons
        self.pheromonsMatrix = np.ones((self.nodeNumber, self.nodeNumber))/self.nodeNumber
        # print(self.pheromonsMatrix)

        # evaporation rate quick:1 > rho > slow:0
        self.rho = 0.05

        # get the time-heuristic
        self.timeMatrix = self.computeTimeMatrix(self.nodes)
        print(self.timeMatrix)

        # initialize alpha  : weight of the pheromone
        # initialize beta   : weight of the heuristic
        # initialize gamma  : weight of the time depedency
        self.alpha = 1
        self.beta = 1
        self.gamma = 2

        self.probabilityMatrix = self.computeProbabilityMatrix()

    
    def fromFile(self, filePath:str):
        # nodes will be stored in a list
        nodes = []
        with open(filePath) as file:
            for i, line in enumerate(file):
                # header of the document
                # print(i)
                # print("start line")
                # print(line)
                # print("end line")
                if i == 4:
                    vehicleNumber, capacity = line.split()[0], line.split()[1]
                    vehicleNumber, capacity = int(vehicleNumber), int(capacity)
                
                # info about each node
                elif i > 10:
                    nodeParameters = [float(x) for x in line.split()]
                    newNode = Node(
                        x=nodeParameters[1],
                        y=nodeParameters[2],
                        demand=nodeParameters[3],
                        readyTime=nodeParameters[4],
                        dueTime=nodeParameters[5],
                        serviceTime=nodeParameters[6],
                    )
                    nodes.append(newNode)
        return vehicleNumber, capacity, nodes
    
    def computeDistanceMatrix(self, nodes:Node):
        D = np.zeros((self.nodeNumber,self.nodeNumber))
        # compute the first upper right hand corner
        for i, nodeI in enumerate(nodes):
            for j, nodeJ in enumerate(nodes[i:]):
                # distance of the arc <i,j>
                D[i][j] = self.computeDistance(nodeI, nodeJ)
        D = D + D.T
        return D

    def computeDistance(self, nodeI:Node, nodeJ:Node):
        return np.sqrt((nodeI.x-nodeJ.x)**2 + (nodeI.y-nodeJ.y)**2)
    
    # note that at the end, we compute a probability, the lower the less probable
    # we are to choose this node. Thus, the time heuristic should be 0 if there
    # is no value to go there, 1 if very urgent. Prioritizing a node over an other:
    # - dueTime is past
    # - probably something else
    def computeTimeMatrix(self, nodes:list[Node]):
        T = np.ones((self.nodeNumber,self.nodeNumber))
        for i, nodeI in enumerate(nodes):
            for j, nodeJ in enumerate(nodes):
                if not (nodeI.readyTime + nodeI.serviceTime > nodeJ.dueTime):
                    T[i][j] = np.max([T[i][j], nodeJ.readyTime-nodeI.readyTime])
            T[i] = T[i]/(np.sum(T[i])+1)                
        return T
    
    def computeProbabilityMatrix(self):
        P = np.ones((self.nodeNumber,self.nodeNumber))
        for i in range(self.nodeNumber):
            for j in range(self.nodeNumber):
                P[i][j] *= ( 1/(self.distanceMatrix[i][j]**self.beta+1) \
                        + self.pheromonsMatrix[i][j]**self.alpha \
                        + self.timeMatrix[i][j]**self.gamma )
            # normalize the columns
            P[i] *= 1/(np.sum(P[i])+1)
        print(P)
        return P

    def getStartNode(self, nodes:list[Node]):
        for node in nodes:
            if node.startNode:
                return node
        return nodes[0]
    
