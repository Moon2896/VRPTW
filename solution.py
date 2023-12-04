import numpy as np
from graph import Graph
from ant import Ant

class Solution:

    def __init__(
            self,
            filePath:str,
            populationSize:int,
            maxIteration:int):

        self.graph = Graph(filePath)
        self.populationSize = populationSize
        self.maxIteration = maxIteration
        self.ants = [Ant(graph=self.graph) for _ in range(populationSize)]
        self.scores = [ant.score for ant in self.ants]
        self.bestScore = np.inf
        self.bestPath = self.ants[0].routes
    
    def update(self):
        for i, ant in enumerate(self.ants):
            ant.update()
            score = ant.score
            self.scores[i] = score
            if score < self.bestScore:
                self.bestScore = score
                self.bestPath = ant.routes
        return 0
    
    def loop(self):
        count = 0
        while count < self.maxIteration:
            self.update()
            print("Iterations: " + str(count) + '\n')
            count += 1
        return 0

    def __str__(self) -> str:
        rep = ""
        rep += "Best score: " + str(self.bestScore) + '\n'
        rep += "Best path: " + str(self.bestPath) + '\n'
        return rep