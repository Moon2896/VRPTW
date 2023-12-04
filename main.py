from solution import Solution
import os

if __name__=="__main__":
    filePath = "./data/r101.txt"
    populationSize = 10
    maxIteration = 10
    solution = Solution(
        filePath=filePath,
        populationSize=populationSize,
        maxIteration=maxIteration)
    print(solution)
    solution.loop()
    print(solution)