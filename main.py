from solution import Solution
import os

if __name__=="__main__":
    filePath = "./data/r101.txt"
    populationSize = 3
    maxIteration = 5
    maxGraphSize = 10
    solution = Solution(
        filePath=filePath,
        populationSize=populationSize,
        maxIteration=maxIteration,
        maxGraphSize=maxGraphSize)
    # print(solution)
    solution.loop()
    print(solution)