"""Test the performance of the implemented algorithms.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""

import random
from statistics import mean, stdev
from time import perf_counter  # for performance measurement

# Algorithms
from a_star import AStarAlgo

from dijkstra import DijkstraAlgo

from floyd_warshall import FloydWarshallAlgo

# Read map files from file system
from map_parser import MapParser


class PerformanceMeasurement:
    """Measure the performance of a given function."""

    @staticmethod
    def measureExecutionTime(func, *args, **kwargs):
        """Report the execution time of the function.

        Keyword arguments:
        func -- the function to be measured
        *args -- input arguments for the function
        **kwargs -- input arguments for the function

        Return: result of the function, execution time
        """
        start = perf_counter()
        result = func(*args, **kwargs)
        end = perf_counter()
        deltaTime = ((end - start) * 1000000)
        return result, round(deltaTime, 1)


class TestAlgorithms:
    """Test class for measuring the performance of all algorithms."""

    def __init__(self, graphMap, seed=None):
        """Initialize required variables.

        Keyword arguments:
        graphMap -- the map data structure for path planning
        seed -- the seed for determining start and end nodes
        """
        self.aStarSolver = AStarAlgo()
        self.dijkstraSolver = DijkstraAlgo()
        self.warshallSolver = FloydWarshallAlgo()

        self.graphMap = graphMap
        self.seed = seed

        # Override random values
        self.startNodeOverride = -1
        self.endNodeOverride = -1

    def setNodeIdOverride(self, startNode, endNode):
        """Override the start and end node.

        Keyword arguments:
        startNode -- the new start node
        endNode -- the new end node
        """
        self.startNodeOverride = startNode
        self.endNodeOverride = endNode

    def getRandomNodeIDs(self):
        """Return random ids if they arent overwritten.

        Note: If the node override values aren't set to negative one
        it will always return startNodeOverride, endNodeOverride

        Return startNodeID, endNodeID
        """
        if self.startNodeOverride != -1 and self.endNodeOverride != -1:
            return self.startNodeOverride, self.endNodeOverride

        random.seed(self.seed)  # used for generating nodes

        # Determine start and end node
        startNodeIndex = random.randint(0, len(self.graphMap.nodePositionDict) - 1)
        endNodeIndex = random.randint(0, len(self.graphMap.nodePositionDict) - 1)

        # Get the node ids
        startNodeId = self.graphMap.nodePositionDict[startNodeIndex].id
        endNodeId = self.graphMap.nodePositionDict[endNodeIndex].id

        return startNodeId, endNodeId

    def testPathfindingPerformance(self, repetitions=100, multiplePaths=False):
        """Run the performance test for the algorithms.

        Keyword arguments:
        repetitions -- the number of runs for the test
        multiplePaths -- if true, the test will choose random paths for each run
        """
        # Perform Measurement
        aStarTimeList = []
        aDijkstraTimeList = []
        aWarshallTimeList = []

        if not multiplePaths:
            startNode, endNode = self.getRandomNodeIDs()

        for i in range(1, repetitions + 1):
            if multiplePaths:
                startNode, endNode = self.getRandomNodeIDs()

            # Run A Star, measure performance
            _, elapsedTime = PerformanceMeasurement.measureExecutionTime(
                self.aStarSolver.solveAlgoritm,
                startNode, endNode, self.graphMap)
            aStarTimeList.append(elapsedTime)

            # Run Dijkstra, measure performance
            _, elapsedTime = PerformanceMeasurement.measureExecutionTime(
                self.dijkstraSolver.solveAlgoritm,
                startNode, endNode, self.graphMap)
            aDijkstraTimeList.append(elapsedTime)

            # Run Floyd Warshall, measure performance
            _, elapsedTime = PerformanceMeasurement.measureExecutionTime(
                self.warshallSolver.solveAlgoritm, startNode, endNode, self.graphMap)
            aWarshallTimeList.append(elapsedTime)

            # Print update each 1000 iterations
            if (i % 1000 == 0 and i != 0) or i == repetitions:
                print(f"""Processed {i} of {repetitions}...""")

        # Print results to the console
        print(f"""Timing results in micro seconds with {repetitions} repetitions:""")
        if not multiplePaths:
            print(f"""Start node: {startNode} --> End node: {endNode}""")
        print(f"""AStar results, Average: {mean(aStarTimeList)} , Std. Dev: {stdev(aStarTimeList)}
            - min:{min(aStarTimeList)}, max: {max(aStarTimeList)}""")
        print(f"""Dijkstra results, Average: {mean(aDijkstraTimeList)} , Std. Dev: {stdev(aDijkstraTimeList)}
            - min:{min(aDijkstraTimeList)}, max: {max(aDijkstraTimeList)}""")
        print(f"""Warshall results, Average: {mean(aWarshallTimeList)} , Std. Dev: {stdev(aWarshallTimeList)}
            - min:{min(aWarshallTimeList)}, max: {max(aWarshallTimeList)}, first run: {aWarshallTimeList[0]}""")
        aWarshallTimeList.pop(0)
        print(f"""Warshall results without 1st, Average: {mean(aWarshallTimeList)} , Std. Dev: {stdev(aWarshallTimeList)}
            - min:{min(aWarshallTimeList)}, max: {max(aWarshallTimeList)}""")


def main():
    """Measure the runtime of the implemented algorithms."""
    # Parse default map file
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('modellstadt')
    graphMap = mapParser.ParseMapData(filepath)

    # Test the algorithms with the parsed map (optionally set a random seed)
    testAlgos = TestAlgorithms(graphMap, seed=None)
    # testAlgos.setNodeIdOverride(0, 7) # override random values, so all test will run between start and end
    testAlgos.testPathfindingPerformance(multiplePaths=False, repetitions=10000)


if __name__ == '__main__':
    main()
