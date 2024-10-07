"""A Star algorithm for finding the shortest path on a graph map.


Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""
# Math functions
from math import sqrt

# Map data structures
from map_graph import MapGraph

# Read map files from file system
from map_parser import MapParser


class AStarAlgo():
    """Implementation of the a star algorithm."""

    class Node:
        """Node parameters for algorithm."""

        def __init__(self, id: int, parentId: int):
            """Initialize the parameters."""
            self.id = id
            self.parent = parentId
            self.g = 0  # Distance to start node
            self.h = 0  # Distance to goal node
            self.f = 0  # Total cost (Sum of Dist to start Node + estimation Dist to goal node)

        def __lt__(self, other):
            """Get node order."""
            return self.f < other.f

        def __eq__(self, other):
            """Check if the nodes are equal."""
            return self.id == other.id

    def addToPriorityQueue(self, priorityQueue, neighbor):
        """Check if the node is in the unchecked nodes.

        Keyword arguments:
        priorityQueue -- the list of nodes in the priority queue
        neighbor -- the neighbor node

        Return: true if the neibhbor is in priority queue
        """
        for node in priorityQueue:
            if (neighbor == node and neighbor.f > node.f):
                return False
        return True

    def solveAlgoritm(self, startNodeId, goalNodeId, map: MapGraph):
        """Find the shortest path between the two nodes on the given map.

        Keyword arguments:
        startNodeId -- the identfier of the start node
        goalNodeId -- the identifier of the goal node
        map -- the map data strucuture used for path planning

        Return: None if no path was found, or a list of node identifiers
        """
        # If Node has been processed, It will get one of the following states
        fullyProcessed = []  # Shortest Path from the starting node is known
        priorityQueue = []  # A Path from the starting node is known but it is not neccesarily the shortest

        # Init Nodes of wanted path (Start --> End). They won't have Parents
        startNode = self.Node(startNodeId, None)
        goalNode = self.Node(goalNodeId, None)

        goalNodePosition = map.nodePositionDict[goalNodeId]

        # Calculate heuristics/costs as estimation of Distance to target/goal node, safe in dict
        costEstimationDict = {}

        # Loop trough each node in map
        for nodeId in map.nodePositionDict.keys():
            if nodeId == startNodeId:
                costEstimationDict.setdefault(nodeId, 0)
                continue

            positionNode = map.nodePositionDict[nodeId]
            euclDistToGoal = sqrt(
                (goalNodePosition.xPos - positionNode.xPos)**2 +
                (goalNodePosition.yPos - positionNode.yPos)**2)
            costEstimationDict.setdefault(nodeId, euclDistToGoal)

        priorityQueue.append(startNode)

        while len(priorityQueue) > 0:
            priorityQueue.sort()
            currentNode = priorityQueue.pop(0)
            fullyProcessed.append(currentNode)

            if currentNode == goalNode:
                path = []
                while currentNode != startNode:
                    path.append(currentNode.id)
                    currentNode = currentNode.parent
                path.append(startNode.id)
                # Return reversed path
                return path[::-1]

            edges = map.edgeDict.get(currentNode.id)
            # Loop neighbors
            for edge in edges:
                neighbor = self.Node(edge.endNodeId, currentNode)
                if(neighbor in fullyProcessed):
                    continue

                neighbor.g = currentNode.g + abs(edge.cost)
                neighbor.h = costEstimationDict.get(neighbor.id)
                neighbor.f = neighbor.g + neighbor.h

                if(self.addToPriorityQueue(priorityQueue, neighbor) is True):
                    priorityQueue.append(neighbor)

        return None


def main():
    """Run the algorithm on a test map."""
    # Parse default map file
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('modellstadt_bidirektional')
    graphMap = mapParser.ParseMapData(filepath)

    #AStar = AStarAlgo()
    #path = AStar.solveAlgoritm(7, 17, graphMap)
    #print(path)


if __name__ == '__main__':
    main()
