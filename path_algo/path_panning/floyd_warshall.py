"""Floyd Warshall algorithm for finding the shortest path on a graph map.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""

# Map data structures
from map_graph import MapGraph

# Read map files from file system
from map_parser import MapParser

import numpy as np

# works Only, if ID of nodes starting with 0 and without gaps!!!


class FloydWarshallAlgo():
    """Implementation of the floyd warshall algorithm."""

    def __init__(self):
        """Initialize the required parameters."""
        self.oldMap = None

    def constructPath(self, startNodeID, goalNodeId):
        """Get the path between the start and end node after Floyd-Matrices are calculated."""
        if (self.nextNodeMatrix[startNodeID, goalNodeId] == np.inf):
            return None
        path = [startNodeID]
        while (startNodeID != goalNodeId):
            startNodeID = int(self.nextNodeMatrix[startNodeID, goalNodeId])
            path.append(startNodeID)

        return path

    def solveAlgoritm(self, startNodeID, goalNodeId, map: MapGraph):
        """Find the shortest path between the two nodes on the given map.
        First Run with init: all Matrices are calculated, later only path is returned.

        Keyword arguments:
        startNodeId -- the identfier of the start node
        goalNodeId -- the identifier of the goal node
        map -- the map data strucuture used for path planning

        Return: None if no path was found, or a list of node identifiers
        """
        if self.oldMap == map:
            return self.constructPath(startNodeID, goalNodeId)

        numberOfNodes = len(map.nodePositionDict.keys())

        # pathDistMatrix: Matrix containing shortest Path length from node --> node
        # Example: The shortest distance found from node 3 to node 4 is entered in column 4 and line 3
        pathDistMatrix = np.matrix(np.ones((numberOfNodes, numberOfNodes)) * np.inf)
        # nextNodeMatrix: Matrix containing Next node (Direction) on shortest Path  from node --> node
        # Example: In column 4 and row 3, the first node on the shortest path from node 3 to node 4 is entered.
        self.nextNodeMatrix = np.matrix(np.ones(
            (numberOfNodes, numberOfNodes)) * np.inf)
        self.oldMap = map
        for k in range(numberOfNodes):
            # Init Matrix with known cost if edge between Nodes, no edge=inf
            if k == 0:
                for nodeId in map.edgeDict.keys():
                    pathDistMatrix[nodeId, nodeId] = 0
                    edges = map.edgeDict.get(nodeId)
                    # Loop throuh outgoing edges of this node
                    for edge in edges:
                        pathDistMatrix[nodeId, edge.endNodeId] = edge.cost
                        self.nextNodeMatrix[nodeId, edge.endNodeId] = edge.endNodeId

            # Update Matrix for each Node
            else:
                for i in range(numberOfNodes):
                    for j in range(numberOfNodes):
                        # If the path distance i -> j is greater than distance from i -> k + k -> j (bypass via k):
                        # Note: np.inf is always greater
                        if pathDistMatrix[i, j] > (pathDistMatrix[i, k] + pathDistMatrix[k, j]):
                            # save Distance of shortest path from i -> j
                            pathDistMatrix[i, j] = (pathDistMatrix[i, k] + pathDistMatrix[k, j])
                            # Save next node to array to find the nodes on the shortest path later
                            self.nextNodeMatrix[i, j] = self.nextNodeMatrix[i, k]

        return self.constructPath(startNodeID, goalNodeId)


def main():
    """Run the algorithm on a test map."""
    # Parse default map file
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('modellstadt')
    graphMap = mapParser.ParseMapData(filepath)
    FlWarshall = FloydWarshallAlgo()

    path = FlWarshall.solveAlgoritm(0, 6, graphMap)
    print(path)


if __name__ == '__main__':
    main()
