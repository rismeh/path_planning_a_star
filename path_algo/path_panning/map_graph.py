"""Data structures for defining a map.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""


class MapGraph():
    """Represenation of a graph based map."""

    def __init__(self):
        """Initialize nodes and edges."""
        self.edgeDict = {}
        self.nodePositionDict = {}


class MapNode:
    """Representation of a map node."""

    def __init__(self, identifier, xPos, yPos):
        """Initialize node values."""
        self.id = identifier
        self.xPos = xPos
        self.yPos = yPos

    def __repr__(self):
        """Get the map node as a string."""
        return ('({0},{1},{2})'.format(self.id, self.xPos, self.yPos))


class MapEdge:
    """Representation of a map edge."""

    def __init__(self, identifier, endNodeId, cost):
        """Initialize edge values."""
        self.id = identifier
        self.endNodeId = endNodeId
        self.cost = cost

    def __repr__(self):
        """Get the map edge as a string."""
        return ('({0},{1},{2})'.format(self.id, self.endNodeId, self.cost))
