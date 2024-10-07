"""Test the path planning algorithms.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""

from a_star import AStarAlgo

from dijkstra import DijkstraAlgo

from floyd_warshall import FloydWarshallAlgo

from map_parser import MapParser

import pytest


def test_dijkstra():
    """Test if dijkstra algorithm is return the correct path."""
    # Setup the test map
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('test_map')
    graphMap = mapParser.ParseMapData(filepath)

    dijkstra = DijkstraAlgo()
    path = dijkstra.solveAlgoritm(2, 6, graphMap)

    expectedPath = [2, 5, 1, 6]

    assert (expectedPath == path),\
        'Path is not equal to the expected path.'


def test_a_start():
    """Test if a star algorithm is return the correct path."""
    # Setup the test map
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('test_map')
    graphMap = mapParser.ParseMapData(filepath)

    aStar = AStarAlgo()

    path = aStar.solveAlgoritm(2, 6, graphMap)

    expectedPath = [2, 5, 1, 6]

    assert (expectedPath == path),\
        'Path is not equal to the expected path.'


def test_floyd_warshall():
    """Test if floyd warshall algorithm is return the correct path."""
    # Setup the test map
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('test_map')
    graphMap = mapParser.ParseMapData(filepath)

    floyd_warshall = FloydWarshallAlgo()

    path = floyd_warshall.solveAlgoritm(2, 6, graphMap)

    expectedPath = [2, 5, 1, 6]

    assert (expectedPath == path),\
        'Path is not equal to the expected path.'
