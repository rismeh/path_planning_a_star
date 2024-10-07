"""Test the map parser.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""

from map_parser import MapParser

import pytest


def test_map_parser():
    """Test if the map parser creates a valid map structure."""
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('test_map')
    graphMap = mapParser.ParseMapData(filepath)

    assert (len(graphMap.nodePositionDict) and len(graphMap.edgeDict)) == 10,\
        'Invalid map, the number of dictionary entries should be 10'

    expected_edge_ids = [0, 8, 21]
    expected_end_node_ids = [5, 7, 8]
    expected_edge_costs = [464, 1435, 343]

    edges = graphMap.edgeDict[0]

    assert len(edges) == 3,\
        'Invalid map, node 0 should have 3 outgoing edges'

    for edge in edges:
        assert edge.id >= 0,\
            'Invalid map, edge identifier is negative'
        assert edge.cost >= 0,\
            'Invalid map, edge cost is negative'
        assert edge.id in expected_edge_ids,\
            'Invalid map, edge id not found in list of node 0 edges.'
        assert edge.endNodeId in expected_end_node_ids,\
            'Invalid map, end node of edge not valid.'
        assert edge.cost in expected_edge_costs,\
            'Invalid map, edge cost invalid.'
