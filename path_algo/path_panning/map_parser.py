"""Includes all functions to parse map data from files.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""
import os

from map_graph import MapEdge, MapGraph, MapNode


class MapParser():
    """Parse map data from a MAP.txt file."""

    def ParseMapData(self, mapFilePath) -> MapGraph:
        """Return the parsed map from the input.

        Keyword arguments:
        mapFilePath -- the full file path to the map(.txt)

        Return: the parsed map data structure
        """
        graph = MapGraph()
        with open(mapFilePath) as f:
            lines = f.readlines()
            lineNumber = 0
            for line in lines:
                data = line.split()  # Split whitespaces
                lineNumber += 1
                if len(data) == 0:
                    continue
                if data[0] == '%':
                    continue
                if data[0] == 'n':
                    # Check if the data is in the right format
                    assert len(data) == 4, 'Parsed invalid node in Line %d.\
                        Expected 4 arguments but %d were given' \
                        % (lineNumber, len(data))
                    assert data[3].isdigit(), 'Node Identifier \
                        is not an integer (Line: %d)' % lineNumber
                    assert data[1].replace('.', '', 1).replace('-', '').isdigit(),\
                        'Node x position is not an float (Line: %d)' % lineNumber
                    assert data[2].replace('.', '', 1).replace('-', '').isdigit(),\
                        'Node y position is not an float (Line: %d)' % lineNumber

                    # Convert string data to numbers
                    identifier = int(data[3])
                    xPos = float(data[1])
                    yPos = float(data[2])

                    graph.edgeDict.setdefault(identifier, [])
                    graph.nodePositionDict.setdefault(identifier, MapNode(identifier, xPos, yPos))

                    continue
                if data[0] == 'e':
                    # Check if the data is in the right format
                    assert len(data) == 5,\
                        'Parsed invalid edge in Line %d. Expected 5 arguments but %d were given'\
                        % (lineNumber, len(data))
                    assert data[1].isdigit(),\
                        'Edge start node identifier is not an integer (Line: %d)' % lineNumber
                    assert data[2].isdigit(),\
                        'Edge end node identifier is not an integer (Line: %d)' % lineNumber
                    assert data[3].isdigit(),\
                        'Edge identifier is not an integer (Line: %d)' % lineNumber
                    assert data[4].replace('.', '', 1).replace('-', '').isdigit(),\
                        'Edge cost value is not an float (Line: %d)' % lineNumber

                    # Convert string data to numbers
                    identifier = int(data[3])
                    idStartNode = int(data[1])
                    idEndNode = int(data[2])
                    cost = float(data[4])

                    assert idStartNode != idEndNode,\
                        'Edge has the same start/end node id %d (Line: %d)' % (idStartNode, lineNumber)

                    connectionList = graph.edgeDict[idStartNode]
                    connectionList.append(MapEdge(identifier, idEndNode, cost))
                    graph.edgeDict.update({idStartNode: connectionList})
                    continue

                # If it a valid map file, we should never come to this point
                assert False,\
                    'Parsed invalid map data. Line %d does not start with (e)dge or (n)ode.' % lineNumber

        return graph

    def GetDefaultMapFilepath(self, fileName):
        """Return the filepath to the example map files (/map).

        Keyword arguments:
        fileName -- the name of the file without pre- or suffix

        Return: the full path with suffix to the file
        """
        fileDir = os.path.join(os.path.dirname(os.path.realpath('__file__/')), 'map/')
        return os.path.join(fileDir, fileName + '.txt')


def main():
    """Parse an example map."""
    mapParser = MapParser()
    filepath = mapParser.GetDefaultMapFilepath('modellstadt')
    graphMap = mapParser.ParseMapData(filepath)

    print(graphMap.edgeDict)


if __name__ == '__main__':
    main()
