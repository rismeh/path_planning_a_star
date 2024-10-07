from math import sqrt
from map_graph import MapGraph
from map_parser import MapParser

#Load the map
mapParser = MapParser()
filepath = '/home/juan/ros2_foxy/src/smec_path_planning/path_algo/map/modellstadt_bidirektional.txt'
graphMap = mapParser.ParseMapData(filepath)

position=graphMap.nodePositionDict[]
