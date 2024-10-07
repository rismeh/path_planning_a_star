import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from marvelmind_interfaces.msg import HedgePos
import numpy as np
from map_parser import MapParser
from a_star import AStarAlgo
import sys
import pytest

#sys.path.append('/home/juan/ros2_foxy/src/smec_path_planning/smec_path_planning')
sys.path.append('/home/hs-coburg.de/mou0308s/ros_ws/src/smec_path_planning/smec_path_planning')
from smec_path import PublisherPath


path= PublisherPath()


test_pos_x_1=1
test_pos_y_1=2
test_start_px_1=[1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
test_start_py_1=[2, 2, 2, 2, 2, 2, 2, 2, 2, 2]

test_pos_x_2=4.8
test_pos_y_2=5.5
test_start_px_2=[1, 1, 1, 1, 1, 1, 1, 1, 1,]
test_start_py_2=[2, 2, 2, 2, 2, 2, 2, 2, 2,]

"""Test functions 'test_check_position' and 'test_check_intersection_over' """
# Test start
goal = [5.5, 2.5]
ego_position_1 = HedgePos()
ego_position_1.x_m = 0.5
ego_position_1.y_m = 2.5

ego_position_2 = HedgePos()
ego_position_2.x_m = 5.0
ego_position_2.y_m = 2.8

ego_position_3 = HedgePos()
ego_position_3.x_m = 5.6
ego_position_3.y_m = 2.5

ego_position_4 = HedgePos()
ego_position_4.x_m = -5.4
ego_position_4.y_m = 2.7

ego_position_5 = HedgePos()
ego_position_5.x_m = 3.0
ego_position_5.y_m = 2.7

ego_position_6 = HedgePos()
ego_position_6.x_m = -2.5
ego_position_6.y_m = 2.5


def test_check_goal():
    assert path.check_goal(ego_position_1, goal).data is False  # Testfall KTC500
    assert path.check_goal(ego_position_2, goal).data is True  # Testfall KTC502
    assert path.check_goal(ego_position_3, goal).data is True  # Testfall KTC501
    assert path.check_goal(ego_position_4, goal).data is False  # Testfall KTC503

intersection_coor = [3.5, 2.5]


def test_check_intersection_over():
    assert path.check_intersection_over(ego_position_1, intersection_coor).data is True  # Testfall KTC600
    assert path.check_intersection_over(ego_position_2, intersection_coor).data is False  # Testfall KTC601
    assert path.check_intersection_over(ego_position_5, intersection_coor).data is False  # Testfall KTC602
    assert path.check_intersection_over(ego_position_6, intersection_coor).data is False  # Testfall KTC603
# Test end

def test_calc_mean():
    mean_x_1,mean_y_1=path.calc_mean(test_pos_x_1,test_pos_y_1,test_start_px_1,test_start_py_1)
    mean_x_2,mean_y_2=path.calc_mean(test_pos_x_2,test_pos_y_2,test_start_px_2,test_start_py_2)

    assert mean_x_1==1
    assert mean_y_1==2
    assert mean_x_2==1.3800000000000001
    assert mean_y_2==2.35

test_point_xy_1=(0,0)
test_dist_1=[]

test_point_xy_2=(3.4,6.8)
test_dist_2=[]

def test_calc_dist():
    dist_1=path.calc_dist(test_point_xy_1,test_dist_1)
    dist_2=path.calc_dist(test_point_xy_2, test_dist_2)
    type_test=type(dist_1)
    assert dist_1==[0.7071067811865476, 
                  1.5206906325745548, 
                  2.5124689052802225, 
                  3.758324094593227, 
                  4.038873605350878, 
                  4.373213921133975, 
                  5.129571132170798, 
                  6.051859218455102, 
                  6.656763477847174, 
                  7.504165509901817, 
                  8.173891362135908, 
                  8.143248737451165, 
                  7.893826701923472, 
                  7.516648189186454, 
                  6.519202405202649, 
                  5.522680508593631, 
                  4.7762432936357, 
                  3.5355339059327378, 
                  2.3048861143232218, 
                  1.5811388300841898, 
                  4.5069390943299865, 
                  5.50567888638631, 
                  6.504805915628844, 
                  7.267220926874317, 
                  7.648529270389178, 
                  7.591113225344488, 
                  6.878408246098802, 
                  5.942432162002357, 
                  5.031152949374527, 
                  3.3634060117684275, 
                  2.704163456597992, 
                  5.367727638395972, 
                  4.981214711292819]
    assert type_test==list
    
    assert dist_2==[6.93541635375988, 
                    6.8200073313743586, 
                    6.611542936410532, 
                    6.559344479443048, 
                    5.311544031635245, 
                    4.563441683641853, 
                    3.3185087012090233, 
                    2.079663434308542, 
                    1.346291201783626, 
                    0.4609772228646443, 
                    0.7158910531638178, 
                    1.3086252328302401, 
                    2.1242645786248002, 
                    2.9832867780352594, 
                    2.9154759474226504, 
                    3.1780497164141406, 
                    3.551408171415953, 
                    4.393176527297759, 
                    5.395600059307584, 
                    6.041522986797286, 
                    6.641724173736816, 
                    6.878408246098802, 
                    7.246550903705844, 
                    7.383258088405145, 
                    6.700746227100382, 
                    5.96028522807424, 
                    5.50567888638631, 
                    5.011237372146724, 
                    4.681078935459217, 
                    4.638156961552724, 
                    4.930770730829004, 
                    2.238861317723811, 
                    2.7950849718747373]
    

test_small_dist_1=[3.8078865529319543, 
                 3.0516389039334255, 
                 2.3048861143232218, 
                 1.7677669529663689, 
                 0.5590169943749475, 
                 0.3535533905932738, 
                 1.5206906325745548, 
                 2.7613402542968153, 
                 3.5089172119045497, 
                 4.5069390943299865, 
                 5.55090082779363, 
                 5.942432162002357, 
                 6.269968101992227, 
                 6.519202405202649, 
                 5.70087712549569, 
                 4.949747468305833, 
                 4.451123453691214, 
                 3.8078865529319543, 
                 3.5089172119045497, 
                 3.5355339059327378, 
                 1.8200274723201295, 
                 2.3048861143232218, 
                 3.0516389039334255, 
                 3.5794552658190883, 
                 3.5355339059327378, 
                 3.2596012026013246, 
                 2.5124689052802225, 
                 1.5206906325745548, 
                 0.5590169943749475, 
                 1.5206906325745548, 
                 2.5124689052802225, 
                 3.132491021535417, 
                 3.7165171868296265]

test_small_dist_2= [5.522680508593631, 
                    5.942432162002357, 
                    6.269968101992227, 
                    6.864765108872991, 
                    5.8576872569299905, 
                    5.303300858899107, 
                    4.5069390943299865, 
                    3.952847075210474, 
                    3.783186487605389, 
                    3.783186487605389, 
                    3.5794552658190883, 
                    3.0516389039334255, 
                    2.3048861143232218, 
                    1.5811388300841898, 
                    0.7071067811865476, 
                    0.7071067811865476, 
                    1.346291201783626, 
                    2.5495097567963922, 
                    3.783186487605389, 
                    4.527692569068709, 
                    7.301540933255117, 
                    7.956915231419774, 
                    8.678277478854891, 
                    9.100137361600648, 
                    8.74642784226795, 
                    8.162413858657255, 
                    7.504165509901817, 
                    6.656763477847174, 
                    5.8576872569299905, 
                    4.5069390943299865, 
                    4.038873605350878, 
                    2.7950849718747373, 
                    1.9525624189766635]

def test_select_node():
    small_dist_node_1=path.select_node(test_small_dist_1)
    small_dist_node_2=path.select_node(test_small_dist_2)
    
    assert small_dist_node_1==(375, 225, 6)
    assert small_dist_node_2==(50, 650, 15)

test_start_node_1=(450,25,21)
test_end_node_1=(50,550,16)
test_start_node_2=(375,225,6)
test_end_node_2=(725,50,24)

def test_deter_path():
    path_test_1=path.deter_path(test_start_node_1,test_end_node_1)
    path_test_2=path.deter_path(test_start_node_2,test_end_node_2)
    assert path_test_1==[21,4,5,6,7,8,32,33,17,16]
    assert path_test_2==[6,5,4,21,22,23,24]


test_path_1=[21,4,5,6,7,8,32,33,17,16]
test_coord_1=[]
test_path_2=[6,5,4,21,22,23,24]
test_coord_2=[]

def test_get_coor():
    coord_test_1=path.get_coordinates(test_path_1,test_coord_1)
    coord_test_2=path.get_coordinates(test_path_2,test_coord_2)
    assert coord_test_1==[(450,25),(375,25),(375,150),(375,225),(375,350),(375,475),(250,475),(150,475),(50,475),(50,550)]
    assert coord_test_2==[(375,225),(375,150),(375,25),(450,25),(550,25),(650,25),(725,50)]



    
