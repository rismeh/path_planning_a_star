"""Module provides entrypoint in a ROS system for communication"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from marvelmind_interfaces.msg import HedgePos
import numpy as np
from map_parser import MapParser
from a_star import AStarAlgo
from std_msgs.msg import Bool
#Inizalizations
startx=[]
starty=[]
dist_start=[]
dist_end=[]
path_coord=[]
epsilon = 0.5  
#Coordinates
nodes =[
    (50, 50, 1),
    (150, 25, 2),
    (250, 25, 3),
    (375, 25, 4),
    (375, 150, 5),
    (375, 225, 6),
    (375, 350, 7),
    (375, 475, 8),
    (375, 550, 9),
    (375, 650, 10),
    (325, 750, 11),
    (250, 775, 12),
    (150, 775, 13),
    (50, 750, 14),
    (50, 650, 15),
    (50, 550, 16),
    (50, 475, 17),
    (50, 350, 18),
    (50, 225, 19),
    (50, 150, 20),
    (450, 25, 21),
    (550, 25, 22),
    (650, 25, 23),
    (725, 50, 24),
    (750, 150, 25),
    (725, 225, 26),
    (650, 225, 27),
    (550, 225, 28),
    (450, 225, 29),
    (250, 225, 30),
    (150, 225, 31),
    (250, 475, 32),
    (150, 475, 33)
]
intersection_coor = [3.5, 2.5]

class PublisherPath(Node):
    """This class represents the information that is published"""
    def __init__(self):
        #rclpy.init()
        super().__init__('minimal_publisher')
        self.declare_parameter('ziel_x')
        self.declare_parameter('ziel_y')
        #Input Parameter
        self.end_node_x=self.get_parameter('ziel_x').get_parameter_value().double_value
        self.end_node_y=self.get_parameter('ziel_y').get_parameter_value().double_value
        #Subscription creation
        self.subscriptionp= self.create_subscription(HedgePos,'hedge_pos',
                                                     self.listener_callback_p,10)
        #Publishers creation
        self.publishergoal_=self.create_publisher(Point, 'goal_nodes',10)
        self.publisherstart_=self.create_publisher(Point,'start_node',10)
        self.publisherend_=self.create_publisher(Point,'end_node',10)
        self.publisher_goal_flag = self.create_publisher(Bool, 'goal', 10)
        self.publisher_intersection_flag = self.create_publisher(Bool, 'intersection_over', 10)

    def listener_callback_p(self,msg):
        """This function is the one that executes the calculations,
        and it is linked to the data coming from marvelmind"""
        #check if goal reached and publish flag
        self.publisher_goal_flag.publish(self.check_goal(msg, [self.end_node_x, self.end_node_y]))
        #check if the intersection is crossed
        self.publisher_intersection_flag.publish(self.check_intersection_over(msg, intersection_coor))

        #Calculation of x and y coordinates for SMEC start point
        mean=self.calc_mean(msg.x_m,msg.y_m,startx,starty)
        print('Das ist der Startpunkt: ',np.array(mean))
        print('Der Zielpunkt ist: ',np.array([self.end_node_x,self.end_node_y]))
        #Calculation distance between mean (x,y) and all the node
        dist_node_start= self.calc_dist(mean,dist_start)
        #Selection of the small distance Node for start
        start_node= self.select_node(dist_node_start)
        print('Der Startnode ist: ',start_node)
        #Calculation distance between goalx and goaly and all the nodes
        goal=(self.end_node_x,self.end_node_y)
        dist_node_end=self.calc_dist(goal,dist_end)
        #Selection of the small distance Node for start
        end_node=self.select_node(dist_node_end)
        print('Der Zielnode ist: ',end_node)
        #Calculation the path using a star algo
        path=self.deter_path(start_node,end_node)
        print('Der Pfad zum Ziel ist: ',path)
        #Extraction of the values of the coordinates form the path in cm
        coord=self.get_coordinates(path,path_coord)
        print(len(coord))
        print('Die Koordinaten des Zielpfades sind: ',coord, type(coord))
        #Publish Information of the coordinates
        self.publi_coord(coord)

    def check_goal(self, msg, target):
        """This function checkes if the ego position defined by msg.x_m and msg.y_m 
        is close enough to the target position """
        result = Bool()
        x_ego = msg.x_m
        y_ego = msg.y_m

        try:
            if x_ego < 0 or y_ego < 0:
                raise ValueError("Negative distance value detected.")
        
            if abs(x_ego - target[0]) <= epsilon and abs(y_ego - target[1]) <= epsilon:
                result.data = True
            else:
                result.data = False
        except ValueError as e:
            print(f"Error: {e}")
            result.data = False

        print(f'Goal {target} reached? {result.data}')
        return result
    
    def check_intersection_over(self,msg, target):
        """This function checkes if the intersection is corssed"""
        result = Bool()
        x_ego = msg.x_m
        y_ego = msg.y_m

        try:
            if x_ego < 0 or y_ego < 0:
                raise ValueError("Negative distance value detected.")
            if abs(y_ego - target[1]) <= epsilon and x_ego < target[0] - epsilon:
                result.data = True
            else:
                result.data = False
        except ValueError as e:
            print(f"Error: {e}")
            result.data = False

        print(f'Intersection {target} crossed? {result.data}')
        return result

    def calc_mean(self,pos_x,pos_y,start_px,start_py):
        """This function calculates the average values of the first 10 values from marvelmind."""
        #Safe the first 10 values of x_m and y_m
        if len(start_px)<10 and len(start_py)<10:
            start_px.append(pos_x)
            start_py.append(pos_y)
        if len(start_px)==10 and len(start_py)==10:
            np.array(start_px)
            np.array(start_py)
            mean_x= np.mean(np.array(start_px))
            mean_y= np.mean(np.array(start_py))
            return mean_x, mean_y
    def calc_dist(self,point_xy,dist):
        """This function calculates the distances from a point to all nodes in a list"""
        start_p = np.array(point_xy)
        for node in nodes:
            node_xy= np.array(node[:2])/100
            try:
                distance = np.linalg.norm(node_xy - start_p)
                dist.append(distance)
            except TypeError:
                continue
        return dist
    def select_node(self, dist_array):
        """This function selects the smallest value in a list."""
        if dist_array:
            dist_min = dist_array.index(min(dist_array))
            small_dist_node = nodes[dist_min]
            return small_dist_node
    def deter_path(self, node_start, node_end):
        """This function determines the nodes that must be traversed to reach the destination."""
        #Load the map
        mapParser = MapParser()
        filepath = mapParser.GetDefaultMapFilepath('modellstadt_bidirektional')
        graphMap = mapParser.ParseMapData(filepath)
        try:
            AStar = AStarAlgo()
            path = AStar.solveAlgoritm(node_start[2], node_end[2], graphMap)
        except TypeError:
            path=[]
        return path
    def get_coordinates(self,path_array,coord_array):
        """This function determines the coordinates of the nodes 
        that must be traversed to reach the destination."""
        while len(path_array)!=len(coord_array):
            for path in path_array:
                for node in nodes:
                    if path==node[2]:
                        coord_array.append(node[:2])
        return coord_array
    def publi_coord(self,data):
        """This function constructs the messages to be published."""
        if data:
            start_x=data[0][0]
            start_y=data[0][1]
            size=len(data)-1
            end_x=data[size][0]
            end_y=data[size][1]
            for point in data:
                point_msg = Point()
                point_msg.x = float(point[0])
                point_msg.y = float(point[1])
                point_msg.z = 0.0
                self.publishergoal_.publish(point_msg)
            start_msg=Point()
            start_msg.x=float(start_x)
            start_msg.y=float(start_y)
            start_msg.z=0.0
            self.publisherstart_.publish(start_msg)
            end_msg=Point()
            end_msg.x=float(end_x)
            end_msg.y=float(end_y)
            end_msg.z=0.0
            self.publisherend_.publish(end_msg)

def main(args=None):
    rclpy.init(args=args)
    publisher_path = PublisherPath()
    rclpy.spin(publisher_path)
    publisher_path.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
