#!/usr/bin/env python

import rospy

import roslib

import math

import numpy as np 

from collections import defaultdict

import string

import re

import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

robot_x = 0.0

robot_y = 0.0

robot_orientation_new = 0.0

def construct_path(camefrom,current):

    total_path = []

    while current in camefrom.keys():
        current = camefrom[current]
        total_path.append(current)

    return total_path


def minimum_of_openset(fscore,openset):
    
    minimum = []
    for i in range(len(openset)):
        minimum.append(fscore[openset[i]])
    
    return openset[minimum.index(min(minimum))]

def find_distance(start,end,alphabets):

    x = re.findall('.',start)
    y = re.findall('.',end)

    x1 = alphabets.index(x[0])
    y1 = alphabets.index(x[1])

    x2 = alphabets.index(y[0])
    y2 = alphabets.index(y[1])

    distance = math.sqrt((x1 - x2) ** 2 + (y1-y2) ** 2)

    return distance

def implement_a_star(grid_to_graph,start,end,alphabets):

    closedset = []
    openset =[start]
    camefrom = defaultdict(lambda: float('-inf'))

    gscore = defaultdict(lambda: float('-inf'))
    gscore[start] = 0

    fscore = defaultdict(lambda: float('-inf'))
    fscore[start] = find_distance(start,end,alphabets)

    #print(fscore[start])

    while openset:

        current = minimum_of_openset(fscore,openset)

        openset.remove(current)
        closedset.append(current)

        if current == end:
            
            return construct_path(camefrom,current)


        for neighbour in grid_to_graph[current]:

            if neighbour in closedset:
                continue

            tentative_g_score = gscore[current] + find_distance(current,neighbour,alphabets)
            
            if neighbour not in openset:
                openset.append(neighbour)
            elif tentative_g_score >= gscore[neighbour]:
                continue

            camefrom[neighbour] = current
            gscore[neighbour] = tentative_g_score
            #print(grid_to_graph[current,neighbour])
            fscore[neighbour] = gscore[neighbour] + grid_to_graph[current,neighbour]
    #print(closedset)   

def is_diagonal_line_between_obstacles(grid,x,y):

    if x+1 != 20:
        if grid[x,y-1] == 1 and grid[x+1,y] == 1:
            #print(x,y)
            return False
        else:
            return True

def covert_shortest_path_to_robotic_world(path,alphabets):

    coorinates_in_robotic_world = []
    for i in path:

         value = re.findall('.',i)
         
         x = alphabets.index(value[1]) 
         y = alphabets.index(value[0]) + 1 

         r_x = x - 9
         r_y = 10 - y

         coorinates_in_robotic_world.append([r_x,r_y])   
    



    return coorinates_in_robotic_world

def odomfunction(data):

    #print("Good")
   
    global robot_x,robot_y,robot_orientation_new

    robot_x = data.pose.pose.position.x
	
    robot_y = data.pose.pose.position.y

    orientation =(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)

    robot_orientation = euler_from_quaternion(orientation)

    robot_orientation_new = robot_orientation[2]  	

    print(robot_x,robot_y,robot_orientation_new)
			 
    
     			
def distance(path_position):

    global robot_x, robot_y

    return math.sqrt((path_position[0] - robot_x) ** 2 + (path_position[1] - robot_y) ** 2)   

def getEndPosition(endx,endy,alphabets):

    x = int(abs(endx + 9)) 
     
    y = int(abs(10 - endy))

    return ([[y,x]],alphabets[y] + alphabets[x])


if __name__ == '__main__':

    

    rospy.init_node('astar')

    subscriber = rospy.Subscriber("base_pose_ground_truth", Odometry, odomfunction)	

    map = np.array([0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1])

    grid = np.reshape(map,(20,18))

    grid_to_graph = defaultdict(list)

    rows,columns = grid.shape

    not_required_values = []

    endx = rospy.get_param('goalx')

    endy = rospy.get_param('goaly')

    

    				

    for i in range(rows):

        not_required_values.append(str(i) + '18')

    for i in range(columns):

        not_required_values.append('20'+ str(i))

    not_required_values.append('2018')

    alphabets = list(string.ascii_lowercase)
    
    end,endposition = getEndPosition(endx,endy,alphabets)
   
    for i in range(rows):
        for j in range(columns):

            edge_detection = np.array([str(i-1)+str(j-1), str(i-1)+str(j), str(i-1)+str(j+1), str(i)+str(j-1), str(i)+str(j+1), str(i+1)+str(j-1), str(i+1)+str(j), str(i+1)+str(j+1)])
            weight_to_edges = [[i-1,j-1],[i-1,j],[i-1,j+1],[i,j-1],[i,j+1],[i+1,j-1],[i+1,j],[i+1,j+1]]

            for k in range(len(edge_detection)):

                if '-' not in edge_detection[k]:

                    if edge_detection[k] not in not_required_values:

                        main_node = alphabets[i] + alphabets[j]

                        x = weight_to_edges[k][0]

                        y = weight_to_edges[k][1]

                        neighbour_node = alphabets[x] + alphabets[y]

                        isdiagonal = is_diagonal_line_between_obstacles(grid,x,y)

                        if grid[i,j] == 0 and grid[x,y] == 0 and isdiagonal:

                            grid_to_graph[main_node].append(neighbour_node)

                            grid_to_graph[main_node,neighbour_node] = math.sqrt((x - end[0][0]) ** 2 + (y - end[0][1]) ** 2) 

    start = 'lb'

    path = implement_a_star(grid_to_graph,start,endposition,alphabets)

    if path is not None:	

    	path.insert(0,endposition)



 	grid_in_robotic_world = covert_shortest_path_to_robotic_world(path,alphabets)

    	print(grid_in_robotic_world)	

    	global robot_x,robot_y,robot_orientation_new    
    
    	message_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    	while grid_in_robotic_world:

    		path_position = grid_in_robotic_world.pop()

    		orientation_goal = math.atan2((path_position[1] - robot_y),(path_position[0] - robot_x))	
		msg = Twist()

		print(path_position)
	
	
		while abs(robot_x - path_position[0]) > 0.30  or abs(robot_y - path_position[1]) > 0.5:

		
    		
			msg.linear.x = 1 * distance(path_position)  
			
			msg.angular.z = 24 * (orientation_goal - robot_orientation_new)

			message_pub.publish(msg)

		
		msg.linear.x = 0 

		message_pub.publish(msg)
    
        
    	msg.linear.x = 2.0

    	message_pub.publish(msg)

    else:

	print("End goal cannot be reached from Start Point")		

    
    
