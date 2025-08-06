import numpy as np
from obstacle_class import Obstacle
from tether_update import distance, tether_update,tether_winding_angle,tether_not_cross_winding,tether_not_cross_with_obstacle,get_midpoint
from shapely.geometry import Polygon, LineString, Point
from queue import PriorityQueue 
from tangent_node import tangent_node_, get_tangent_node,obstacle,get_goal_
import copy as cp
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as P
from matplotlib.patches import Circle
from shapely import is_simple
from shapely.affinity import scale
import time

class Robot_pair:
        def __init__(self,tangent_node1, tangent_node2, tether_config, parent = None):
            self.tangent_node1 = tangent_node1
            self.tangent_node2 = tangent_node2
            self.tether_config = tether_config
            self.parent = parent
            self.num = 2
            
      
def g_cost(tangent_node, transit, neighbor):
    r = 1
    if type(tangent_node.center[0]) != list:
        r = 1
        v1 = np.array(tangent_node.point) - np.array(tangent_node.center)
        v2 = np.array(transit) - np.array(tangent_node.center)
        if np.linalg.norm(v1)*np.linalg.norm(v2) != 0:
            dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
            if dot_product > 1:
                dot_product = 1
            arc = np.arccos(dot_product)
            cost = abs(arc*r) + distance(transit, neighbor)
            return cost     
        else:
            return 0   
    if type(tangent_node.center[0]) == list: 
        if abs(distance(transit, tangent_node.center[0]) - r) < 0.01:
            center = tangent_node.center[0]
            r = 1
            v1 = np.array(tangent_node.point) - np.array(center)
            v2 = np.array(transit) - np.array(center)
            if np.linalg.norm(v1)*np.linalg.norm(v2) != 0:
                dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
                if dot_product > 1:
                    dot_product = 1
                arc = np.arccos(dot_product)
                cost = abs(arc*r) + distance(transit, neighbor)
                return cost  
        if abs(distance(transit, tangent_node.center[1]) - r) < 0.01:
            center = tangent_node.center[1]
            r = 1
            v1 = np.array(tangent_node.point) - np.array(center)
            v2 = np.array(transit) - np.array(center)
            if np.linalg.norm(v1)*np.linalg.norm(v2) != 0:
                dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
                if dot_product > 1:
                    dot_product = 1
                arc = np.arccos(dot_product)
                cost = abs(arc*r) + distance(transit, neighbor)
                return cost 
            
            
        
def heuristic_cost(current_node, goal):
    return distance(current_node, goal)

def list2tuple(tether_config):
    tuplelist = []
    for i in tether_config:
        tuplelist.append(tuple(np.round(i,2)))
    return tuple(tuplelist)



def winding_constraint_check(winding_angle,winding_constraint):
    return max(winding_constraint - winding_angle,0)

    
def robot_direction_check(neighbor1, neighbor2, transit1, transit2, tether_config):
    if neighbor1 == [] or None:
        return False
    if neighbor2 == [] or None:
        return False
    if transit1 == None and transit2 == None:
        return True
    current_transit1 = transit1
    current_transit2 = transit2
    new_transit1 = neighbor1[0]
    new_transit2 = neighbor2[0]
    current_robot1 = tether_config[0]
    current_robot2 = tether_config[-1]
    new_neighbor1 = neighbor1[1]
    new_neighbor2 = neighbor2[1]
    vect1 = np.array(current_robot1) - np.array(current_transit1)
    vect1_ = np.array(new_transit1) - np.array(current_robot1)
    vect1__ = np.array(new_neighbor1) - np.array(new_transit1)
    vect2 = np.array(current_robot2) - np.array(current_transit2)
    vect2_ = np.array(new_transit2) - np.array(current_robot2)
    vect2__ = np.array(new_neighbor2) - np.array(new_transit2)
    if np.linalg.norm(vect1) != 0:
        if np.linalg.norm(vect1_) == 0:
            vect1_ = vect1
        if np.linalg.norm(vect1__) == 0:
            vect1__ = vect1
        dot1 = np.dot(vect1, vect1_)/(np.linalg.norm(vect1)*np.linalg.norm(vect1_))
        dot1_ = np.dot(vect1_, vect1__)/(np.linalg.norm(vect1_)*np.linalg.norm(vect1__)) 
        if dot1 < 0 or dot1_ < 0:
            return False            

    if np.linalg.norm(vect2) != 0:
        if np.linalg.norm(vect2_) == 0:
            vect2_ = vect2
        if np.linalg.norm(vect2__) == 0:
            vect2__ = vect2
        dot2 = np.dot(vect2, vect2_)/(np.linalg.norm(vect2)*np.linalg.norm(vect2_))
        dot2_ = np.dot(vect2_, vect2__)/(np.linalg.norm(vect2_)*np.linalg.norm(vect2__)) 
        if dot2 < 0 or dot2_ < 0:
            return False 
        
    if np.linalg.norm(vect1) == 0:
        if np.linalg.norm(vect1_) != 0:
            if np.linalg.norm(vect1__) != 0:
                dot1_ = np.dot(vect1_, vect1__)/(np.linalg.norm(vect1_)*np.linalg.norm(vect1__)) 
                if dot1_ < 0:
                    return False  
                
    if np.linalg.norm(vect2) == 0:
        if np.linalg.norm(vect2_) != 0:
            if np.linalg.norm(vect2__) != 0:
                dot2_ = np.dot(vect2_, vect2__)/(np.linalg.norm(vect2_)*np.linalg.norm(vect2__)) 
                if dot2_ < 0:
                    return False 
        

    return True

# def reform(tether_config,obstacle_points):
#     # print(tether_config)
#     r = 1
#     first = tether_config[0]
#     second = tether_config[-1]
#     middle = tether_config[1:-1]
#     # print(first,second,middle)
#     for point in obstacle_points:
#         if abs(distance(point,first) - r) < 0.001:
#             # print(first, point)
#             first = point
#         if abs(distance(point,second) - r) < 0.001:
#             second = point
#         # if distance(point,first) - r < 0.001 or distance(point,first) - r > -0.001:
#         #     # print(first, point)
#         #     first = point
#         # if abs(distance(point,second) - r) < 0.001 or distance(point,first) - r > -0.001:
#             second = point
#     return [first,*middle,second]  

def winding_Astar(start_pos,goal_pos,tether_config,obstacle, winding_constraint_range,omega,step, length_limit):
    boundary = 10000
    winding_constraint = winding_constraint_range[0]
    winding_constraint_upper_limit = winding_constraint_range[1]
    obstacle_points = []
    for i in obstacle:
        for j in i:
            obstacle_points.append(j)   
    start1 = start_pos[0]
    start2 = start_pos[1]
    goal1 = goal_pos[0]
    goal2 = goal_pos[1]
    tangent_node1 = tangent_node_(start1,None,goal1,start1,obstacle)
    tangent_node2 = tangent_node_(start2,None,goal2,start2,obstacle)
    
    tag = 0
    iteration = 0
    openlist = PriorityQueue()
    close_list = {}
    close_list[list2tuple(tether_config)] = 0
    cost_so_far = {}
    first_robot_pair = Robot_pair(tangent_node1, tangent_node2, tether_config, parent = None)
    first_robot_pair.winding = 0
    first_robot_pair.cost = 0
    first_robot_pair.transit1 = start1[0:2]
    first_robot_pair.transit2 = start2[0:2]
    openlist.put((0,0,tag,first_robot_pair))
    solution_candidate = []
    
    while not openlist.empty():
        info = openlist.get()
        index = info[0]
        total_path_cost = info[1]
        if total_path_cost >= boundary:
            continue
        iteration += 1        
        print(iteration)        
        current_robot_pair = info[3]
        robot1 = current_robot_pair.tangent_node1
        robot2 = current_robot_pair.tangent_node2
        tether_config = current_robot_pair.tether_config
        winding = current_robot_pair.winding
        current_cost_to_go = current_robot_pair.cost
        transit1 = current_robot_pair.transit1   
        transit2 = current_robot_pair.transit2 
        path_cost = current_robot_pair.cost

        # fig,ax = plt.subplots()
        # ax.set_xlim([-5,30])
        # ax.set_ylim([-5,30])
        # # plt.title("Winding Constraint: "+ str(np.round(winding_constraint,3))+" rad")
        # for i in obstacle:
        #     p = P(i, facecolor = 'k')
        #     ax.add_patch(p)
        # for point in obstacle_points:
        #     circle = Circle((point[0],point[1]), 1,color = "grey", fill = False, linewidth = 0.3)
        #     ax.add_patch(circle) 
        # plt.plot([i[0] for i in tether_config],[i[1] for i in tether_config],'r')
        # print(total_path_cost)
        # plt.show()
        # time.sleep(1)
        # plt.close()

        neighbor_list1 = robot1.neighbor
        neighbor_list2 = robot2.neighbor
        for neighbor1 in neighbor_list1:
            for neighbor2 in neighbor_list2: 
                if neighbor1 != [] and neighbor2 != []:#
                    if robot_direction_check(neighbor1, neighbor2, current_robot_pair.transit1, current_robot_pair.transit2, tether_config) == True:
                        path_cost = current_cost_to_go + g_cost(robot1,neighbor1[0], neighbor1[1]) + g_cost(robot2,neighbor2[0], neighbor2[1])                    
                        new_tether_config1, length1 = tether_update([robot1.point, robot2.point], [neighbor1[0], neighbor2[0]],tether_config, obstacle)  
                        new_tether_config, length = tether_update([neighbor1[0], neighbor2[0]], [neighbor1[1], neighbor2[1]],new_tether_config1, obstacle)  
                        new_tether_config_reform = [[round(elem, 3) for elem in sublist] for sublist in new_tether_config]
                        # new_tether_config_reform = reform(new_tether_config,obstacle_points)
                        new_winding = tether_winding_angle(new_tether_config)                            
                        if tether_not_cross_winding(new_tether_config, tether_config) == True and tether_not_cross_with_obstacle(new_tether_config,obstacle) ==True: 
                            if length <= length_limit:
                                    if list2tuple(new_tether_config_reform) not in close_list or path_cost < close_list[list2tuple(new_tether_config_reform)]:    
                                            close_list[list2tuple(new_tether_config_reform)] = path_cost       
                                            new_tangent_node1 = tangent_node_(neighbor1,robot1.point,goal1,start1,obstacle)
                                            new_tangent_node2 = tangent_node_(neighbor2,robot2.point,goal2,start2,obstacle)
                                            New_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair)                            
                                            New_robot_pair.winding =  new_winding
                                            New_robot_pair.cost = path_cost
                                            New_robot_pair.transit1 = neighbor1[0]
                                            New_robot_pair.transit2 = neighbor2[0]


                                            if distance(neighbor1[1], goal1[0:2]) < 0.01 and distance(neighbor2[1], goal2[0:2]) < 0.01 and new_winding > winding_constraint and new_winding < winding_constraint_upper_limit: 
                                                solution_candidate.append(New_robot_pair)
                                                if omega == 0:
                                                    print("solution found")
                                                    print(path_cost)
                                                    print(iteration)
                                                    return solution_candidate
                                                boundary = path_cost
                                                omega = max(0, omega - step)
                                                print("solution found")
                                                print(path_cost)
  
                                                if openlist.empty():
                                                    return solution_candidate
                                                switching_list = PriorityQueue()
                                                while not openlist.empty():
                                                    info = openlist.get()
                                                    total_cost = info[0]
                                                    total_path_cost = info[1]
                                                    current_robot_pair = info[3]
                                                    tether_config = current_robot_pair.tether_config

                                                    winding_angle = tether_winding_angle(tether_config)
                                                    winding_cost = winding_constraint_check(winding_angle,winding_constraint)
                                                    switching_list.put((total_path_cost + omega*winding_cost, total_path_cost, tag, current_robot_pair))
                                                    tag = tag + 1
                                               
                                                openlist = switching_list
                                            else:
                                                winding_cost_bias = winding_constraint_check(new_winding,winding_constraint) 
                                                total_path_cost = path_cost + distance(neighbor1[1], goal1[0:2]) + distance(neighbor2[1], goal2[0:2])
                                                tag = tag+1
                                            
                                                if total_path_cost < boundary:
                                                    openlist.put((omega*winding_cost_bias+total_path_cost,total_path_cost,tag,New_robot_pair)) 
        

    if solution_candidate == []:
        return False
    else:
        print("already optimal")
        print(iteration)
        return solution_candidate

def is_the_input_valid(start_pos,goal_pos,obstacle,length_limit,r):
    obstacle_points = []
    obstacle_polygon = []
    for i in obstacle:
        obstacle_polygon.append(Polygon(i))
        for j in i:
            obstacle_points.append(j)
    start1 = start_pos[0][0:2]
    start2 = start_pos[1][0:2]  
    goal1 = goal_pos[0][0:2]
    goal2 = goal_pos[1][0:2]
    if start_pos[0][2] < 0 or start_pos[0][2] > np.pi:
        return False
    if start_pos[1][2] < 0 or start_pos[1][2] > np.pi:
        return False
    if goal_pos[0][2] < 0 or goal_pos[0][2] > np.pi:
        return False
    if goal_pos[0][2] < 0 or goal_pos[0][2] > np.pi:
        return False
    # start and goal orientation between (-pi, pi)
    check_list = [start1,start2,goal1,goal2]
    for point in check_list:
        p = Point(point)
        for obs in obstacle_points:
            if distance(point, obs) < r:
                return False
        for poly in obstacle_polygon:
            if poly.intersects(p) == True:
                return False            
    # The start and goal positions are not too close to the obstacle vertices and not crossing the obstacles
    if distance(start1,start2) > length_limit or distance(goal1,goal2) > length_limit:
        return False   
    #length limit check
    x_max = max([p[0] for p in obstacle_points])
    x_min = min([p[0] for p in obstacle_points])
    y_max = max([p[1] for p in obstacle_points])
    y_min = min([p[1] for p in obstacle_points])
    if start1[0]<= x_max and start1[1] > x_min and start1[1] <= y_max and start1[1] >= y_min:
        return False
    if start2[0]<= x_max and start2[1] > x_min and start2[1] <= y_max and start2[1] >= y_min:
        return False
    if goal1[0]<= x_max and goal1[1] > x_min and goal1[1] <= y_max and goal1[1] >= y_min:
        return False
    if  goal2[0]<= x_max and  goal2[1] > x_min and goal2[1] <= y_max and goal2[1] >= y_min:
        return False
    return True





