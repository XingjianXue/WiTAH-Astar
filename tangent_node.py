import numpy as np
from obstacle_class import Obstacle
from tether_update import distance
from shapely.geometry import Polygon, LineString, Point
import copy as cp


# change the size and locations of obstacles here
obstacle = [[[6, 5], [12,5], [12,10], [6.1,10]],
            [[15, 17], [20, 17.5], [19, 22], [15, 22]],
            [[15.9,9.5],[18,6],[19.6,12],[17,13], [15.2, 12]],
            [[2,17],[6,16],[6.7,21],[5,23], [1.9,19]]]


def get_tangent_node(point1,point2,angle_range1,angle_range2):
    margin = 0.01
    r = 1
    neighbor_list = []
    vector_point_to_point = np.array(point2) - np.array(point1)
    angle = np.arctan2(vector_point_to_point[1],vector_point_to_point[0])
    
    if angle_range1 == [-np.pi, np.pi]:
        vertical_vect_t1 = np.array([r*np.cos(angle+np.pi/2), r*np.sin(angle+np.pi/2)])
        vertical_vect_t1 = np.round(vertical_vect_t1,4)
        vertical_vect_n1 = vertical_vect_t1
        t1 = vertical_vect_t1 + np.array(point1)
        n1 = vertical_vect_n1 + np.array(point2)
        angle_t1 = np.arctan2(vertical_vect_t1[1], vertical_vect_t1[0])
        angle_n1 = np.arctan2(vertical_vect_n1[1], vertical_vect_n1[0])

                
        if max(angle_range2) == np.pi:
            if angle_n1 == -np.pi:
                angle_n1 = np.pi
                
        if min(angle_range2) == -np.pi:
            if angle_n1 == np.pi:
                angle_n1 = -np.pi 
        

        if np.max(angle_range2) - np.min(angle_range2) < np.pi:
            if angle_n1 <= np.max(angle_range2) + margin and angle_n1 >= np.min(angle_range2) - margin:
                neighbor_list.append([list(t1),list(n1)])                                               
        if np.max(angle_range2) - np.min(angle_range2) > np.pi:
            if angle_n1 >= np.max(angle_range2) - margin and angle_n1 <= np.pi + margin or angle_n1 >= -np.pi - margin and angle_n1 <= np.min(angle_range2) + margin:
                neighbor_list.append([list(t1),list(n1)]) 
                                          
                    
                
        vertical_vect_t2 = np.array([r*np.cos(angle-np.pi/2), r*np.sin(angle-np.pi/2)])
        vertical_vect_t2 = np.round(vertical_vect_t2, 4)
        vertical_vect_n2 = vertical_vect_t2
        t2 = vertical_vect_t2 + np.array(point1)
        n2 = vertical_vect_n2 + np.array(point2)
        angle_t2 = np.arctan2(vertical_vect_t2[1], vertical_vect_t2[0])
        angle_n2 = np.arctan2(vertical_vect_n2[1], vertical_vect_n2[0])
        
        if max(angle_range2) == np.pi:
            if angle_n2 == -np.pi:
                angle_n2 = np.pi
                
        if min(angle_range2) == -np.pi:
            if angle_n2 == np.pi:
                angle_n2 = -np.pi 
                
        if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_n2 <= np.max(angle_range2) + margin and angle_n2 >= np.min(angle_range2) - margin:
                    neighbor_list.append([list(t2),list(n2)]) 

        if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_n2 >= np.max(angle_range2)- margin and angle_n2 <= np.pi+margin or angle_n2 >= -np.pi-margin and angle_n2 <= np.min(angle_range2)+margin:
                    neighbor_list.append([list(t2),list(n2)])
                                    
        l = distance(point1,point2)/2
        h_sq = l**2 - r**2
        if h_sq < 0:
            h_sq = 0     
        h = np.sqrt(h_sq)
        # print(h)
        # if h_sq < 0:
        #     h_sq = 0 
        theta = np.arctan2(h,r)
        vertical_vect_t3 = np.array([r*np.cos(angle+theta), r*np.sin(angle+theta)])
        vertical_vect_n3 =  -vertical_vect_t3
        t3 = vertical_vect_t3 + np.array(point1)
        n3 = vertical_vect_n3 + np.array(point2)
        angle_t3 = np.arctan2(vertical_vect_t3[1], vertical_vect_t3[0])
        angle_n3 = np.arctan2(vertical_vect_n3[1], vertical_vect_n3[0])
        if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_n3 <= np.max(angle_range2) + margin and angle_n3 >= np.min(angle_range2) - margin:
                    neighbor_list.append([list(t3),list(n3)])

        if np.max(angle_range2) - np.min(angle_range2) > np.pi:
            if angle_n3 >= np.max(angle_range2) - margin and angle_n3 <= np.pi + margin or angle_n3 >= -np.pi - margin and angle_n3 <= np.min(angle_range2) + margin:
                    neighbor_list.append([list(t3),list(n3)])
                
        vertical_vect_t4 = np.array([r*np.cos(angle-theta), r*np.sin(angle-theta)])
        vertical_vect_n4 = -vertical_vect_t4
        t4 = vertical_vect_t4 + np.array(point1)
        n4 = vertical_vect_n4 + np.array(point2)
        angle_t4 = np.arctan2(vertical_vect_t4[1], vertical_vect_t4[0])
        angle_n4 = np.arctan2(vertical_vect_n4[1], vertical_vect_n4[0])
        if np.max(angle_range2) - np.min(angle_range2) < np.pi:
            if angle_n4 <= np.max(angle_range2) + margin and angle_n4 >= np.min(angle_range2) - margin:
                neighbor_list.append([list(t4),list(n4)])

        if np.max(angle_range2) - np.min(angle_range2) > np.pi:
            if angle_n4 >= np.max(angle_range2)-margin  and angle_n4 <= np.pi + margin or angle_n4 >= -np.pi - margin and angle_n4 <= np.min(angle_range2) + margin:
                neighbor_list.append([list(t4),list(n4)])
                        
   
    if angle_range1 != [-np.pi, np.pi]:  
        # print(angle)
        vertical_vect_t1 = np.array([r*np.cos(angle+np.pi/2), r*np.sin(angle+np.pi/2)])
        vertical_vect_t1 = np.round(vertical_vect_t1,4)
        vertical_vect_n1 = vertical_vect_t1
        # print(vertical_vect_t1)
        t1 = vertical_vect_t1 + np.array(point1)
        n1 = vertical_vect_n1 + np.array(point2)
        angle_t1 = np.arctan2(vertical_vect_t1[1], vertical_vect_t1[0])
        angle_n1 = np.arctan2(vertical_vect_n1[1], vertical_vect_n1[0])
        
        if max(angle_range1) == np.pi:
            if angle_t1 == -np.pi:
                angle_t1 = np.pi
                
        if min(angle_range1) == -np.pi:
            if angle_t1 == np.pi:
                angle_t1 = -np.pi  
                
                
        if max(angle_range2) == np.pi:
            if angle_n1 == -np.pi:
                angle_n1 = np.pi
                
        if min(angle_range2) == -np.pi:
            if angle_n1 == np.pi:
                angle_n1 = -np.pi 
        # print(angle_range1, angle_range2, angle_t1, angle_n1)
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t1 <= np.max(angle_range1)+margin and angle_t1 >= np.min(angle_range1)-margin:
                    if angle_n1 <= np.max(angle_range2)+margin and angle_n1 >= np.min(angle_range2)-margin:
                        neighbor_list.append([list(t1),list(n1)]) 
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t1 >= np.max(angle_range1) - margin and angle_t1 <= np.pi + margin or angle_t1 >= -np.pi - margin and angle_t1 <= np.min(angle_range1) + margin:
                    if angle_n1 <= np.max(angle_range2)+margin and angle_n1 >= np.min(angle_range2)-margin:
                        neighbor_list.append([list(t1),list(n1)]) 
                        
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t1 <= np.max(angle_range1) + margin and angle_t1 >= np.min(angle_range1) - margin:
                    if angle_n1 >= np.max(angle_range2) - margin and angle_n1 <= np.pi + margin or angle_n1 >= -np.pi - margin and angle_n1 <= np.min(angle_range2) + margin:
                        neighbor_list.append([list(t1),list(n1)]) 
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t1 >= np.max(angle_range1) - margin and angle_t1 <= np.pi + margin or angle_t1 >= -np.pi- margin and angle_t1 <= np.min(angle_range1)+ margin:
                    if angle_n1 >= np.max(angle_range2)- margin and angle_n1 <= np.pi+ margin or angle_n1 >= -np.pi- margin and angle_n1 <= np.min(angle_range2)+ margin:
                        neighbor_list.append([list(t1),list(n1)])                   
                    
        # print(angle) 
        # print(np.cos(angle-np.pi/2))   
        # print(np.sin(angle-np.pi/2))   
        vertical_vect_t2 = np.array([r*np.cos(angle-np.pi/2), r*np.sin(angle-np.pi/2)])
        vertical_vect_t2 = np.round(vertical_vect_t2,4)
        vertical_vect_n2 = vertical_vect_t2

        # print(vertical_vect_n2)
        t2 = vertical_vect_t2 + np.array(point1)
        n2 = vertical_vect_n2 + np.array(point2)

        angle_t2 = np.arctan2(vertical_vect_t2[1], vertical_vect_t2[0])
        angle_n2 = np.arctan2(vertical_vect_n2[1], vertical_vect_n2[0])
        
        if max(angle_range1) == np.pi:
            if angle_t2 == -np.pi:
                angle_t2 = np.pi
                
        if min(angle_range1) == -np.pi:
            if angle_t2 == np.pi:
                angle_t2 = -np.pi  
                
                
        if max(angle_range2) == np.pi:
            if angle_n2 == -np.pi:
                angle_n2 = np.pi
                
        if min(angle_range2) == -np.pi:
            if angle_n2 == np.pi:
                angle_n2 = -np.pi 
                
                 

        
        # print(angle_range1, angle_range2, angle_t2, angle_n2)
        
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t2 <= np.max(angle_range1)+ margin and angle_t2 >= np.min(angle_range1) -margin:
                    if angle_n2 <= np.max(angle_range2)+ margin and angle_n2 >= np.min(angle_range2) - margin:
                        neighbor_list.append([list(t2),list(n2)])
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t2 >= np.max(angle_range1)- margin and angle_t2 <= np.pi +margin or angle_t2 >= -np.pi - margin and angle_t2 <= np.min(angle_range1)+margin:
                    if angle_n2 <= np.max(angle_range2) + margin and angle_n2 >= np.min(angle_range2) - margin:
                        neighbor_list.append([list(t2),list(n2)])
                        
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t2 <= np.max(angle_range1)+margin and angle_t2 >= np.min(angle_range1) - margin:
                    if angle_n2 >= np.max(angle_range2)-margin and angle_n2 <= np.pi+margin or angle_n2 >= -np.pi-margin and angle_n2 <= np.min(angle_range2)+margin:
                        neighbor_list.append([list(t2),list(n2)])
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t2 >= np.max(angle_range1)-margin and angle_t2 <= np.pi+margin or angle_t2 >= -np.pi-margin and angle_t2 <= np.min(angle_range1)+margin:
                    if angle_n2 >= np.max(angle_range2)-margin and angle_n2 <= np.pi+margin or angle_n2 >= -np.pi-margin and angle_n2 <= np.min(angle_range2)+margin:
                        neighbor_list.append([list(t2),list(n2)])    
        
        
        l = distance(point1,point2)/2
        h_sq = l**2 - r**2
        if h_sq < 0:
            h_sq = 0     
        h = np.sqrt(h_sq)
        theta = np.arctan2(h,r)
        vertical_vect_t3 = np.array([r*np.cos(angle+theta), r*np.sin(angle+theta)])
        vertical_vect_t3 = np.round(vertical_vect_t3,4)
        vertical_vect_n3 =  -vertical_vect_t3
        t3 = vertical_vect_t3 + np.array(point1)
        n3 = vertical_vect_n3 + np.array(point2)
        angle_t3 = np.arctan2(vertical_vect_t3[1], vertical_vect_t3[0])
        angle_n3 = np.arctan2(vertical_vect_n3[1], vertical_vect_n3[0])
        
        if max(angle_range1) == np.pi:
            if angle_t3 == -np.pi:
                angle_t3 = np.pi
                
        if min(angle_range1) == -np.pi:
            if angle_t3 == np.pi:
                angle_t3 = -np.pi  
                
                
        if max(angle_range2) == np.pi:
            if angle_n3 == -np.pi:
                angle_n3 = np.pi
                
        if min(angle_range2) == -np.pi:
            if angle_n3 == np.pi:
                angle_n3 = -np.pi
        
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t3 <= np.max(angle_range1)+margin and angle_t3 >= np.min(angle_range1)-margin:
                    if angle_n3 <= np.max(angle_range2)+margin and angle_n3 >= np.min(angle_range2)-margin:
                        neighbor_list.append([list(t3),list(n3)])
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t3 >= np.max(angle_range1)-margin and angle_t3 <= np.pi+margin or angle_t3 >= -np.pi-margin and angle_t3 <= np.min(angle_range1)+margin:
                    if angle_n3 <= np.max(angle_range2)+margin and angle_n3 >= np.min(angle_range2)-margin:
                        neighbor_list.append([list(t3),list(n3)])
                        
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t3 <= np.max(angle_range1)+margin and angle_t3 >= np.min(angle_range1)-margin:
                    if angle_n3 >= np.max(angle_range2) - margin and angle_n3 <= np.pi+margin or angle_n3 >= -np.pi-margin and angle_n3 <= np.min(angle_range2)+margin:
                        neighbor_list.append([list(t3),list(n3)])
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t3 >= np.max(angle_range1)-margin and angle_t3 <= np.pi+margin or angle_t3 >= -np.pi-margin and angle_t3 <= np.min(angle_range1)+margin:
                    if angle_n3 >= np.max(angle_range2) - margin and angle_n3 <= np.pi+margin or angle_n3 >= -np.pi-margin and angle_n3 <= np.min(angle_range2)+margin:
                        neighbor_list.append([list(t3),list(n3)]) 
                
        vertical_vect_t4 = np.array([r*np.cos(angle-theta), r*np.sin(angle-theta)])
        vertical_vect_t4 = np.round(vertical_vect_t4,4)
        vertical_vect_n4 = -vertical_vect_t4
        t4 = vertical_vect_t4 + np.array(point1)
        n4 = vertical_vect_n4 + np.array(point2)
        angle_t4 = np.arctan2(vertical_vect_t4[1], vertical_vect_t4[0])
        angle_n4 = np.arctan2(vertical_vect_n4[1], vertical_vect_n4[0])
        
        if max(angle_range1) == np.pi:
            if angle_t4 == -np.pi:
                angle_t4 = np.pi
                
        if min(angle_range1) == -np.pi:
            if angle_t4 == np.pi:
                angle_t4 = -np.pi  
                
                
        if max(angle_range2) == np.pi:
            if angle_n4 == -np.pi:
                angle_n4 = np.pi
                
        if min(angle_range2) == -np.pi:
            if angle_n2 == np.pi:
                angle_n2 = -np.pi
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t4 <= np.max(angle_range1)+margin and angle_t4 >= np.min(angle_range1)-margin:
                    if angle_n4 <= np.max(angle_range2)+margin and angle_n4 >= np.min(angle_range2)-margin:
                        neighbor_list.append([list(t4),list(n4)])
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) < np.pi:
                if angle_t4 >= np.max(angle_range1)-margin and angle_t4 <= np.pi+margin or angle_t4 >= -np.pi-margin and angle_t4 <= np.min(angle_range1)+margin:
                    if angle_n4 <= np.max(angle_range2)+margin and angle_n4 >= np.min(angle_range2)-margin:
                        neighbor_list.append([list(t4),list(n4)])
                        
        if np.max(angle_range1) - np.min(angle_range1) < np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t4 <= np.max(angle_range1)+margin and angle_t4 >= np.min(angle_range1)-margin:
                    if angle_n4 >= np.max(angle_range2)-margin and angle_n4 <= np.pi+margin or angle_n4 >= -np.pi-margin and angle_n4 <= np.min(angle_range2)+margin:
                        neighbor_list.append([list(t4),list(n4)])
                        
        if np.max(angle_range1) - np.min(angle_range1) > np.pi:
            if np.max(angle_range2) - np.min(angle_range2) > np.pi:
                if angle_t4 >= np.max(angle_range1)-margin and angle_t4 <= np.pi+margin or angle_t4 >= -np.pi-margin and angle_t4 <= np.min(angle_range1)+margin:
                    if angle_n4 >= np.max(angle_range2)-margin and angle_n4 <= np.pi+margin or angle_n4 >= -np.pi-margin and angle_n4 <= np.min(angle_range2)+margin:
                        neighbor_list.append([list(t4),list(n4)])            
    

    return neighbor_list

    
    
def get_goal_(point, centerPoint,angle_range, goal, goal_direction):
    r = 1
    goal_vector = [r*np.cos(goal_direction), r*np.sin(goal_direction)]
    circle_vector1 = [r*np.cos(goal_direction+np.pi/2), r*np.sin(goal_direction+np.pi/2)]
    circle_vector2 = [r*np.cos(goal_direction-np.pi/2), r*np.sin(goal_direction-np.pi/2)]
    circle_center1 = np.array(goal) + circle_vector1
    circle_center2 = np.array(goal) + circle_vector2
    
    if abs(distance(circle_center1, point)-1) < 0.01 or abs(distance(circle_center2, point)-1) < 0.01:
        return [[goal, goal]]
    angle1 = np.arctan2(-circle_vector1[1], -circle_vector1[0])
    angle11 = angle1 + 0.99*np.pi
    if angle11 > np.pi:
        angle11 = angle11 - 2*np.pi
    angle12 = angle1 - 0.99*np.pi
    if angle12 < -np.pi:
        angle12 = angle12 + 2*np.pi
    vect11 = [r*np.cos(angle11), r*np.sin(angle11)]
    vect12 = [r*np.cos(angle12), r*np.sin(angle12)]
    if np.dot(vect11, goal_vector) < 0:
        goal_angle_range1 = [angle11, angle1]
    if np.dot(vect12, goal_vector) < 0:
        goal_angle_range1 = [angle12, angle1]
        
    angle2 = np.arctan2(-circle_vector2[1], -circle_vector2[0])
    angle21 = angle2 + 0.99*np.pi
    if angle21 > np.pi:
        angle21 = angle21 - 2*np.pi
    angle22 = angle2 - 0.99*np.pi
    if angle22 < -np.pi:
        angle22 = angle22 + 2*np.pi
    vect21 = [r*np.cos(angle21), r*np.sin(angle21)]
    vect22 = [r*np.cos(angle22), r*np.sin(angle22)]
    if np.dot(vect21, goal_vector) < 0:
        goal_angle_range2 = [angle21, angle2]
    if np.dot(vect22, goal_vector) < 0:
        goal_angle_range2 = [angle22, angle2]  
    
    # print(centerPoint,circle_center1,angle_range,goal_angle_range1)
    # print(centerPoint,circle_center2,angle_range,goal_angle_range2)
    neighbor1 = get_tangent_node(centerPoint,circle_center1,angle_range,goal_angle_range1)
    neighbor2 = get_tangent_node(centerPoint,circle_center2,angle_range,goal_angle_range2)
    # print(neighbor1)
    # print(neighbor2)
    neighbor_list = []
    for neighbor in neighbor1:
        vect0 = np.array(neighbor[0]) - np.array(point)  
        vect1 = np.array(neighbor[1]) - np.array(neighbor[0])
        vect2 = np.array(goal) - np.array(neighbor[1])
        vect3 = goal_vector
        if np.linalg.norm(vect2) == 0:
            vect2 = vect1
        if np.linalg.norm(vect3) == 0:
            vect3 = vect2
        if np.linalg.norm(vect0) == 0:
            vect0 = vect1
        if np.dot(vect1, vect2)/(np.linalg.norm(vect1)*np.linalg.norm(vect2)) > 0:
            if np.dot(vect2, vect3)/(np.linalg.norm(vect2)*np.linalg.norm(vect3)) > 0:
                if np.dot(vect0, vect1)/(np.linalg.norm(vect1)*np.linalg.norm(vect2)) > 0:
                    neighbor_list.append(neighbor)
                
    for neighbor in neighbor2:
        vect0 = np.array(neighbor[0]) - np.array(point)        
        vect1 = np.array(neighbor[1]) - np.array(neighbor[0])
        vect2 = np.array(goal) - np.array(neighbor[1])
        vect3 = goal_vector
        # if np.linalg.norm(vect1) == 0:
        #     print(neighbor[1], neighbor[0])
        #     print(centerPoint,circle_center2,angle_range,goal_angle_range2)

        if np.linalg.norm(vect2) == 0:
            vect2 = vect1
        if np.linalg.norm(vect3) == 0:
            vect3 = vect2
        if np.linalg.norm(vect0) == 0:
            vect0 = vect1
        if np.dot(vect1, vect2)/(np.linalg.norm(vect1)*np.linalg.norm(vect2)) > 0:
            if np.dot(vect2, vect3)/(np.linalg.norm(vect2)*np.linalg.norm(vect3)) > 0:
                if np.dot(vect0, vect1)/(np.linalg.norm(vect1)*np.linalg.norm(vect2)) > 0:
                    neighbor_list.append(neighbor)
         
    if len(neighbor_list) == 1:
        return neighbor_list
    
    if len(neighbor_list) == 2:
        neighbor1 = neighbor_list[0]
        neighbor2 = neighbor_list[1]
        dist1 = distance(neighbor1[1], goal)
        dist2 = distance(neighbor2[1], goal)
        if dist1 < dist2:
            return [neighbor1]
        else:
            return [neighbor2]
    else:
        return []
        


def get_goal_from_start_(start, start_direction, goal, goal_direction):
    r = 1
    start_vector = [r*np.cos(start_direction), r*np.sin(start_direction)]
    circle_vector1 = [r*np.cos(start_direction+np.pi/2), r*np.sin(start_direction+np.pi/2)]
    circle_vector2 = [r*np.cos(start_direction-np.pi/2), r*np.sin(start_direction-np.pi/2)]
    circle_center1 = np.array(start) + circle_vector1
    circle_center2 = np.array(start) + circle_vector2
    angle1 = np.arctan2(-circle_vector1[1], -circle_vector1[0])
    angle11 = angle1 + 0.99*np.pi
    if angle11 > np.pi:
        angle11 = angle11 - 2*np.pi
    angle12 = angle1 - 0.99*np.pi
    if angle12 < -np.pi:
        angle12 = angle12 + 2*np.pi
    vect11 = [r*np.cos(angle11), r*np.sin(angle11)]
    vect12 = [r*np.cos(angle12), r*np.sin(angle12)]
    if np.dot(vect11, start_vector) > 0:
        start_angle_range1 = [angle11, angle1]
    if np.dot(vect12, start_vector) > 0:
        start_angle_range1 = [angle12, angle1]
        
    angle2 = np.arctan2(-circle_vector2[1], -circle_vector2[0])
    angle21 = angle2 + 0.99*np.pi
    if angle21 > np.pi:
        angle21 = angle21 - 2*np.pi
    angle22 = angle2 - 0.99*np.pi
    if angle22 < -np.pi:
        angle22 = angle22 + 2*np.pi
    vect21 = [r*np.cos(angle21), r*np.sin(angle21)]
    vect22 = [r*np.cos(angle22), r*np.sin(angle22)]
    if np.dot(vect21, start_vector) > 0:
        start_angle_range2 = [angle21, angle2]
    if np.dot(vect22, start_vector) > 0:
        start_angle_range2 = [angle22, angle2]  
        
    # print(start_angle_range1)
    # print(start_angle_range2)
    
    route1 = get_goal_(start, circle_center1,start_angle_range1, goal, goal_direction)
    # print(start, circle_center2,start_angle_range2, goal, goal_direction)
    route2 = get_goal_(start, circle_center2,start_angle_range2, goal, goal_direction)
    # print(route1, route2)
    
    if route2 != []:
        return route2
    
    if route1 != []:
        return route1
    
    else: 
        return []


      

def approaching_goal(goal_center1, goal_center2, point):
    if abs(distance(point, goal_center1) - 1) < 0.01:
        return goal_center1
    if abs(distance(point, goal_center2) - 1) < 0.01:
        return goal_center2
    else:
        return False
    
def approaching_start(start_center1, start_center2, point):
    if abs(distance(point, start_center1) - 1) < 0.01:
        return start_center1
    if abs(distance(point, start_center2) - 1) < 0.01:
        return start_center2
    else:
        return False
        
        
class tangent_node_:
    def __init__(self,point,parent,goal_pos,start_pos,obstacle):
        self.r = 1      
          
        self.goal = goal_pos[0:2]
        self.goal_direction = goal_pos[2]
        goal_vector = [self.r*np.cos(self.goal_direction), self.r*np.sin(self.goal_direction)]
        circle_vector1 = [self.r*np.cos(self.goal_direction+np.pi/2), self.r*np.sin(self.goal_direction+np.pi/2)]
        circle_vector2 = [self.r*np.cos(self.goal_direction-np.pi/2), self.r*np.sin(self.goal_direction-np.pi/2)]
        self.goal_center1 = np.array(self.goal) + circle_vector1
        self.goal_center2 = np.array(self.goal) + circle_vector2  
        
        self.start = start_pos[0:2]
        self.start_direction = start_pos[-1]
        start_vector = [self.r*np.cos(self.start_direction), self.r*np.sin(self.start_direction)]
        circle_vector1 = [self.r*np.cos(self.start_direction+np.pi/2), self.r*np.sin(self.start_direction+np.pi/2)]
        circle_vector2 = [self.r*np.cos(self.start_direction-np.pi/2), self.r*np.sin(self.start_direction-np.pi/2)]
        self.start_center1 = np.array(self.start) + circle_vector1
        self.start_center2 = np.array(self.start) + circle_vector2
        
        self.parent = parent
        self.neighbor = []
        
        self.obstacle_class_list = []
        self.obstacle_points = []
        obstacle_ = cp.deepcopy(obstacle)
        for i in obstacle_:
            self.obstacle_class_list.append(Obstacle(i))
            for j in i:
                self.obstacle_points.append(j)

        if len(point) == 3:
            self.get_neighbor_start()

        if len(point) == 2:
            self.point = point[1]
            self.transit = point[0]
            #add from 1-2
            # print(self.goal)
            # print(self.point)
            # print(self.parent)
            # print("######################################################################")
            # print(self.start_center1, self.start_center2)
            # print(self.goal_center1, self.goal_center2)
            # print(approaching_goal(self.goal_center1, self.goal_center2, self.point))
            # print(approaching_goal(self.goal_center1, self.goal_center2, self.parent))
            if type(approaching_goal(self.goal_center1, self.goal_center2, self.point)) == bool and type(approaching_goal(self.goal_center1, self.goal_center2, self.parent)) == bool:
                # print(1)
                self.center,self.angle_range = self.get_center_and_angle_range()
                # print(self.get_center_and_angle_range())
                self.parent_center, self.parent_angle_range = self.get_center_and_angle_range_of_parent()
                # print(self.parent_center, self.parent_angle_range)
                
                self.get_neighbor()
                # print(self.center)
            if type(approaching_goal(self.goal_center1, self.goal_center2, self.point)) != bool and type(approaching_goal(self.goal_center1, self.goal_center2, self.parent)) == bool:
                # print(2)
                self.center,self.angle_range = self.get_center_and_angle_range()
                self.parent_center, self.parent_angle_range = self.get_center_and_angle_range_of_parent()
                self.neighbor.append([self.goal, self.goal])
                # print(self.center)
                # print(1)
            if type(approaching_goal(self.goal_center1, self.goal_center2, self.point)) != bool and type(approaching_goal(self.goal_center1, self.goal_center2, self.parent)) != bool:
                if distance(self.parent,self.goal) > 0.01:
                    if abs(distance(self.parent, self.goal_center1) - self.r) < 0.01:
                        self.center = self.goal_center1
                        # vect1 = np.array(self.parent) -np.array(self.goal_center1)
                        # vect2 = np.array(self.point) -np.array(self.goal_center1)
                        # self.transit = self.goal
                        self.angle_range = [0,0]
                        self.parent_center = self.goal_center1
                        self.parent_angle_range = [0,0]
                        self.neighbor.append([self.goal, self.goal]) 
                        
                    else:
                        self.center = self.goal_center2
                        self.transit = self.point
                        self.angle_range = [0,0]
                        self.parent_center = self.goal_center2
                        self.parent_angle_range = [0,0]
                        self.neighbor.append([self.goal, self.goal]) 
                                              
                else:    
                    self.center = self.goal
                    self.angle_range = [0,0]
                    self.parent_center = self.goal
                    self.parent_angle_range = [0,0]
                    self.neighbor.append([self.goal, self.goal])

            
    def get_neighbor_start(self):
        line_list = []
        theta = self.start_direction
        position = self.start
        self.point = self.start
                
        
        r = 1
        start_vector = [r*np.cos(theta), r*np.sin(theta)]
        circle_vector1 = [r*np.cos(theta+np.pi/2), r*np.sin(theta+np.pi/2)]
        circle_vector2 = [r*np.cos(theta-np.pi/2), r*np.sin(theta-np.pi/2)]
        circle_center1 = np.array(position) + circle_vector1
        circle_center2 = np.array(position) + circle_vector2
        angle1 = np.arctan2(-circle_vector1[1], -circle_vector1[0])
        angle11 = angle1 + 0.99*np.pi
        if angle11 > np.pi:
            angle11 = angle11 - 2*np.pi
        angle12 = angle1 - 0.99*np.pi
        if angle12 < -np.pi:
            angle12 = angle12 + 2*np.pi
        vect11 = [r*np.cos(angle11), r*np.sin(angle11)]
        vect12 = [r*np.cos(angle12), r*np.sin(angle12)]
        if np.dot(vect11, start_vector) > 0:
            start_angle_range1 = [angle11, angle1]
        if np.dot(vect12, start_vector) > 0:
            start_angle_range1 = [angle12, angle1]
            
        angle2 = np.arctan2(-circle_vector2[1], -circle_vector2[0])
        angle21 = angle2 + 0.99*np.pi
        if angle21 > np.pi:
            angle21 = angle21 - 2*np.pi
        angle22 = angle2 - 0.99*np.pi
        if angle22 < -np.pi:
            angle22 = angle22 + 2*np.pi
        vect21 = [r*np.cos(angle21), r*np.sin(angle21)]
        vect22 = [r*np.cos(angle22), r*np.sin(angle22)]
        if np.dot(vect21, start_vector) > 0:
            start_angle_range2 = [angle21, angle2]
        if np.dot(vect22, start_vector) > 0:
            start_angle_range2 = [angle22, angle2]  
            
        goal_line = get_goal_from_start_(position, theta, self.goal, self.goal_direction)
        self.center = list([list(circle_center1),list(circle_center2)])
        

        for i in self.obstacle_class_list:
            for j in range(0,len(i.single_obstacle)):
                if distance(circle_center1,i.single_obstacle[j]) > 0.01:
                    neighbor_list1 = get_tangent_node(circle_center1,i.single_obstacle[j],start_angle_range1,i.angle[j])
                    neighbor_list2 = get_tangent_node(circle_center2,i.single_obstacle[j],start_angle_range2,i.angle[j])
                    line_list = [*line_list, *neighbor_list1, *neighbor_list2]
        # line_list = [*line_list, [self.point, self.point],*goal_line]
        line_list = [*line_list, *goal_line]

        for i in line_list:
            tag = 0
            line = LineString(i)
            for j in self.obstacle_class_list:   
                poly = Polygon(j.single_obstacle) 
                poly = poly.buffer(1)           
                if poly.intersects(line) == False:
                    tag = tag + 1
            if tag == len(self.obstacle_class_list):
                self.neighbor.append(i)


                            
    
    def get_neighbor(self): 
        goal_line = get_goal_(self.point, self.center,self.angle_range, self.goal, self.goal_direction)
        line_list = [] 
        # self.neighbor.append([self.point, self.point])
        for i in self.obstacle_class_list:
            for j in range(0,len(i.single_obstacle)):
                if distance(self.center,i.single_obstacle[j]) > 0.01:
                    neighbor_list = get_tangent_node(self.center,i.single_obstacle[j],self.angle_range,i.angle[j])
                    line_list = [*line_list, *neighbor_list]
        # line_list = [*line_list, [self.point, self.point],*goal_line]
        line_list = [*line_list, *goal_line]        
        for i in line_list:
            tag = 0
            line = LineString(i)
            for j in self.obstacle_class_list:   
                poly = Polygon(j.single_obstacle)            
                if poly.intersects(line) == False:
                    tag = tag + 1
            if tag == len(self.obstacle_class_list):
                self.neighbor.append(i)

        
    def get_center_and_angle_range(self):
        goal_center = approaching_goal(self.goal_center1, self.goal_center2, self.point)
        if type(goal_center) != bool:
            vect1 = np.array(self.goal) - np.array(goal_center)
            vect2 = np.array(self.point) - np.array(goal_center)
            angle1 = np.arctan2(vect1[1], vect1[0])
            angle2 = np.arctan2(vect2[1], vect2[0])
            return [list(goal_center), [angle1, angle2]]

        if type(goal_center) == bool:
            for i in self.obstacle_class_list:
                for j in range(0,len(i.single_obstacle)):   
                    if abs(distance(self.point, i.single_obstacle[j]) - self.r) < 0.01:
                        return [i.single_obstacle[j], i.angle[j]]
                          
    def get_center_and_angle_range_of_parent(self):
        for i in self.obstacle_class_list:
            for j in range(0,len(i.single_obstacle)):   
                if abs(distance(self.parent, i.single_obstacle[j]) - self.r) < 0.01:
                    return [i.single_obstacle[j], i.angle[j]]

        start_center = approaching_goal(self.start_center1, self.start_center2, self.transit)
        # print("????")
        # print(start_center)
        if type(start_center) != bool:
            vect1 = np.array(self.start) - np.array(start_center)
            vect2 = np.array(self.transit) - np.array(start_center)
            angle1 = np.arctan2(vect1[1], vect1[0])
            angle2 = np.arctan2(vect2[1], vect2[0])
            return [list(start_center), [angle1, angle2]]


