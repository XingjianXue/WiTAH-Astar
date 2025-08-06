import numpy as np

# obstacle = [[[6, 5], [12,5], [12,10], [6.1,10]],
#             [[15, 17], [20, 17.5], [19, 22], [15, 22]],
#             [[15.9,9.5],[18,6],[19.6,12],[17,13], [15.2, 12]],
#             [[2,17],[6,17],[6.7,21],[5,23], [1.9,19]]]

class Obstacle:
    def __init__(self, single_obstacle):
        self.single_obstacle = single_obstacle
        self.r = 1
        self.angle = []
        self.get_angle()
        
    def get_angle(self):
        for i in range(0,len(self.single_obstacle)):
            if i == 0:
                angle_range = []
                point = np.array(self.single_obstacle[i])
                vect1 = np.array(self.single_obstacle[i + 1]) - np.array(self.single_obstacle[i])
                vect2 = np.array(self.single_obstacle[-1]) - np.array(self.single_obstacle[i])
                vert11 = np.array([vect1[1],-vect1[0]])/np.linalg.norm(vect1)*self.r
                vert12 = np.array([-vect1[1],vect1[0]])/np.linalg.norm(vect1)*self.r
                vert21 = np.array([vect2[1],-vect2[0]])/np.linalg.norm(vect1)*self.r
                vert22 = np.array([-vect2[1],vect2[0]])/np.linalg.norm(vect1)*self.r
                if np.dot(vert11, vect2) < 0:
                    angle_range.append(np.arctan2(vert11[1],vert11[0]))
                if np.dot(vert12, vect2) < 0:
                    angle_range.append(np.arctan2(vert12[1],vert12[0]))
                if np.dot(vert21, vect1) < 0:
                    angle_range.append(np.arctan2(vert21[1],vert21[0]))
                if np.dot(vert22, vect1) < 0:
                    angle_range.append(np.arctan2(vert22[1],vert22[0]))
                self.angle.append(angle_range)
                
            if i == len(self.single_obstacle) - 1:
                angle_range = []
                point = np.array(self.single_obstacle[i])
                vect1 = np.array(self.single_obstacle[i - 1]) - np.array(self.single_obstacle[i])
                vect2 = np.array(self.single_obstacle[0]) - np.array(self.single_obstacle[i])
                vert11 = np.array([vect1[1],-vect1[0]])/np.linalg.norm(vect1)*self.r
                vert12 = np.array([-vect1[1],vect1[0]])/np.linalg.norm(vect1)*self.r
                vert21 = np.array([vect2[1],-vect2[0]])/np.linalg.norm(vect1)*self.r
                vert22 = np.array([-vect2[1],vect2[0]])/np.linalg.norm(vect1)*self.r
                if np.dot(vert11, vect2) < 0:
                    angle_range.append(np.arctan2(vert11[1],vert11[0]))
                if np.dot(vert12, vect2) < 0:
                    angle_range.append(np.arctan2(vert12[1],vert12[0]))
                if np.dot(vert21, vect1) < 0:
                    angle_range.append(np.arctan2(vert21[1],vert21[0]))
                if np.dot(vert22, vect1) < 0:
                    angle_range.append(np.arctan2(vert22[1],vert22[0]))
                self.angle.append(angle_range)
                
            if i != 0 and i != len(self.single_obstacle) - 1:
                angle_range = []
                point = np.array(self.single_obstacle[i])
                vect1 = np.array(self.single_obstacle[i - 1]) - np.array(self.single_obstacle[i])
                vect2 = np.array(self.single_obstacle[i + 1]) - np.array(self.single_obstacle[i])
                vert11 = np.array([vect1[1],-vect1[0]])/np.linalg.norm(vect1)*self.r
                vert12 = np.array([-vect1[1],vect1[0]])/np.linalg.norm(vect1)*self.r
                vert21 = np.array([vect2[1],-vect2[0]])/np.linalg.norm(vect1)*self.r
                vert22 = np.array([-vect2[1],vect2[0]])/np.linalg.norm(vect1)*self.r
                if np.dot(vert11, vect2) < 0:
                    angle_range.append(np.arctan2(vert11[1],vert11[0]))
                if np.dot(vert12, vect2) < 0:
                    angle_range.append(np.arctan2(vert12[1],vert12[0]))
                if np.dot(vert21, vect1) < 0:
                    angle_range.append(np.arctan2(vert21[1],vert21[0]))
                if np.dot(vert22, vect1) < 0:
                    angle_range.append(np.arctan2(vert22[1],vert22[0]))
                self.angle.append(angle_range)
                
# obs = Obstacle([[6, 5], [12,5], [12,10], [6.1,10]])
# print(obs.angle)

            
        
        
        