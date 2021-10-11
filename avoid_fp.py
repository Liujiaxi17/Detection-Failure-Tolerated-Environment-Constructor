#ljx
import math
import numpy as np
from zzz_perception_msgs.msg import GridMapMsgStamped
from tf.transformations import euler_from_quaternion
from cmath import polar

from time import time

class FP_avoider():
    def __init__(self, GridMapMsg):
        self.boundries = list(GridMapMsg.g_map.distancelist)
        
        self.NUMOFGRID = GridMapMsg.g_map.NUMOFGRID
        self.ANGLERESOLUTION = GridMapMsg.g_map.ANGLERESOLUTION
        self.MAXDISTANCE = GridMapMsg.g_map.MAXDISTANCE

        self.ego_x = GridMapMsg.g_map.position.x
        self.ego_y = GridMapMsg.g_map.position.y
        self.ego_euler = euler_from_quaternion((GridMapMsg.g_map.quaternion.x,GridMapMsg.g_map.quaternion.y,\
                                                GridMapMsg.g_map.quaternion.z,GridMapMsg.g_map.quaternion.w))

        #self.ego_theta = math.degrees(ego_euler[2])
        if abs(self.ego_euler [1]) >1.5 and abs(self.ego_euler[1]) < 4.5:
            self.ego_euler = self.ego_euler + math.pi        
        self.ego_theta = self.ego_euler[2]
        
        
    def avoid_FP(self,fp):
        #if no collision
        #return True
        if len(fp.t) < 2 :
            return True
        ego_x = self.ego_x
        ego_y = self.ego_y       
        ego_theta = self.ego_theta
        fp_tran_x = (np.array(fp.x)-ego_x)*math.cos(ego_theta)+(np.array(fp.y)-ego_y)*math.sin(ego_theta)
        fp_tran_y = -(np.array(fp.x)-ego_x)*math.sin(ego_theta)+(np.array(fp.y)-ego_y)*math.cos(ego_theta)
        alpha = np.array(fp.yaw)-ego_theta #in radians
        for i in range(len(fp_tran_x)):
            obs_angle = math.atan2(2.0,4.0) #ego car's dimension
            obs_alpha_0 = alpha[i] #- math.atan2(fp_tran_y[i],fp_tran_x[i])
            obs_theta = math.atan2(fp_tran_y[i],fp_tran_x[i])
            obs_distance = math.sqrt(fp_tran_x[i]**2+fp_tran_y[i]**2)
            if obs_distance < 3:
                continue
            if obs_distance > self.MAXDISTANCE-4:
                break
            
            obs_length = math.sqrt(5) #half of the rectangle's duijiaoxian
            #find the pole angle range 
            temp_angle_1 = obs_alpha_0 + obs_angle
            temp_angle_2 = obs_alpha_0 - obs_angle
            l_c1 = obs_length * math.cos(temp_angle_1)
            l_c2 = obs_length * math.cos(temp_angle_2)
            l_s1 = obs_length * math.sin(temp_angle_1)
            l_s2 = obs_length * math.sin(temp_angle_2)

            (distance_1, obs_alpha_1)  = polar(complex(fp_tran_x[i]+l_c1,fp_tran_y[i]+l_s1))
            (distance_2, obs_alpha_2)  = polar(complex(fp_tran_x[i]+l_c2,fp_tran_y[i]+l_s2))
            (distance_3, obs_alpha_3)  = polar(complex(fp_tran_x[i]-l_c1,fp_tran_y[i]-l_s1))
            (distance_4, obs_alpha_4)  = polar(complex(fp_tran_x[i]-l_c2,fp_tran_y[i]-l_s2))

            #use the four points of the rectangle and its center to check collsion
            obs_alpha_list = [obs_alpha_1,obs_alpha_2,obs_alpha_3,obs_alpha_4, obs_theta]
            obs_distance_list = [distance_1 , distance_2 ,distance_3 ,distance_4 , obs_distance]
            
            for i in range(5):
                index = int(math.degrees(obs_alpha_list[i])/self.ANGLERESOLUTION+0.5)
                if (index < 0):
                    index = index + self.NUMOFGRID
                if self.boundries[index] < obs_distance_list[i]:
                    return False
                # index_right = index + 1
                # if (index_right == self.NUMOFGRID):
                #     index_right = 0
                # if self.boundries[index] < obs_distance_list[i] or self.boundries[index_right] < obs_distance_list[i]: 
                #     #print(obs_alpha_list, ego_theta)
                #     #print(obs_distance_list[i],math.degrees(obs_alpha_list[i]),self.boundries[index],self.boundries[index_right],index,index_right)
                #     return False

        return True  

