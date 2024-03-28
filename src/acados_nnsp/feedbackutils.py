import sys
import time, os
import numpy as np
import pandas as pd
import rospy

# import NN model
from nn_model import load_nn_model

from scipy.interpolate import CubicSpline

# import ros messages
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates



class AMCL_pos:
    def __init__(self):
        self.pos = np.array([0, 0, 0])
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        print("Subscribed to amcl_pose")
    def callback(self,PosWCov):
        self.pos = np.array([PosWCov.pose.pose.position.x, PosWCov.pose.pose.position.y, 2*np.arctan2(PosWCov.pose.pose.orientation.z,PosWCov.pose.pose.orientation.w)])

class Goal_reference:
    def __init__(self):
        self.goal = np.array([0, 0, 0])
        self.sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.callback)
        print("Subscribed to /move_base/TrajectoryPlannerROS/global_plan")
    def callback(self,path):
        last_point = path.poses[-1]
        self.goal = np.array([last_point.pose.position.x, last_point.pose.position.y, 2*np.arctan2(last_point.pose.orientation.z, last_point.pose.orientation.w)])
class Path_reference:
    def __init__(self):
        #self.goal = np.zeros((5,3))
        self.s = np.array([0.1,0.6,1.1,1.6])
        self.x = np.zeros(4)
        self.y = np.zeros(4)
        self.idx = 0
        self.sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.callback)
        print("Subscribed to /move_base/TrajectoryPlannerROS/global_plan")
    def callback(self,path):
        last_index = len(path.poses)-1
        point1 = path.poses[int(np.floor(last_index/5))]
        point2 = path.poses[int(np.floor(2*last_index/5))]
        point3 = path.poses[int(np.floor(3*last_index/5))]
        point4 = path.poses[int(np.floor(4*last_index/5))]
        point5 = path.poses[last_index]
        path = np.array([[point1.pose.position.x, point1.pose.position.y, 2*np.arctan2(point1.pose.orientation.z, point1.pose.orientation.w)],
                        [point2.pose.position.x, point2.pose.position.y, 2*np.arctan2(point2.pose.orientation.z, point2.pose.orientation.w)],
                        [point3.pose.position.x, point3.pose.position.y, 2*np.arctan2(point3.pose.orientation.z, point3.pose.orientation.w)],
                        [point4.pose.position.x, point4.pose.position.y, 2*np.arctan2(point4.pose.orientation.z, point4.pose.orientation.w)],
                        [point5.pose.position.x, point5.pose.position.y, 2*np.arctan2(point5.pose.orientation.z, point5.pose.orientation.w)]])
        self.x = path[1:,0]
        self.y = path[1:,1]
        self.idx = last_index
        if last_index>0:
            grad = np.array([path[1:,:2]-path[:4,:2]])
            grad = np.squeeze(grad)
            dists = np.sqrt(grad[:,0]**2+grad[:,1]**2)
            dists = dists.reshape(4) 
            self.s = np.cumsum(dists)
        else:
            self.s = np.array([0.0,0.2,0.4,0.6])
class Path_reference_Npoint:
    def __init__(self,N=5):
        #self.goal = np.zeros((5,3))
        self.N = N
        self.s = np.linspace(0,1,N)
        self.x = np.zeros(N)
        self.y = np.zeros(N)
        #self.last_idx = 0
        self.path = np.zeros((N,3))
        self.sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.callback)
        print("Subscribed to /move_base/TrajectoryPlannerROS/global_plan")
    def callback(self,path):
        last_index = len(path.poses)
        if self.N>last_index:
            self.N = last_index
            self.s = np.linspace(0,1,last_index)
            self.x = np.zeros(last_index)
            self.y = np.zeros(last_index)
            self.path = np.zeros((last_index,3))
        path_indexes = np.floor(np.arange(self.N)*(last_index/self.N)).astype(int)
        i=0
        for index in path_indexes:
            self.path[i,:] = np.array([path.poses[index].pose.position.x, path.poses[index].pose.position.y, 2*np.arctan2(path.poses[index].pose.orientation.z, path.poses[index].pose.orientation.w)])
            i+=1
        self.x = self.path[1:,0]
        self.y = self.path[1:,1]
        #self.last_idx = last_index
        if last_index>0:
            grad = np.array([self.path[1:,:2]-self.path[:-1,:2]])
            grad = np.squeeze(grad)
            dists = np.sqrt(grad[:,0]**2+grad[:,1]**2)
            dists = dists.reshape(self.N-1) 
            self.s = np.cumsum(dists)
        else:
            self.s = np.linspace(0,1,N)