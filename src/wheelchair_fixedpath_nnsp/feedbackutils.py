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
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import OccupancyGrid



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

class Costmap:
    def __init__(self):
        self.width = 200
        self.length = 200
        self.resolution = 0.05
        #self.costmap = np.zeros((200,200))
        self.obstacleX = np.zeros(10)-100
        self.obstacleY = np.zeros(10)-100
        self.sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.callback)
        print("Subscribed to /move_base/local_costmap/costmap")
    def callback(self,grid):
        self.width = grid.info.width
        self.height = grid.info.height
        self.resolution = grid.info.resolution
        costmap = np.array(grid.data).reshape((self.width,self.height))
        count = 0
        originX = grid.info.origin.position.x
        originY = grid.info.origin.position.y
        H = self.height//2
        W = self.width//2
        for i in range(5, H):
            side = (2+2*i)
            if count>9:
                    break
            for k in range(side-2):
                if(costmap[H+i-(side-1),W+i-k-1]>99):
                    self.obstacleX[count] = (i-k-1)*self.resolution+originX
                    self.obstacleY[count] = (i-(side-1))*self.resolution+originY
                    count+=1
                if count>9:
                    break
                if(costmap[H+i,W+i-k-1]):
                    self.obstacleX[count] = (i-k-1)*self.resolution+originX
                    self.obstacleY[count] = (i)*self.resolution+originY
                    count+=1
                if count>9:
                    break
                if(costmap[H+i-k-1,W+i-(side-1)]):
                    self.obstacleX[count] = (i-(side-1))*self.resolution+originX
                    self.obstacleY[count] = (i-k-1)*self.resolution+originY
                    count+=1
                if count>9:
                    break
                if(costmap[H+i-k-1,W+i]):
                    self.obstacleX[count] = i*self.resolution+originX
                    self.obstacleY[count] = (i-k-1)*self.resolution+originY
                    count+=1

class CostmapPath:
    def __init__(self):
        self.width = 200
        self.length = 200
        self.resolution = 0.05
        self.originX = 0
        self.originY = 0
        self.costmap = np.zeros((200,200))
        self.sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.callback)
        print("Subscribed to /move_base/local_costmap/costmap")
    def callback(self,grid):
        self.width = grid.info.width
        self.height = grid.info.height
        self.resolution = grid.info.resolution
        self.originX = grid.info.origin.position.x
        self.originY = grid.info.origin.position.y
        self.costmap = np.array(grid.data).reshape((self.width,self.height))

    def findObst(self,N=10):
        obs_indexes = np.argwhere(self.costmap>99)
        obsX = np.zeros(N)-100
        obsY = np.zeros(N)-100
        if obs_indexes.shape[0]<=1:
            return obsX, obsY
        H = self.height//2
        W = self.width//2
        diff_row = abs(obs_indexes[:,0]-H)
        if diff_row.shape[0]<N:
            N = diff_row.shape[0]
        diff_col = abs(obs_indexes[:,1]-W)
        obs_rows = obs_indexes[np.argpartition(diff_row,N-1)[:N-1],0]
        obs_cols = obs_indexes[np.argpartition(diff_col,N-1)[:N-1],1]
        obsY[:obs_rows.shape[0]] = -(obs_rows-H)*self.resolution+self.originY
        obsX[:obs_cols.shape[0]] = (obs_cols-W)*self.resolution+self.originX
        return obsX, obsY

    def findObstPath(self,path,N=10):
        obs_indexes = np.argwhere(self.costmap>99)
        obsX = np.zeros(N)-100
        obsY = np.zeros(N)-100
        if obs_indexes.shape[0]<=1:
            return obsX, obsY
        H = self.height//2
        W = self.width//2
        steps = path.shape[0]
        
        for i in range(steps):
            obs_per_step = N//steps if N>steps else 1
            col_offset = (path[i,1]-originY)//self.resolution
            row_offset = (path[i,0]-originX)//self.resolution
            diff_row = abs(obs_indexes[:,0]-H-col_offset)
            if diff_row.shape[0]<obs_per_step:
                obs_per_step = diff_row.shape[0]
            diff_col = abs(obs_indexes[:,1]-W-row_offset)
            obs_rows = obs_indexes[np.argpartition(diff_row,obs_per_step)[:obs_per_step],0]
            obs_cols = obs_indexes[np.argpartition(diff_col,obs_per_step)[:obs_per_step],1]
            obsY[:obs_rows.shape[0]] = (obs_rows-H)*self.resolution+self.originY
            obsX[:obs_cols.shape[0]] = (obs_cols-W)*self.resolution+self.originX
            
        
        return obsX, obsY
        
    def findObstacles(self,path,N=10):
        costmap = self.costmap
        count = 0
        originX = grid.info.origin.position.x
        originY = grid.info.origin.position.y
        H = self.height//2
        W = self.width//2
        obstacleX = np.zeros(N)-100
        obstacleY = np.zeros(N)-100
        steps = path.shape[0]
        obs_per_step = N//steps if N>steps else 1
        for j in range(steps):
            col_offset = (path[j,1]-originY)//self.resolution
            row_offset = (path[j,0]-originX)//self.resolution
            for i in range(5, H):
                side = (2+2*i)
                if count>N-1:
                    break
                for k in range(side-2):
                    if(costmap[H+i-(side-1)+row_offset,W+i-k-1+col_offset]>99):
                        self.obstacleX[count] = (i-k-1)*self.resolution+originX
                        self.obstacleY[count] = (i-(side-1))*self.resolution+originY
                        count+=1
                    if count>9:
                        break
                    if(costmap[H+i,W+i-k-1]):
                        self.obstacleX[count] = (i-k-1)*self.resolution+originX
                        self.obstacleY[count] = (i)*self.resolution+originY
                        count+=1
                    if count>9:
                        break
                    if(costmap[H+i-k-1,W+i-(side-1)]):
                        self.obstacleX[count] = (i-(side-1))*self.resolution+originX
                        self.obstacleY[count] = (i-k-1)*self.resolution+originY
                        count+=1
                    if count>9:
                        break
                    if(costmap[H+i-k-1,W+i]):
                        self.obstacleX[count] = i*self.resolution+originX
                        self.obstacleY[count] = (i-k-1)*self.resolution+originY
                        count+=1

                
def ArcLength(x,y):
    dx = x[1:]-x[:-1]
    dy = y[1:]-y[:-1]
    s = np.cumsum(np.sqrt(dx**2+dy**2))
    return s


class Gazebo_pos:
    def __init__(self,index=17):
        self.robot_index = index
        self.pos = np.array([0, 0, 0])
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        print("Subscribed to gazebo model states")
    def callback(self,states):
        robot_pose = states.pose[self.robot_index]
        self.pos = np.array([robot_pose.position.x, robot_pose.position.y, 2*np.arctan2(robot_pose.orientation.z,robot_pose.orientation.w)])



class Gazebo_caster_state:
    def __init__(self,index_l=4,index_r=6):
        self.index_l = index_l
        self.index_r = index_r
        self.rot = np.array([0.0,0.0])
        self.sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback)
        print("Subscribed to gazebo link states")
    def callback(self,states):
        rot_l = states.pose[self.index_l]
        rot_r = states.pose[self.index_r]
        self.rot[0] = 2*np.arctan2(rot_l.orientation.z,rot_l.orientation.w)
        self.rot[1] = 2*np.arctan2(rot_r.orientation.z,rot_r.orientation.w)
