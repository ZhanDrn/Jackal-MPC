#!/usr/bin/env python3
"""ACADOS_torch controller."""

import sys
import time, os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import rospy

# import NN model
from nn_model import load_nn_model

from scipy.interpolate import CubicSpline

from solversetup import *
from plotFcn import *
from spatialConversion import *
from feedbackutils import *

# import ros messages
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

# initialize global variables
gazebo_pose = [0,0,0]
amcl_pos = [0,0,0]
s_plan = np.array([0.1,0.6,1.1,1.6])
x_plan = np.zeros(4)
y_plan = np.zeros(4)
idx_plan = 0

# Define ros messages callback
def pose_gazebo_callback(states):
    global gazebo_pose
    index=17
    robot_pose = states.pose[index]
    gazebo_pose = np.array([robot_pose.position.x, robot_pose.position.y, 2*np.arctan2(robot_pose.orientation.z,robot_pose.orientation.w)])

def pose_amcl_callback(PosWCov):
    global amcl_pos
    amcl_pos = np.array([PosWCov.pose.pose.position.x, PosWCov.pose.pose.position.y, 2*np.arctan2(PosWCov.pose.pose.orientation.z,PosWCov.pose.pose.orientation.w)])

def global_plan_callback(path):
    global x_plan, y_plan, idx_plan, s_plan, dists
    last_index = len(path.poses)-1
    point1 = path.poses[int(last_index//5)]
    point2 = path.poses[int(2*last_index//5)]
    point3 = path.poses[int(3*last_index//5)]
    point4 = path.poses[int(4*last_index//5)]
    point5 = path.poses[last_index]
    path = np.array([[point1.pose.position.x, point1.pose.position.y, 2*np.arctan2(point1.pose.orientation.z, point1.pose.orientation.w)],
                    [point2.pose.position.x, point2.pose.position.y, 2*np.arctan2(point2.pose.orientation.z, point2.pose.orientation.w)],
                    [point3.pose.position.x, point3.pose.position.y, 2*np.arctan2(point3.pose.orientation.z, point3.pose.orientation.w)],
                    [point4.pose.position.x, point4.pose.position.y, 2*np.arctan2(point4.pose.orientation.z, point4.pose.orientation.w)],
                    [point5.pose.position.x, point5.pose.position.y, 2*np.arctan2(point5.pose.orientation.z, point5.pose.orientation.w)]])
    x_plan = path[1:,0]
    y_plan = path[1:,1]
    idx_plan = last_index
    if last_index>0:
        grad = np.array([path[1:,:2]-path[:4,:2]])
        grad = np.squeeze(grad)
        dists = np.sqrt(grad[:,0]**2+grad[:,1]**2)
        dists = dists.reshape(4) 
        s_plan = np.cumsum(dists)
    else:
        s_plan = np.array([0.1,0.6,1.1,1.6])




def main():
    global x_plan, y_plan, idx_plan, s_plan, amcl_pos, gazebo_pose, dists
    print("Starting ros node")

    rospy.init_node('mpc_simple')
    control_pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray,queue_size = 10)
    #control_pub = rospy.Publisher('/cmd_vel', Float64MultiArray,queue_size = 10)
    traj_pub = rospy.Publisher('mpc_solver/low_level_trajectory', Path,queue_size = 10)
    
    ## All subscriptions
    #sub_pose_gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_gazebo_callback)
    #sub_pose_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_amcl_callback)
    #sub_global_plan = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, global_plan_callback)
    #pos_sub = AMCL_pos()
    pos_sub = Gazebo_pos(1)
    #path_sub = Path_reference_Npoint()
    costmap_sub = CostmapPath()
    
    #Solver Setup
    Tf = 3.5  # prediction horizon
    N = 140  # number of discretization steps
    N_obst = 1
    r=0.11
    Ts = Tf/N
    learned_dyn = load_nn_model("/home/zhan/MPC_skid_ws_1/src/acados_nnsp/models/bestmodel_backup.pt", USE_GPU=True)
    
    constraint, model, acados_solver = acados_settings(Tf, N, learned_dyn,N_obst)
    # dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    print('Warming up model...')
    x_l = []
    for i in range(N):
        x_l.append(acados_solver.get(i, "x")[:5])
    for i in range(5*N):
        learned_dyn.approx_params(np.stack(x_l, axis=0), flat=True)
    print('Warmed up!')
    th = 0
    s = 0
    
    x0 = np.array([0, 0, 0, 0, 0, 0])
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    status = acados_solver.solve()
    x = np.array(acados_solver.get(1, "x"))
    V_target = 0.5
    x_l = []

    iter=0
    rate = rospy.Rate(40) # hz
    file = open('/home/zhan/MPC_skid_ws_1/src/acados_fixedpath_obst_nnsp/trifolium_path.csv')
    data = np.genfromtxt(file,delimiter=',')
    n = len(data[0])
    points_x = data[0]*5
    points_y = data[1]*5
    orient_w = data[2]
    orient_z = data[3]
    arcLength = ArcLength(points_x, points_y)
    #arcLength = np.linspace(0,6.7*2,num = points_x.shape[0])
    csX = CubicSpline(arcLength, points_x[1:])
    csY = CubicSpline(arcLength, points_y[1:])
    #print(arcLength)
    #plt.plot(csX(arcLength),csY(arcLength))
    #plt.show()
    while not rospy.is_shutdown():
        iter+=1
        #print(s_plan.shape)
        #print(s_plan, dists, idx_plan)
        #csX = CubicSpline(x[5]+path_sub.s, path_sub.x)
        #csY = CubicSpline(x[5]+path_sub.s, path_sub.y)
        pos = pos_sub.pos
        #pos = amcl_pos
        #rot = amcl_pos[2]
        # Process sensor data here.
        x0 = np.array(acados_solver.get(0, "x"))[5]
        s = FindS(csX,csY,pos[0],pos[1],x0-2,x0+2,0.1)
        #s = x0
        xe, ye ,the = RefToS(csX,csY,pos[0],pos[1],pos[2],s)
        x0 = np.array([xe, ye, the, x[3], x[4],s])

        yref = np.zeros(9)
        yref[5] = s+Tf*2
        for j in range(N-1):
            Xref = np.array(acados_solver.get(j+1, "x"))
            #yref=np.array([0,0,0,0,0,s+Ts*V_target*j,0,0])
            #yref=np.array([0,0,0,0,0,x[5]+path_sub.s[-1],0,0,0])
            acados_solver.set(j, "yref", yref)
        #yref = np.zeros(18)
        #yref[5] = x[5]+path_sub.s[-1]
        #yref=np.array([0,0,0,0,0,x[5]+path_sub.s[-1],0,0])
        acados_solver.set(N, "yref", yref[:6])
        acados_solver.set(0, "lbx", x0)
        acados_solver.set(0, "ubx", x0)
        
        x_l = []
        for i in range(N):
            x_l.append(acados_solver.get(i, "x"))
        x_l = np.array(x_l).reshape((N,-1))
        params = learned_dyn.approx_params(np.stack(x_l[:,:5], axis=0), flat=True)
        #print(np.sum(np.isnan(costmap_sub.obstacleX)))
        #print(np.sum(np.isnan(costmap_sub.obstacleY)))
        
        for i in range(N):
            K = ComputeCurvature(csY,csX,acados_solver.get(i, "x")[5])
            #obsX = costmap_sub.obstacleX-csX(x_l[i,5])
            #obsY = costmap_sub.obstacleY-csY(x_l[i,5])
            if N_obst>1:
                obsX, obsY = costmap_sub.findObst(N=N_obst)
            else:
                obsX = -100
                obsY = -100
            
            #print(obsX)
            #print(obsY)
            #acados_solver.set(i, "p", K)
            acados_solver.set(i, "p", np.hstack((params[i],K,obsX,obsY)))
        status = acados_solver.solve()
        x = np.array(acados_solver.get(1, "x"))
        u = np.array(acados_solver.get(0, "u"))
        print("State: %g %g %g %g %g %g" % (x[0], x[1], x[2], x[3], x[4], x[5]))
        print("Feedback: %g %g %g %g %g %g" % (x0[0], x0[1], x0[2], x0[3], x0[4],x0[5]))
        control = [x[4],x[4],x[3],x[3]]
        msg = Float64MultiArray()
        msg.data = control
        rospy.loginfo(msg)
        control_pub.publish(msg)
        traj_msg = Path()
        pose_msg = PoseStamped()
        for i in range(N//3):
            pose_msg.pose.position.x = x_l[3*i,0]+csX(x_l[3*i,5])
            pose_msg.pose.position.y = x_l[3*i,1]+csY(x_l[3*i,5])
            pose_msg.pose.position.z = 0.0
            traj_msg.poses.append(pose_msg)
        traj_pub.publish(traj_msg)

        rate.sleep()

if __name__ == "__main__":
    main()
