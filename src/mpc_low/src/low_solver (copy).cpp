#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/SetModelConfiguration.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <rosgraph_msgs/Clock.h>

#include "acados_solver_holder.cpp"
//#include "acados_solver_holder.h"
// acados

using namespace std;

ofstream myfile;
ofstream myfile_low;
ofstream myperffile;

double gazebo_time;

void gazebo_time_msg(const rosgraph_msgs::Clock::ConstPtr& msg){
    gazebo_time = msg->clock.toSec();
    return;
}

// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 
    double goal_pose[6]={0.0};
    double current_pose[6]={0.0};
    double tracking_goal[30]={0.0};
    int goal_status = 0;
    double costmap_data[520] = {-100.0};

    void goal_calback(const nav_msgs::Path::ConstPtr& msg){
        /*int size_of_trajectory = msg->poses.size();
        double angle_goal = 0.0;
        if (size_of_trajectory>10) size_of_trajectory = 10;
        if (size_of_trajectory>1) {
            goal_status = 1;
            for (int i=0;i<size_of_trajectory;i++){
                tracking_goal[i*3+0] = msg->poses[i].pose.position.x;
                tracking_goal[i*3+1] = msg->poses[i].pose.position.y;
                angle_goal = 2* std::atan2( msg->poses[i].pose.orientation.z,msg->poses[i].pose.orientation.w);
                tracking_goal[i*3+2] = angle_goal;
            }
            for (int i=size_of_trajectory;i<10;i++) {
                tracking_goal[i*3+0] = tracking_goal[(size_of_trajectory-1)*3+0];
                tracking_goal[i*3+1] = tracking_goal[(size_of_trajectory-1)*3+1];
                tracking_goal[i*3+2] = tracking_goal[(size_of_trajectory-1)*3+2];
            }
            //goal_pose[0] = msg->poses[2].pose.position.x;
            //goal_pose[1] = msg->poses[2].pose.position.y;
            //double angle_goal = 2* std::atan2( msg->poses[2].pose.orientation.z,msg->poses[2].pose.orientation.w);
            //goal_pose[2] = angle_goal;
            goal_pose[0] = tracking_goal[(size_of_trajectory-1)*3+0];
            goal_pose[1] = tracking_goal[(size_of_trajectory-1)*3+0];
            goal_pose[2] = tracking_goal[(size_of_trajectory-1)*3+0];
        }*/
        int size_of_trajectory = (int)msg->poses.size()/2;
        double angle_goal = 0.0;
        if (size_of_trajectory>10) size_of_trajectory = 10;
        if (size_of_trajectory>1) {
            goal_status = 1;
            for (int i=0;i<size_of_trajectory;i++){
                /*
                tracking_goal[i*3+0] = msg->poses[i*2].pose.position.x;
                tracking_goal[i*3+1] = msg->poses[i*2].pose.position.y;
                angle_goal = 2* std::atan2( msg->poses[i*2].pose.orientation.z,msg->poses[i*2].pose.orientation.w);
                tracking_goal[i*3+2] = angle_goal;
                */
                tracking_goal[i*3+0] = msg->poses[i*2].pose.position.x;
                tracking_goal[i*3+1] = msg->poses[i*2].pose.position.y;
                
                tf::Quaternion q(
                    msg->poses[i*2].pose.orientation.x,
                    msg->poses[i*2].pose.orientation.y,
                    msg->poses[i*2].pose.orientation.z,
                    msg->poses[i*2].pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                
                angle_goal = yaw;
                tracking_goal[i*3+2] = angle_goal;
            }
            for (int i=size_of_trajectory;i<10;i++) {
                tracking_goal[i*3+0] = tracking_goal[(size_of_trajectory-1)*3+0];
                tracking_goal[i*3+1] = tracking_goal[(size_of_trajectory-1)*3+1];
                tracking_goal[i*3+2] = tracking_goal[(size_of_trajectory-1)*3+2];
            }
            goal_pose[0] = tracking_goal[(size_of_trajectory-1)*3+0];
            goal_pose[1] = tracking_goal[(size_of_trajectory-1)*3+0];
            goal_pose[2] = tracking_goal[(size_of_trajectory-1)*3+0];
        }
        else {
            goal_pose[0] = 0.0;
            goal_pose[1] = 0.0;
            goal_pose[2] = 0.0;
            goal_status = 0;
        }

        ROS_INFO("Trajectory recieved: %i ", msg->poses.size());
        return;
    }

    void amcl_pose_calback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        current_pose[0] = msg->pose.pose.position.x;
        current_pose[1] = msg->pose.pose.position.y;
        double angle_current = 2* std::atan2( msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        current_pose[2] = angle_current;
        //ROS_INFO("Trajectory recieved: %i ", msg->poses.size());
        return;
    }


    void costmap_calback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //ROS_INFO("Map recieved: %i ", msg->data.size());
        int obst_count = 0.0;
        int a = 0;
        int length = msg->data.size();
        // Map 200x200 with resolution of 0.05m
        // we need to convert map index to meters offset, map is 10m by 10m, middle is 5m
        for (int i=0;i<length;i++) {
	    if (msg->data[i]>99.0) {
            obst_count=obst_count+1;
            costmap_data[2*obst_count+0] = ((int)i / 200)*0.05-5.0;
            costmap_data[2*obst_count+1] = (i % 200)*0.05-5.0;
            if (obst_count==260) break;
        }
	    //printf("%f",obst_count);
	    a = a+1;
        }
        //printf("%i",obst_count);
        ROS_INFO("Obst info: %i", obst_count);
        ROS_INFO("Map info: %f %i %i %i", msg->info.resolution, msg->info.width, msg->info.height, length);
        return;
    }

}; 


int main(int argc, char **argv)
{
  myfile.open("../../../data_low.csv", ios::out);
  ros::init(argc, argv, "joint_controller_low");

  ros::NodeHandle n;


  ROS_INFO("Node Started");
  //--------------------------------
  GoalFollower my_follower;
  ros::Subscriber goal_topic = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1, &GoalFollower::goal_calback, &my_follower);
  ros::Subscriber costmap_topic = n.subscribe("/move_base/local_costmap/costmap", 1, &GoalFollower::costmap_calback, &my_follower);
  ros::Subscriber amcl_pose_topic = n.subscribe("/amcl_pose", 1, &GoalFollower::amcl_pose_calback, &my_follower);
  ros::Subscriber gazebo_time_listener = n.subscribe("/clock", 1, gazebo_time_msg);

  ros::Publisher low_level_trajectory = n.advertise<nav_msgs::Path>("mpc_solver/low_level_trajectory", 1);

  ros::Publisher motor_controller = n.advertise<std_msgs::Float64MultiArray>("/joint_group_velocity_controller/command", 1);

  ros::Publisher goal_arrow_pub = n.advertise<geometry_msgs::PoseStamped>("goal_arrow", 1);



  my_NMPC_solver myMpcSolver=my_NMPC_solver(5);

  double cgoal[3];

  double smallest_dist;
  double local_val;
  double min_dist[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000};
  ros::Rate loop_rate(40);

  // Big loop
  while (ros::ok())
  {
    double current_error = (my_follower.goal_pose[0]-my_follower.current_pose[0])*(my_follower.goal_pose[0]-my_follower.current_pose[0])+
(my_follower.goal_pose[1]-my_follower.current_pose[1])*(my_follower.goal_pose[1]-my_follower.current_pose[1])+
(my_follower.goal_pose[2]-my_follower.current_pose[2])*(my_follower.goal_pose[2]-my_follower.current_pose[2]);
    if (my_follower.goal_status || current_error>0.10) {
        // Goal reference position
        double current_robot_position[3];
        double costmap_data[520];
        double current_robot_goal[3];
        for (int i = 0; i < 2; ++i) current_robot_position[ i ] = 0.0;
        current_robot_position[ 2 ] = my_follower.current_pose[2]; // set orientation of the real world
 
        // substiture position from the goal and keep the goal orientation
	current_robot_goal[ 0 ] = my_follower.goal_pose[0]-my_follower.current_pose[0];
	current_robot_goal[ 1 ] = my_follower.goal_pose[1]-my_follower.current_pose[1];
	current_robot_goal[ 2 ] = my_follower.goal_pose[2];
    
        // load constraints for costmap
        for (int i = 0; i < 520; ++i) costmap_data[ i ] = my_follower.costmap_data[i];//-1000.0;

        double tracking_goal[30]={0.0};

        for (int i=0;i<10;i++) {
            tracking_goal[i*3+0] = my_follower.goal_pose[0]-my_follower.current_pose[0];
            tracking_goal[i*3+1] = my_follower.goal_pose[1]-my_follower.current_pose[1];
            tracking_goal[i*3+2] = my_follower.goal_pose[2];
        }

        // Check pose visualy
        geometry_msgs::PoseStamped goal_pose;
        
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = "odom";
        goal_pose.pose.position.x = my_follower.goal_pose[0];
        goal_pose.pose.position.y = my_follower.goal_pose[1];
        goal_pose.pose.position.z = 0.0;

        tf2::Quaternion myQuaternion;

        myQuaternion.setRPY(0.0,0.0,my_follower.goal_pose[2]);

        myQuaternion=myQuaternion.normalize();
        goal_pose.pose.orientation.x = myQuaternion.x();
        goal_pose.pose.orientation.y = myQuaternion.y();
        goal_pose.pose.orientation.z = myQuaternion.z();
        goal_pose.pose.orientation.w = myQuaternion.w();
        goal_arrow_pub.publish(goal_pose);

        /*ROS_INFO("Goal %f %f %f",my_follower.goal_pose[0],
                  my_follower.goal_pose[1],my_follower.goal_pose[2]); 
        ROS_INFO("State %f %f %f",my_follower.current_pose[0],
                  my_follower.current_pose[1],my_follower.current_pose[2]);
        */
        double result[8]={0.0};
        double trajectory[33]={0.0};
        int status=myMpcSolver.solve_my_mpc(current_robot_position, costmap_data, current_robot_goal, tracking_goal, result, trajectory);
        
        if (status > 0) {
            //ROS_INFO("Destroying solver object");
            myMpcSolver.reset_solver();
            myMpcSolver=my_NMPC_solver(5);
            ROS_INFO("Solver recreated");
        }

        if (status==4) {
              for (int i=0; i<4; i++) result[i] = 0.0;
        }

        double cgoal_trejectory[33];
        for (int i=0;i<11;i++) {
            cgoal_trejectory[3*i+0] = trajectory[3*i+0] + my_follower.current_pose[0];
            cgoal_trejectory[3*i+1] = trajectory[3*i+1] + my_follower.current_pose[1];
            cgoal_trejectory[3*i+2] = 0.0;
            //ROS_INFO("%f %f %f",cgoal_trejectory[3*i+0], cgoal_trejectory[3*i+1], cgoal_trejectory[3*i+2]);
        }
        nav_msgs::Path path;
        path.header.frame_id="odom";
        geometry_msgs::PoseStamped pose;
        
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "odom";
        for (int i=0;i<11;i++) {
            pose.pose.position.x = cgoal_trejectory[3*i+0];
            pose.pose.position.y = cgoal_trejectory[3*i+1];
            pose.pose.position.z = cgoal_trejectory[3*i+2];
            
            path.poses.push_back(pose);
        }
         
        low_level_trajectory.publish(path);
        std_msgs::Float64MultiArray wheel_command;
        wheel_command.data.push_back(result[0]);
        wheel_command.data.push_back(result[0]);
        wheel_command.data.push_back(result[1]);
        wheel_command.data.push_back(result[1]);
        motor_controller.publish(wheel_command);
        ROS_INFO("Time %f; Status %i; Error: %f",result[5], status, current_error);
    }
    //ROS_INFO("goal_status %i; current_error %f", my_follower.goal_status, current_error);
    ros::spinOnce();
    loop_rate.sleep();
  }
  //** end of Loop

  myfile.close();
  return 0;
}

