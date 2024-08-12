#ifndef KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_JOINT_TRAJECTORY_CONTROLLER_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Duration.h>
#include <kinova_msgs/JointVelocity.h>
#include <boost/thread.hpp>

#include "kinova_ros_types.h"
#include "kinova_comm.h"
#include "kinova_api.h"

namespace kinova
{

class JointTrajectoryController
{
public:
    JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle &n);
    ~JointTrajectoryController();


private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_command_; // command subscriber listening to 'trajectory_controller/command'
    ros::Publisher pub_joint_feedback_; // feedback publisher sending to 'trajectory_controller/state'
    ros::Publisher pub_joint_velocity_; // velocity publisher sending to 'in/joint_velocity'

    ros::Time time_pub_joint_vel_; // time of timer thread start to publish joint velocity command

    ros::Timer timer_pub_joint_vel_; // timer to publish joint velocities
    boost::mutex terminate_thread_mutex_;
    boost::thread* thread_update_state_;
    bool terminate_thread_;

    KinovaComm kinova_comm_; // Communication pipeline for the arm

    std::string traj_frame_id_; // origin of trajectory transforms
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_command_points_; // trajectory points from most recent trajectory
    control_msgs::FollowJointTrajectoryFeedback traj_feedback_msg_; // trajectory feedback message

    // stores the command to send to robot, in Kinova type (KinovaAngles)
    std::vector<KinovaAngles> kinova_angle_command_; // intermediate command storage

    uint number_joint_; // number of joints of the robot
    int traj_command_points_index_; // current index in traj_command_points_, defined by time
    std::vector<std::string> joint_names_; // names of the joints
    std::string prefix_; // robot name prefix
    const static int num_possible_joints = 7; // number of possible joints supported by the system
    float current_velocity_command[num_possible_joints]; // storage array to keep calculated velocity commands
    double remaining_motion_time[num_possible_joints]; // time of motion remaining for each joint during the last command

    // call back function when receive a trajectory command
    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg);

    // reflash the robot state and publish the joint state: either by timer or thread
    void update_state(); // by thread

    // regularly publish the velocity commands of the trajectory 
    void pub_joint_vel(const ros::TimerEvent&); // by timer

};

}

#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H