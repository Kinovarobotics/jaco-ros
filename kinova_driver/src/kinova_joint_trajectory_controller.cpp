#include <kinova_driver/kinova_joint_trajectory_controller.h>
#include <angles/angles.h>
#include <ros/console.h>

using namespace kinova;

JointTrajectoryController::JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle& n):
    kinova_comm_(kinova_comm),
    nh_(n)
{
    ros::NodeHandle pn("~");
    std::string robot_type;
    nh_.param<std::string>("robot_name",prefix_,"j2n6s300");
    nh_.param<std::string>("robot_type",robot_type,"j2n6s300");
    number_joint_ =robot_type[3] - '0';

    sub_command_ = nh_.subscribe("trajectory_controller/command", 1, &JointTrajectoryController::commandCB, this);

    pub_joint_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("trajectory_controller/state", 1);
    pub_joint_velocity_ = pn.advertise<kinova_msgs::JointVelocity>("in/joint_velocity", 2);

    traj_frame_id_ = "root";
    joint_names_.resize(number_joint_);
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = prefix_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }

    timer_pub_joint_vel_ = nh_.createTimer(ros::Duration(0.0085), &JointTrajectoryController::pub_joint_vel, this, false, false);
    terminate_thread_ = false;

    thread_update_state_ = new boost::thread(boost::bind(&JointTrajectoryController::update_state, this));

    traj_feedback_msg_.joint_names.resize(joint_names_.size());
    traj_feedback_msg_.desired.positions.resize(joint_names_.size());
    traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
    traj_feedback_msg_.actual.positions.resize(joint_names_.size());
    traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
    traj_feedback_msg_.error.positions.resize(joint_names_.size());
    traj_feedback_msg_.error.velocities.resize(joint_names_.size());
    traj_feedback_msg_.joint_names = joint_names_;

    // counter in the timer to publish joint velocity command: pub_joint_vel()
    traj_command_points_index_ = 0;
    for(int i=0; i<num_possible_joints; i++) current_velocity_command[i] = 0;
}

JointTrajectoryController::~JointTrajectoryController()
{
    ROS_WARN("destruction entered!");
    {
        boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
        terminate_thread_ = true;
    }

    sub_command_.shutdown();
    pub_joint_feedback_.shutdown();
    pub_joint_velocity_.shutdown();

    timer_pub_joint_vel_.stop();
    thread_update_state_->join();
    delete thread_update_state_;
}



void JointTrajectoryController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg)
{
    bool command_abort = false;

    traj_command_points_ = traj_msg->points;
    ROS_INFO_STREAM_NAMED("Trajectory controller", "Receive trajectory with points number: " << traj_command_points_.size());
    if(traj_command_points_.empty()) return;    // abort if 
    // Map the index in joint_names and the msg
    std::vector<int> lookup(number_joint_, -1);

    for (size_t j = 0; j<number_joint_; j++)
    {
        for (size_t k = 0; k<traj_msg->joint_names.size(); k++)
            if(traj_msg->joint_names[k] == joint_names_[j]) // find joint_j in msg;
            {
                lookup[j] = k;
                break;
            }

        if (lookup[j] == -1) // if joint_j not found in msg;
        {
            std::string error_msg = "Joint name : " + joint_names_[j] + " not found in the msg.";
            ROS_ERROR("%s", error_msg.c_str());
            command_abort = true;
            return;
        }
    }

    // check msg validation
    for (size_t j = 0; j<traj_command_points_.size(); j++)
    {
        // position should not be empty
        if (traj_command_points_[j].positions.empty()) // find joint_j in msg;
        {
            ROS_ERROR_STREAM("Positions in trajectory command cannot be empty at point: " << j);
            command_abort = true;
            break;
        }
        // position size match
        if (traj_command_points_[j].positions.size() != number_joint_)
        {
            ROS_ERROR_STREAM("Positions at point " << j << " has size " << traj_command_points_[j].positions.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }
        // velocity should not be empty
        if (traj_command_points_[j].velocities.empty()) 
        {
            ROS_ERROR_STREAM("Velocities in trajectory command cannot be empty at point: " << j);
            command_abort = true;
            break;
        }
        // if velocity provided, size match
        if (traj_command_points_[j].velocities.size() != number_joint_)
        {
            ROS_ERROR_STREAM("Velocities at point " << j << " has size " << traj_command_points_[j].velocities.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }
    }

    if(command_abort)
        return;

    // store angle velocity command sent to robot
    kinova_angle_command_.resize(traj_command_points_.size());
    for (size_t i = 0; i<traj_command_points_.size(); i++)
    {
        kinova_angle_command_[i].InitStruct(); // initial joint velocity to zeros.
        for(int actuator = 0; actuator < number_joint_; actuator++)
        {
            kinova_angle_command_[i][actuator] = traj_command_points_[i].velocities[actuator] *180/M_PI;
        }
    }

    std::vector<double> durations(traj_command_points_.size(), 0.0); // computed by time_from_start
    double trajectory_duration = traj_command_points_[0].time_from_start.toSec();

    durations[0] = trajectory_duration;

    for (int i = 1; i<traj_command_points_.size(); i++)
    {
        durations[i] = (traj_command_points_[i].time_from_start - traj_command_points_[i-1].time_from_start).toSec();
        trajectory_duration += durations[i];
    }

    // start timer thread to publish joint velocity command
    time_pub_joint_vel_ = ros::Time::now();
    timer_pub_joint_vel_.start();
}


void JointTrajectoryController::pub_joint_vel(const ros::TimerEvent&)
{
    // send out each velocity command with corresponding duration delay.
    kinova_msgs::JointVelocity joint_velocity_msg;

    if (traj_command_points_index_ <  kinova_angle_command_.size() && ros::ok())
    {
        const ros::Duration current_time_from_start = ros::Time::now() - time_pub_joint_vel_;
        // check for remaining motion time if in last command
        if(traj_command_points_index_ == kinova_angle_command_.size()-1)
        {
            const double current_time = current_time_from_start.toSec();
            for(int i=0; i<number_joint_; i++)
            {
                if(current_time > remaining_motion_time[i]) 
                    current_velocity_command[i] = 0;
            }
        }
        
        joint_velocity_msg.joint1 = current_velocity_command[0];
        joint_velocity_msg.joint2 = current_velocity_command[1];
        joint_velocity_msg.joint3 = current_velocity_command[2];
        joint_velocity_msg.joint4 = current_velocity_command[3];
        joint_velocity_msg.joint5 = current_velocity_command[4];
        joint_velocity_msg.joint6 = current_velocity_command[5];
        joint_velocity_msg.joint7 = current_velocity_command[6];

        pub_joint_velocity_.publish(joint_velocity_msg);

        if( current_time_from_start >= traj_command_points_[traj_command_points_index_].time_from_start)
        {
            ROS_INFO_STREAM("Moved to point " << traj_command_points_index_++);
            for(int i=0; i<num_possible_joints; i++) // store next angle commands per joint
                current_velocity_command[i] = kinova_angle_command_[traj_command_points_index_][i];

            // if the last command is reached, calculate remaining motion time
            if(traj_command_points_index_ == kinova_angle_command_.size()-1)
            {
                const double t1 = traj_command_points_[traj_command_points_index_ -1].time_from_start.toSec();
                for(int i=0; i<number_joint_; i++)
                {
                    current_velocity_command[i] = kinova_angle_command_[traj_command_points_index_ - 1][i];
                    const double position_delta = traj_command_points_[traj_command_points_index_].positions[i] - traj_command_points_[traj_command_points_index_-1].positions[i];
                    remaining_motion_time[i] = t1 + position_delta / (current_velocity_command[i] * M_PI / 180.);
                }
            }
        }
    }
    else // if come accross all the points, then stop timer.
    {
        joint_velocity_msg.joint1 = 0;
        joint_velocity_msg.joint2 = 0;
        joint_velocity_msg.joint3 = 0;
        joint_velocity_msg.joint4 = 0;
        joint_velocity_msg.joint5 = 0;
        joint_velocity_msg.joint6 = 0;
        joint_velocity_msg.joint7 = 0;
        pub_joint_velocity_.publish(joint_velocity_msg);

        traj_command_points_.clear();
        for(int i=0; i<num_possible_joints; i++) current_velocity_command[i] = 0;

        traj_command_points_index_ = 0;
        timer_pub_joint_vel_.stop();
    }
}

void JointTrajectoryController::update_state()
{
    ros::Rate update_rate(10);
    while (nh_.ok())
    {
        // check if terminate command is sent from main thread
        {
            boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
            if (terminate_thread_)
            {
                break;
            }
        }

        traj_feedback_msg_.header.frame_id = traj_frame_id_;
        traj_feedback_msg_.header.stamp = ros::Time::now();
        KinovaAngles current_joint_angles;
        KinovaAngles current_joint_velocity;
        AngularPosition current_joint_command;

        kinova_comm_.getAngularCommand(current_joint_command);
        kinova_comm_.getJointAngles(current_joint_angles);
        kinova_comm_.getJointVelocities(current_joint_velocity);

        for(int actuator = 0; actuator < number_joint_; actuator++)
        {
            traj_feedback_msg_.desired.positions[actuator] = current_joint_command.Actuators[actuator] *M_PI/180;
            traj_feedback_msg_.actual.positions[actuator] = current_joint_angles[actuator] *M_PI/180;
            traj_feedback_msg_.actual.velocities[actuator] = current_joint_velocity[actuator] *M_PI/180;
        }

        for (size_t j = 0; j<joint_names_.size(); j++)
        {
            traj_feedback_msg_.error.positions[j] = traj_feedback_msg_.actual.positions[j] - traj_feedback_msg_.desired.positions[j];
        }

        pub_joint_feedback_.publish(traj_feedback_msg_);
        update_rate.sleep();
    }
}