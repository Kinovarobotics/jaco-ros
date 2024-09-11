#include <kinova_driver/joint_trajectory_action_server.h>

using namespace kinova;

JointTrajectoryActionController::JointTrajectoryActionController(ros::NodeHandle &n, std::string &robot_name):
    nh_(n),
    has_active_goal_(false)
{
    std::string robot_type = "";
    std::string address;

    address = "/" + robot_name + "_driver/robot_type";
    nh_.getParam(address,robot_type);
    if (robot_type == "")
    {
        ROS_ERROR_STREAM("Parameter "<<address<<" not found, make sure robot driver node is running");
    }

    address = "/" + robot_name + "/follow_joint_trajectory";
    action_server_follow_.reset(
                new FJTAS(nh_, address,
                          boost::bind(&JointTrajectoryActionController::goalCBFollow, this, _1),
                          boost::bind(&JointTrajectoryActionController::cancelCBFollow, this, _1),
                          false));

    ros::NodeHandle pn("~");

    int arm_joint_num = robot_type[3]-'0';
    joint_names_.resize(arm_joint_num);

    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_name + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);
    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        std::string ns = std::string("constraints/") + joint_names_[i];
        double g, t;
        pn.param(ns + "/goal", g, -1.0);
        pn.param(ns + "/trajectory", t, -1.0);
        goal_constraints_[joint_names_[i]] = g;
        trajectory_constraints_[joint_names_[i]] = t;
    }

    pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.00);


    pub_controller_command_ = nh_.advertise<trajectory_msgs::JointTrajectory>
            ("/"+ robot_name + "_driver/trajectory_controller/command", 1);
    sub_controller_state_ = nh_.subscribe("/" + robot_name + "_driver/trajectory_controller/state",
            1, &JointTrajectoryActionController::controllerStateCB, this);
    watchdog_timer_ = nh_.createTimer(ros::Duration(1.0),
                                      &JointTrajectoryActionController::watchdog, this);


    ros::Time started_waiting_for_controller = ros::Time::now();
    while (ros::ok() && !last_controller_state_)
    {
        ros::spinOnce();
        if (started_waiting_for_controller != ros::Time(0) &&
                ros::Time::now() > started_waiting_for_controller + ros::Duration(30.0))
        {
            ROS_WARN("Waited for the controller for 30 seconds, but it never showed up. Continue waiting the feedback of trajectory state on topic /trajectory_controller/state ...");
            started_waiting_for_controller = ros::Time(0);
        }
        ros::WallDuration(0.1).sleep();
    }


    action_server_follow_->start();
    ROS_INFO("Start Follow_Joint_Trajectory_Action server!");
    ROS_INFO("Waiting for an plan execution (goal) from Moveit");

}


JointTrajectoryActionController::~JointTrajectoryActionController()
{
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
}


static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
        if (count(b.begin(), b.end(), a[i]) != 1)
            return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
        if (count(a.begin(), a.end(), b[i]) != 1)
            return false;
    }

    return true;
}


void JointTrajectoryActionController::watchdog(const ros::TimerEvent &e)
{
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
        bool should_abort = false;
        if (!last_controller_state_)
        {
            should_abort = true;
            ROS_WARN("Aborting goal because we have never heard a controller state message.");
        }
        else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
        {
            should_abort = true;
            ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                     (now - last_controller_state_->header.stamp).toSec());
        }

        if (should_abort)
        {
            // Stops the controller.
            trajectory_msgs::JointTrajectory empty;
            empty.joint_names = joint_names_;
            pub_controller_command_.publish(empty);

            // Marks the current goal as aborted.
            active_goal_.setAborted();
            has_active_goal_ = false;
        }
    }
}


void JointTrajectoryActionController::goalCBFollow(FJTAS::GoalHandle gh)
{
    ROS_INFO("Joint_trajectory_action_server received goal!");

    // Ensures that the joints in the goal match the joints we are commanding.
    if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
        ROS_ERROR("Joints on incoming goal don't match our joints");
        gh.setRejected();
        return;
    }


    // Cancels the currently active goal.
    if (has_active_goal_)
    {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    first_fb_ = true;
    ROS_INFO("Joint_trajectory_action_server accepted goal!");


    // Sends the trajectory along to the controller
    current_traj_ = active_goal_.getGoal()->trajectory;
    pub_controller_command_.publish(current_traj_);
    ROS_INFO("Joint_trajectory_action_server published goal via command publisher!");
}


void JointTrajectoryActionController::cancelCBFollow(FJTAS::GoalHandle gh)
{
    if (active_goal_ == gh)
    {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }

}


void JointTrajectoryActionController::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
    ROS_INFO_ONCE("Joint_trajectory_action_server receive feedback of trajectory state from topic: /trajectory_controller/state");

    last_controller_state_ = msg;

    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
        return;
    if (current_traj_.points.empty())
        return;

    //joint trajectory message seems to have header.stamp = 0
    // using first feedback msg as guide for starting timestamp
    if (first_fb_)
    {
        start_time_ = msg->header.stamp;
        first_fb_ = false;
    }

    if (now - start_time_ < current_traj_.points[0].time_from_start)
    {
        return;
    }

    if (!setsEqual(joint_names_, msg->joint_names))
    {
        ROS_ERROR_ONCE("Joint names from the controller don't match our joint names.");
        return;
    }

    int last = current_traj_.points.size() - 1;
    ros::Time end_time = start_time_ + current_traj_.points[last].time_from_start;

    if (end_time - now < ros::Duration(goal_time_constraint_))
    {
        // Checks that we have ended inside the goal constraints
        bool inside_goal_constraints = true;
        bool inside_velocity_constraints = true;
        
        for (size_t i = 0; i < msg->joint_names.size() && inside_goal_constraints && inside_velocity_constraints; ++i)
        {
            // computing error from goal pose
            double abs_error = fabs(msg->actual.positions[i] - current_traj_.points[last].positions[i]);
            abs_error = abs_error - 2*M_PI*std::floor((abs_error+M_PI)/(2*M_PI)); // fix rotation to [-PI,PI)
            double goal_constraint = goal_constraints_[msg->joint_names[i]];
            if (goal_constraint >= 0 && abs_error > goal_constraint)
                inside_goal_constraints = false;
            // It's important to be stopped if that's desired.
            if ( !(msg->desired.velocities.empty()) && (fabs(msg->desired.velocities[i]) < 1e-6) )
            {
                if (fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
                    inside_velocity_constraints = false;
            }
        }

        if (inside_goal_constraints && inside_velocity_constraints)
        {
            active_goal_.setSucceeded();
            has_active_goal_ = false;
            first_fb_ = true;
        }
        else if (now - end_time < ros::Duration(goal_time_constraint_))
        {
            // Still have some time left to make it.
        }
        else
        {
            active_goal_.setSucceeded();
            has_active_goal_ = false;
            first_fb_ = true;
            // ROS_WARN("Aborting because we wound up outside the %s constraints", inside_velocity_constraints?"goal":"velocity");
            // for(int j = 0; j<7; j++){
            //     double abs_error; 
            //     if(!inside_goal_constraints){
            //         abs_error = fabs(msg->actual.positions[j] - current_traj_.points[last].positions[j]); 
            //         abs_error = abs_error - 2*M_PI*std::floor((abs_error+M_PI)/(2*M_PI)); // fix rotation to [-PI,PI)
            //     } else {
            //         abs_error = fabs(msg->actual.velocities[j] - stopped_velocity_tolerance_);
            //     }
            //     ROS_WARN_STREAM("Error for joint " << j+1 << ": " << abs_error);
            // }
            // active_goal_.setAborted();
            // has_active_goal_ = false;
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_joint_trajecotry_action_server");
    ros::NodeHandle node;

    // Retrieve the (non-option) argument:
     std::string robot_name = "";
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robot_name provided in the argument!" << std::endl;
        return -1;
    }
    else
    {
        robot_name = argv[argc-1];
    }
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::JointTrajectoryActionController jtac(node, robot_name);

    ros::waitForShutdown();
    return 0;
}