#include <pick_place.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>
#include <csignal>
#include <unistd.h>

const double FINGER_MAX = 6400;

using namespace kinova;

PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh)
{
//    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//    {
//        ros::console::notifyLoggerLevelsChanged();
//    }

    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300");
    nh_.param<bool>("/robot_connected",robot_connected_,true);

    if (robot_connected_)
    {
        //sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state", 1, &PickPlace::get_current_state, this);
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ +"_driver/out/tool_pose", 1, &PickPlace::get_current_pose, this);
    }

    // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    // construct a `PlanningScene` that maintains the state of the world (including the robot).
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

//    //  every time need retrive current robot state, do the following.
//    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
//    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup("arm");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
    while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3]-'0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }

    // set pre-defined joint and pose values.
    define_joint_values();

    // pick process
    result_ = false;
    my_pick();
}

PickPlace::~PickPlace()
{
    // shut down pub and subs
    //sub_joint_.shutdown();
    //sub_pose_.shutdown();
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}

void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

/**
 * @brief PickPlace::gripper_action
 * @param gripper_rad close for 6400 and open for 0.0
 * @return true is gripper motion reaches the goal
 */
bool PickPlace::gripper_action(double finger_turn)
{
    if(robot_connected_ == false)
    {
        if (finger_turn>0.5*FINGER_MAX)
        {
          gripper_group_->setNamedTarget("Close");
        }
        else
        {
          gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

void PickPlace::clear_workscene()
{
    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove partition
    co_.id = "partition";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove shelf
    co_.id = "shelf";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove cable
    co_.id = "cable";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove light
    co_.id = "light";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove support_legs
    co_.id = "support_legs";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove beam
    co_.id = "beam";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    // remove division
    co_.id = "division";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // remove target
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //remove attached target
    aco_.object.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_aco_.publish(aco_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    
    clear_obstacle();
}

void PickPlace::build_workscene()
{
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();

    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // add table
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.99;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.405;
    co_.primitive_poses[0].position.z = -0.03/2.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove partition
    co_.id = "partition";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add partition
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.03;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.96;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.90;
    co_.primitive_poses[0].position.z = 0.48;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove shelf
    co_.id = "shelf";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add shelf
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.38;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.07;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.71;
    co_.primitive_poses[0].position.z = 0.705;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove cable
    co_.id = "cable";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add cable
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.09;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.18;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.855;
    co_.primitive_poses[0].position.z = 0.09;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove light
    co_.id = "light";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add light
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.575;
    co_.primitive_poses[0].position.z = 0.645;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove support_legs
    co_.id = "support_legs";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add support_legs
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.10;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.70;
    co_.primitive_poses[0].position.x = -0.375;
    co_.primitive_poses[0].position.y = 0.0;
    co_.primitive_poses[0].position.z = -0.35;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove division
    co_.id = "division";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add division
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.04;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.30;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.20;
    co_.primitive_poses[0].position.x = -0.28;
    co_.primitive_poses[0].position.y = 0.75;
    co_.primitive_poses[0].position.z = 0.58;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    
    //remove beam
    co_.id = "beam";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    
    //add beam
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.04;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.04;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.0;
    co_.primitive_poses[0].position.z = -0.04;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();}

void PickPlace::clear_obstacle()
{
    co_.id = "box";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove box ");
    //      std::cin >> pause_;
}
   
void PickPlace::add_obstacle()
{
    clear_obstacle();

    co_.id = "box";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.56;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.43;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.09;

    co_.primitive_poses[0].position.x = 0.10;
    co_.primitive_poses[0].position.y = 0.285;
    co_.primitive_poses[0].position.z = 0.045;

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
}

void PickPlace::define_joint_values()
{
    point1_joint_.resize(joint_names_.size());
    point1_joint_[0] = 2 *M_PI/180.0;
    point1_joint_[1] = 218 *M_PI/180.0;
    point1_joint_[2] = 73 *M_PI/180.0;
    point1_joint_[3] = 359 *M_PI/180.0;
    point1_joint_[4] = -318 *M_PI/180.0;
    point1_joint_[5] = -2 *M_PI/180.0;
    
    point2_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    point2_joint_[0] = -4 *M_PI/180.0;
    point2_joint_[1] = 236 *M_PI/180.0;
    point2_joint_[2] = 119 *M_PI/180.0;
    point2_joint_[3] = 345 *M_PI/180.0;
    point2_joint_[4] = -285 *M_PI/180.0;
    point2_joint_[5] = -3 *M_PI/180.0;
    
    point3_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    point3_joint_[0] = 21 *M_PI/180.0;
    point3_joint_[1] = 247 *M_PI/180.0;
    point3_joint_[2] = 153 *M_PI/180.0;
    point3_joint_[3] = 333 *M_PI/180.0;
    point3_joint_[4] = -256 *M_PI/180.0;
    point3_joint_[5] = -11 *M_PI/180.0;
    
    point4_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    point4_joint_[0] = 26 *M_PI/180.0;
    point4_joint_[1] = 224 *M_PI/180.0;
    point4_joint_[2] = 96 *M_PI/180.0;
    point4_joint_[3] = 351 *M_PI/180.0;
    point4_joint_[4] = -299 *M_PI/180.0;
    point4_joint_[5] = -1 *M_PI/180.0;
    
    point5_joint_.resize(joint_names_.size());
    point5_joint_[0] = -9 *M_PI/180.0;
    point5_joint_[1] = 225 *M_PI/180.0;
    point5_joint_[2] = 73 *M_PI/180.0;
    point5_joint_[3] = 359 *M_PI/180.0;
    point5_joint_[4] = -318 *M_PI/180.0;
    point5_joint_[5] = -2 *M_PI/180.0;
}

void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group)
{
    bool replan = true;
    int count = 0;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (replan == true && ros::ok())
    {
        // reset flag for replan
        count = 0;
        result_ = false;

        // try to find a success plan.
        double plan_time;
        while (result_ == false && count < 5)
        {
            count++;
            plan_time = 20+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result_ = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result_ == true)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result_ == true)
    {
        group.execute(my_plan);
    }
    ros::WallDuration(1.0).sleep();
}


bool PickPlace::my_pick()
{
    clear_workscene();
    build_workscene();
    
    group_->clearPathConstraints();
    group_->setNamedTarget("Home");
    evaluate_plan(*group_);

    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    ///////////////////////////////////////////////////////////
    //// joint space obstacle
    ///////////////////////////////////////////////////////////

    ROS_INFO_STREAM("DEMO OF REPEATED PICK PLACE OPERATION");
    ROS_INFO_STREAM("Demonstrates moving robot from one joint position to another");
    add_obstacle();
    
    
    ROS_INFO_STREAM("Grasping ...");
    gripper_action(0.95*FINGER_MAX); // partially close
    
    ROS_INFO_STREAM("Planning to go to 1 joint pose ...");
    for(int i = 0; i < 1; i++) {
    	group_->setJointValueTarget(point1_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching 2 position ...");
    for(int i = 0; i < 1; i++) {
    	group_->setJointValueTarget(point2_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to 3 position ...");
    for(int i = 0; i < 1; i++) {
    	group_->setJointValueTarget(point3_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Planning to go to 4 position ...");
    for(int i = 0; i < 1; i++) {
    	group_->setJointValueTarget(point4_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Planning to go to 1 position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(point5_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Releasing ...");
    gripper_action(0.0); // full open
    
    group_->setNamedTarget("Home");
    evaluate_plan(*group_);

    return true;
}

volatile sig_atomic_t flag = 0;

void my_function(int sig){ // can be called asynchronously
  flag = 1; // set flag
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    signal(SIGINT, my_function); 
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    while(1){
        if (flag){
            std::cout << "Interrupted by Keyboard" << std::endl;
            break;
        }
        kinova::PickPlace pick_place(node);
    }
    ros::spin();
    return 0;
}
