#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>

const double FINGER_MAX = 6400;

using namespace kinova;



tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}


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
    define_cartesian_pose();
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

    co_.id = "bot_obstacle";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "top_obstacle";
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

void PickPlace::add_complex_obstacle()
{
    clear_obstacle();

    // add obstacle between robot and object
    co_.id = "bot_obstacle";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    double box_h = 0.2;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box_h;

    co_.primitive_poses[0].position.x = 0;
    co_.primitive_poses[0].position.y = 0.3;
    co_.primitive_poses[0].position.z = box_h/2.0;
    co_.primitive_poses[0].orientation.w = 1.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "top_obstacle";
    co_.primitive_poses[0].position.z = box_h/2.0 + box_h + 0.4;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD box ");
    //      std::cin >> pause_;
}

void PickPlace::add_attached_obstacle()
{
    //once the object is know to be grasped
    //we remove obstacle from work scene
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //and then we declare it as an attached obstacle
    aco_.object.operation = moveit_msgs::CollisionObject::ADD;
    aco_.link_name = robot_type_ + "_end_effector";
    aco_.touch_links.push_back(robot_type_ + "_end_effector");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_1");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_2");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_3");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_1");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_2");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_3");
    pub_aco_.publish(aco_);
}

void PickPlace::add_target()
{
    //remove target_cylinder
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //add target_cylinder
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.04;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.6;
    co_.primitive_poses[0].position.z = 0.3;
    can_pose_.pose.position.x = co_.primitive_poses[0].position.x;
    can_pose_.pose.position.y = co_.primitive_poses[0].position.y;
    can_pose_.pose.position.z = co_.primitive_poses[0].position.z;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    aco_.object = co_;
    ros::WallDuration(0.1).sleep();
}

void PickPlace::check_collision()
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");

    collision_request.group_name = "arm";
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");

    // check contact
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 4: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it)
    {
        ROS_INFO("Contact between: %s and %s",
                 it->first.first.c_str(),
                 it->first.second.c_str());
    }

    // allowed collision matrix
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene_->getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for(it2 = collision_result.contacts.begin();
        it2 != collision_result.contacts.end();
        ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 5: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");
}


void PickPlace::define_cartesian_pose()
{
    tf::Quaternion q;
    
    // define start pose before grasp
    start_pose_.header.frame_id = "root";
    start_pose_.header.stamp = ros::Time::now();
    start_pose_.pose.position.x = 0.5;
    start_pose_.pose.position.y = -0.5;
    start_pose_.pose.position.z = 0.5;

    q = EulerZYZ_to_Quaternion(-M_PI/4, M_PI/2, M_PI);
    start_pose_.pose.orientation.x = q.x();
    start_pose_.pose.orientation.y = q.y();
    start_pose_.pose.orientation.z = q.z();
    start_pose_.pose.orientation.w = q.w();

    // define grasp pose
    grasp_pose_.header.frame_id = "root";
    grasp_pose_.header.stamp  = ros::Time::now();

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose_.pose.position.x = 0.0;
    grasp_pose_.pose.position.y = 0.6;
    grasp_pose_.pose.position.z = 0.3;

    q = EulerZYZ_to_Quaternion(M_PI/4, M_PI/2, M_PI);
    grasp_pose_.pose.orientation.x = q.x();
    grasp_pose_.pose.orientation.y = q.y();
    grasp_pose_.pose.orientation.z = q.z();
    grasp_pose_.pose.orientation.w = q.w();

    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    grasp_pose_= generate_gripper_align_pose(grasp_pose_, 0.03999, M_PI/4, M_PI/2, M_PI);
    pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, M_PI/4, M_PI/2, M_PI);
    postgrasp_pose_ = grasp_pose_;
    postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;

}

void PickPlace::define_joint_values()
{

    start_joint_.resize(joint_names_.size());
    //    getInvK(start_pose_, start_joint_);
    start_joint_[0] = 234.4 *M_PI/180.0;
    start_joint_[1] = 256.0 *M_PI/180.0;
    start_joint_[2] = 91.4 *M_PI/180.0;
    start_joint_[3] = 163.4 *M_PI/180.0;
    start_joint_[4] = 88.5 *M_PI/180.0;
    start_joint_[5] = 151.0 *M_PI/180.0;


    grasp_joint_.resize(joint_names_.size());
    //    getInvK(grasp_pose, grasp_joint_);
    grasp_joint_[0] = 144.0 *M_PI/180.0;
    grasp_joint_[1] = 256.5 *M_PI/180.0;
    grasp_joint_[2] = 91.3 *M_PI/180.0;
    grasp_joint_[3] = 163.8 *M_PI/180.0;
    grasp_joint_[4] = 88.5 *M_PI/180.0;
    grasp_joint_[5] = 151.3 *M_PI/180.0;

    pregrasp_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, pregrasp_joint_);
    pregrasp_joint_[0] = 145.4 *M_PI/180.0;
    pregrasp_joint_[1] = 253.7 *M_PI/180.0;
    pregrasp_joint_[2] = 67.0 *M_PI/180.0;
    pregrasp_joint_[3] = 151.0 *M_PI/180.0;
    pregrasp_joint_[4] = 118.5 *M_PI/180.0;
    pregrasp_joint_[5] = 141.7 *M_PI/180.0;

//    postgrasp_joint_ = pregrasp_joint_;
    postgrasp_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    postgrasp_joint_[0] = 144 *M_PI/180.0;
    postgrasp_joint_[1] = 249 *M_PI/180.0;
    postgrasp_joint_[2] = 88 *M_PI/180.0;
    postgrasp_joint_[3] = 165 *M_PI/180.0;
    postgrasp_joint_[4] = 83 *M_PI/180.0;
    postgrasp_joint_[5] = 151 *M_PI/180.0;
    
    
    home_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    home_joint_[0] = -85 *M_PI/180.0;
    home_joint_[1] = 168 *M_PI/180.0;
    home_joint_[2] = 57 *M_PI/180.0;
    home_joint_[3] = 241 *M_PI/180.0;
    home_joint_[4] = 83 *M_PI/180.0;
    home_joint_[5] = 76 *M_PI/180.0;
    
    PrG_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    PrG_joint_[0] = 3.0353;
    PrG_joint_[1] = 3.9773;
    PrG_joint_[2] = 1.7967;
    PrG_joint_[3] = 5.966;
    PrG_joint_[4] = 7.3266;
    PrG_joint_[5] = 1.0641;
    
    G_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    G_joint_[0] = 174 *M_PI/180.0;
    G_joint_[1] = 237 *M_PI/180.0;
    G_joint_[2] = 100 *M_PI/180.0;
    G_joint_[3] = 346 *M_PI/180.0;
    G_joint_[4] = 406 *M_PI/180.0;
    G_joint_[5] = 65 *M_PI/180.0;
    
    PoG_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    PoG_joint_[0] = 176 *M_PI/180.0;
    PoG_joint_[1] = 219 *M_PI/180.0;
    PoG_joint_[2] = 114 *M_PI/180.0;
    PoG_joint_[3] = 334 *M_PI/180.0;
    PoG_joint_[4] = 445 *M_PI/180.0;
    PoG_joint_[5] = 52 *M_PI/180.0;
    
    int_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    int_joint_[0] = 265 *M_PI/180.0;
    int_joint_[1] = 197 *M_PI/180.0;
    int_joint_[2] = 73 *M_PI/180.0;
    int_joint_[3] = 346 *M_PI/180.0;
    int_joint_[4] = 419 *M_PI/180.0;
    int_joint_[5] = 54 *M_PI/180.0;
    
    PrR_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    PrR_joint_[0] = 361 *M_PI/180.0;
    PrR_joint_[1] = 214 *M_PI/180.0;
    PrR_joint_[2] = 103 *M_PI/180.0;
    PrR_joint_[3] = 340 *M_PI/180.0;
    PrR_joint_[4] = 435 *M_PI/180.0;
    PrR_joint_[5] = 53 *M_PI/180.0;
    
    R_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    R_joint_[0] = 361 *M_PI/180.0;
    R_joint_[1] = 233 *M_PI/180.0;
    R_joint_[2] = 92 *M_PI/180.0;
    R_joint_[3] = 349 *M_PI/180.0;
    R_joint_[4] = 401 *M_PI/180.0;
    R_joint_[5] = 64 *M_PI/180.0;
    
    PoR_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    PoR_joint_[0] = 361 *M_PI/180.0;
    PoR_joint_[1] = 221 *M_PI/180.0;
    PoR_joint_[2] = 98 *M_PI/180.0;
    PoR_joint_[3] = 342 *M_PI/180.0;
    PoR_joint_[4] = 423 *M_PI/180.0;
    PoR_joint_[5] = 61 *M_PI/180.0;
}


/**
 * @brief PickPlace::generate_gripper_align_pose
 * @param targetpose_msg pick/place pose (object location): where gripper close/open the fingers (grasp/release the object). Only position information is used.
 * @param dist distance of returned pose to targetpose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
 * @return a pose defined in a spherical coordinates where origin is located at the target pose. Normally it is a pre_grasp/post_realease pose, where gripper axis (last joint axis) is pointing to the object (target_pose).
 */
geometry_msgs::PoseStamped PickPlace::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.pose.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pose_msg: x " << pose_msg.pose.position.x  << ", y " << pose_msg.pose.position.y  << ", z " << pose_msg.pose.position.z  << ", qx " << pose_msg.pose.orientation.x  << ", qy " << pose_msg.pose.orientation.y  << ", qz " << pose_msg.pose.orientation.z  << ", qw " << pose_msg.pose.orientation.w );

    return pose_msg;

}


void PickPlace::setup_constrain(geometry_msgs::Pose target_pose, bool orientation, bool position)
{
    if ( (!orientation) && (!position) )
    {
        ROS_WARN("Neither orientation nor position constrain applied.");
        return;
    }

    moveit_msgs::Constraints grasp_constrains;

    // setup constrains
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = robot_type_ + "_end_effector";
    ocm.header.frame_id = "root";
    ocm.orientation = target_pose.orientation;
    ocm.absolute_x_axis_tolerance = 2*M_PI;
    ocm.absolute_y_axis_tolerance = 2*M_PI;
    ocm.absolute_z_axis_tolerance = M_PI/10;
    ocm.weight = 0.5;
    if (orientation)
    {
        grasp_constrains.orientation_constraints.push_back(ocm);
    }


    /* Define position constrain box based on current pose and target pose. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    // group_->getCurrentPose() does not work.
//    current_pose_ = group_->getCurrentPose();
    geometry_msgs::Pose current_pose;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
//        ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "current_pose_: x " << current_pose_.pose.position.x  << ", y " << current_pose_.pose.position.y  << ", z " << current_pose_.pose.position.z  << ", qx " << current_pose_.pose.orientation.x  << ", qy " << current_pose_.pose.orientation.y  << ", qz " << current_pose_.pose.orientation.z  << ", qw " << current_pose_.pose.orientation.w );
    }

    double constrain_box_scale = 2.0;
    primitive.dimensions[0] = constrain_box_scale * std::abs(target_pose.position.x - current_pose.position.x);
    primitive.dimensions[1] = constrain_box_scale * std::abs(target_pose.position.y - current_pose.position.y);
    primitive.dimensions[2] = constrain_box_scale * std::abs(target_pose.position.z - current_pose.position.z);

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    // place between start point and goal point.
    box_pose.position.x = (target_pose.position.x + current_pose.position.x)/2.0;
    box_pose.position.y = (target_pose.position.y + current_pose.position.y)/2.0;
    box_pose.position.z = (target_pose.position.z + current_pose.position.z)/2.0;

    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = robot_type_+"_end_effector";
    pcm.header.frame_id = "root";
    pcm.constraint_region.primitives.push_back(primitive);
    pcm.constraint_region.primitive_poses.push_back(box_pose);
    pcm.weight = 0.5;
    if(position)
    {
        grasp_constrains.position_constraints.push_back(pcm);
    }

    group_->setPathConstraints(grasp_constrains);


//    // The bellowing code is just for visulizing the box and check.
//    // Disable this part after checking.
//    co_.id = "check_constrain";
//    co_.operation = moveit_msgs::CollisionObject::REMOVE;
//    pub_co_.publish(co_);

//    co_.operation = moveit_msgs::CollisionObject::ADD;
//    co_.primitives.push_back(primitive);
//    co_.primitive_poses.push_back(box_pose);
//    pub_co_.publish(co_);
//    planning_scene_msg_.world.collision_objects.push_back(co_);
//    planning_scene_msg_.is_diff = true;
//    pub_planning_scene_diff_.publish(planning_scene_msg_);
//    ros::WallDuration(0.1).sleep();
}

void PickPlace::check_constrain()
{
    moveit_msgs::Constraints grasp_constrains = group_->getPathConstraints();
    bool has_constrain = false;
    ROS_INFO("check constrain result: ");
    if (!grasp_constrains.orientation_constraints.empty())
    {
        has_constrain = true;
        ROS_INFO("Has orientation constrain. ");
    }
    if(!grasp_constrains.position_constraints.empty())
    {
        has_constrain = true;
        ROS_INFO("Has position constrain. ");
    }
    if(has_constrain == false)
    {
        ROS_INFO("No constrain. ");
    }
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
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();
            if (pause_ == "r" || pause_ == "R" )
            {
                replan = true;
            }
            else
            {
                replan = false;
            }
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
        if (pause_ == "e" || pause_ == "E")
        {
            group.execute(my_plan);
        }
    }
    ros::WallDuration(1.0).sleep();
}


bool PickPlace::my_pick()
{
    clear_workscene();
    ros::WallDuration(1.0).sleep();
    build_workscene();
    ros::WallDuration(1.0).sleep();

    ROS_INFO_STREAM("Press any key to send robot to home position ...");
    std::cin >> pause_;
    group_->clearPathConstraints();
    group_->setNamedTarget("Home");
    evaluate_plan(*group_);

    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    ///////////////////////////////////////////////////////////
    //// joint space without obstacle
    ///////////////////////////////////////////////////////////

    ROS_INFO_STREAM("Joint space motion planing without obstacle");
    ROS_INFO_STREAM("Demonstrates moving robot from one joint position to another");
    ROS_INFO_STREAM("Planning to go to start pose ...");
    //group_->setJointValueTarget(home_joint_);
    //evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to pre-grasp joint pose ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PrG_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(G_joint_);
    	}
    evaluate_plan(*group_);
    

    ROS_INFO_STREAM("Grasping ...");
    gripper_action(0.5*FINGER_MAX); // partially close

    ROS_INFO_STREAM("Planning to go to post-grasp position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PoG_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to intermediate position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(int_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Planning to go to pre-release position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PrR_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Planning to go to release position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(R_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing ...");
    gripper_action(0.0); // full open
    
    ROS_INFO_STREAM("Planning to go to post-release position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PoR_joint_);
    	}
    evaluate_plan(*group_);


    ///////////////////////////////////////////////////////////
    //// joint space with obstacle
    ///////////////////////////////////////////////////////////
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("Press any key to start motion plan in joint space with obstacle ...");
    std::cin >> pause_;
    add_obstacle();
    ROS_INFO_STREAM("Planning to go to pre-grasp joint pose ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PrG_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(G_joint_);
    	}
    evaluate_plan(*group_);
    

    ROS_INFO_STREAM("Grasping ...");
    gripper_action(0.5*FINGER_MAX); // partially close

    ROS_INFO_STREAM("Planning to go to post-grasp position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PoG_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to intermediate position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(int_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Planning to go to pre-release position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PrR_joint_);
    	}
    evaluate_plan(*group_);
    
    ROS_INFO_STREAM("Planning to go to release position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(R_joint_);
    	}
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing ...");
    gripper_action(0.0); // full open
    
    ROS_INFO_STREAM("Planning to go to post-release position ...");
    for(int i = 0; i < 10; i++) {
    	group_->setJointValueTarget(PoR_joint_);
    	}
    evaluate_plan(*group_);

    '''
    ///////////////////////////////////////////////////////////
    //// Cartesian space without obstacle
    ///////////////////////////////////////////////////////////
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("Motion planning in cartesian space without obstacles ...");
    clear_workscene();
    build_workscene();
    add_target();
    ros::WallDuration(0.1).sleep();

    ROS_INFO_STREAM("Press any key to move to start pose ...");
    std::cin >> pause_;
    group_->setPoseTarget(start_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(pregrasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(grasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Grasping ...");
    add_attached_obstacle();
    gripper_action(0.75*FINGER_MAX); // partially close

    ROS_INFO_STREAM("Planning to return to start position  ...");
    group_->setPoseTarget(start_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing gripper ...");

    gripper_action(0.0); // full open


    ///////////////////////////////////////////////////////////
    //// Cartesian space with obstacle
    ///////////////////////////////////////////////////////////
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("Press any key to start motion plan in cartesian space with obstacle ...");
    std::cin >> pause_;
    clear_workscene();
    build_workscene();
    add_target();
    add_complex_obstacle();
    ros::WallDuration(1.0).sleep();

    //ROS_INFO_STREAM("Planning to go to start position ...");
    //group_->setPoseTarget(start_pose_);
    //evaluate_plan(*group_);

    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(pregrasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to grasp position ...");
    group_->setPoseTarget(grasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Press any key to grasp ...");
    std::cin >> pause_;
    add_attached_obstacle();
    gripper_action(0.75*FINGER_MAX); // partially close

    ROS_INFO_STREAM("Planning to go to start position  ...");
    group_->setPoseTarget(start_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing gripper ...");
    gripper_action(0.0); // full open

    ROS_INFO_STREAM("Press any key to start motion planning with orientation constrains ...");
    ROS_INFO_STREAM("Constraints are setup so that the robot keeps the target vertical");
    std::cin >> pause_;
    clear_workscene();
    build_workscene();
    add_target();

    //ROS_INFO_STREAM("Planning to go to start position ...");
    //group_->setPoseTarget(start_pose_);
    //evaluate_plan(*group_);

    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(pregrasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to grasp position ...");
    group_->setPoseTarget(grasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Press any key to grasp ...");
    std::cin >> pause_;

    add_attached_obstacle();
    gripper_action(0.75*FINGER_MAX); // partially close

    setup_constrain(grasp_pose_.pose, true, false);

    ROS_INFO_STREAM("Planning to go to start position  ...");
    group_->setPoseTarget(start_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing gripper ...");
    gripper_action(0.0); // full open

//  If need to double check if reach target position.
//    replacement of group_->getCurrentJointValues();
//    { // scope for mutex update
//        boost::mutex::scoped_lock lock_state(mutex_state_);
//        sensor_msgs::JointState copy_state = current_state_;
//    }
//    replacement of group_->getCurrentPose();
//    { // scope for mutex update
//        boost::mutex::scoped_lock lock_state(mutex_state_);
//        geometry_msgs::PoseStamped copy_pose = current_pose_;
//    }
'''

    clear_workscene();
    ROS_INFO_STREAM("Press any key to quit ...");
    std::cin >> pause_;
    return true;
}


void PickPlace::getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value)
{
    // TODO: transform cartesian command to joint space, and alway motion plan in joint space.
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::PickPlace pick_place(node);

    ros::spin();
    return 0;
}
