#ifndef JACO_DRIVER_JACO_IK_SOLVER_H_
#define JACO_DRIVER_JACO_IK_SOLVER_H_

#include <string>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Pose.h>

#include "jaco_driver/jaco_types.h"

namespace urdf
{
    class Model;
}

namespace KDL
{
    class ChainFkSolverPos;
    class ChainIkSolverVel;
    class ChainIkSolverPos;
    
    class ChainJntToJacSolver;
}

namespace jaco
{

class JacoIKSolver
{

public:

    JacoIKSolver();

    virtual ~JacoIKSolver();

    bool initFromURDF(const std::string urdf, const std::string root_name,
                      const std::string tip_name, unsigned int max_iter, std::string error);
    
    geometry_msgs::Pose jointsToCartesian(JacoAngles JAngle);

    bool cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out);

    bool cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out, const KDL::JntArray& q_seed);
    
    KDL::Jacobian jointToJacobian(JacoAngles);

    inline const KDL::JntArray& jointLowerLimits() const { return q_min_; }

    inline const KDL::JntArray& jointUpperLimits() const { return q_max_; }

    inline const std::vector<std::string>& jointNames() const { return joint_names_; }

    inline unsigned int numJoints() const { return joint_names_.size(); }

private:

    //!Object to describe the (serial) kinematic chain
    KDL::Chain chain_;

    KDL::JntArray q_min_, q_max_, q_seed_;

    std::vector<std::string> joint_names_;
	
    // Solvers
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_solver_;
    boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos> ik_solver_;
    
    
    boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    


};

}

#endif
