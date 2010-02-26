//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef PR2_ARM_IK_SOLVER_H
#define PR2_ARM_IK_SOLVER_H

#include <urdf/model.h>
#include <Eigen/Array>
#include <kdl/chainiksolver.hpp>
#include <pr2_arm_ik/pr2_arm_ik.h>
#include <pr2_arm_ik/pr2_arm_ik_utils.h>
#include <kinematics_msgs/GetKinematicTreeInfo.h>
#include <kinematics_msgs/PositionIKRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>

namespace pr2_arm_ik
{

  typedef std::pair<double, double> cost_pair;

  class PR2ArmIKSolver : public KDL::ChainIkSolverPos
  {
    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** @class
     *  @brief ROS/KDL based interface for the inverse kinematics of the PR2 arm
     *  @author Sachin Chitta <sachinc@willowgarage.com>
     *
     *  This class provides a ROS/KDL based interface to the inverse kinematics of the PR2 arm. It inherits from the KDL::ChainIkSolverPos class
     *  but also exposes additional functionality to return the multiple solutions from an inverse kinematics computation. It uses an instance of
     *  a ros::Node to find the robot description. It can thus be used only if the robot description is available on a ROS param server.
     *
     *  To use this wrapper, you must have a roscore running with a robot description available from the ROS param server. 
     */
    PR2ArmIKSolver(urdf::Model robot_model, 
                   std::string root_frame_name,
                   std::string tip_frame_name,
                   double search_discretization_angle, 
                   int free_angle);

    ~PR2ArmIKSolver(){};

    /** 
     * @brief A pointer to the inverse kinematics solver 
     */ 
    PR2ArmIK pr2_arm_ik_;

    /**
     * @brief Indicates whether the solver has been successfully initialized
     */
    bool active_;

    /**
     * @brief The KDL solver interface that is required to be implemented. NOTE: This method only returns a solution
     * if it exists for the free parameter value passed in. To search for a solution in the entire workspace use the CartToJntSearch 
     * method detailed below.
     *
     * @return < 0 if no solution is found
     * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
     * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A single inverse kinematic solution (if it exists).  
     */
    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);

    /**
     * @brief An extension of the KDL solver interface to return all solutions found. NOTE: This method only returns a solution 
     * if it exists for the free parameter value passed in. To search for a solution in the entire workspace use the CartToJntSearch 
     * method detailed below.
     *
     * @return < 0 if no solution is found
     * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
     * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     */
    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out);
  
     /**
     * @brief This method searches for and returns the first set of solutions it finds. 
     *
     * @return < 0 if no solution is found
     * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
     * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param timeout The amount of time (in seconds) to spend looking for a solution.
     */
    int CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out, const double &timeout);

     /**
     * @brief This method searches for and returns the closest solution to the initial guess in the first set of solutions it finds. 
     *
     * @return < 0 if no solution is found
     * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
     * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
     * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param timeout The amount of time (in seconds) to spend looking for a solution.
     */
    int CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray &q_out, const double &timeout);
    /**
       @brief A method to get chain information about the serial chain that the IK operates on 
       @param response This class gets populated with information about the joints that IK operates on, including joint names and limits.
    */
    void getChainInfo(kinematics_msgs::KinematicTreeInfo &response);

     /**
     * @brief This method searches for and returns the first solution it finds that also satisifies both user defined callbacks.
     *
     * @return < 0 if no solution is found
     * @param The kinematics request in kinematics_msgs::PositionIKRequest form
     * @param The set of links to check for collisions
     * @param The set of links to disable collision checks for
     * @param The robot state (use this to fill out state information for other joints)
     * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
     * @param desired_pose_callback A callback function to which the desired pose is passed in
     * @param solution_callback A callback function to which IK solutions are passed in
     */
    int CartToJntSearchWithCallbacks(const kinematics_msgs::PositionIKRequest &ik_request, 
                                     const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                     const motion_planning_msgs::RobotState &robot_state,
                                     const double &timeout,
                                     KDL::JntArray &q_out, 
                                     motion_planning_msgs::ArmNavigationErrorCodes &error_code,
                                     const boost::function<void(const kinematics_msgs::PositionIKRequest&, 
                                                                const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                                const motion_planning_msgs::RobotState &, motion_planning_msgs::ArmNavigationErrorCodes &)> &desired_pose_callback,
                                     const boost::function<void(const KDL::JntArray&, 
                                                                const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                                                                const motion_planning_msgs::RobotState &, motion_planning_msgs::ArmNavigationErrorCodes &)> &solution_callback);

    private:

    bool getCount(int &count, int max_count, int min_count);

    double search_discretization_angle_;

    int free_angle_;
  };
}
#endif// PR2_ARM_IK_SOLVER_H
