/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef PR2_ARM_IK_NODE_H
#define PR2_ARM_IK_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
#include <pr2_arm_ik/pr2_arm_ik_solver.h>
#include <tf_conversions/tf_kdl.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicTreeInfo.h>
#include <kinematics_msgs/GetCollisionFreePositionIK.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/shared_ptr.hpp>

namespace pr2_arm_ik
{
  class PR2ArmIKNode
  {
    public:

    /** @class
     *  @brief ROS/KDL based interface for the inverse kinematics of the PR2 arm
     *  @author Sachin Chitta <sachinc@willowgarage.com>
     *
     *  This class provides a ROS/KDL based interface to the inverse kinematics of the PR2 arm. It inherits from the KDL::ChainIkSolverPos class
     *  but also exposes additional functionality to return the multiple solutions from an inverse kinematics computation. It uses an instance of
     *  a ros::NodeHandle to find the robot description. It can thus be used only if the robot description is available on a ROS param server.
     *
     *  To use this wrapper, you must have a roscore running with a robot description available from the ROS param server. 
     */
    PR2ArmIKNode();

    virtual ~PR2ArmIKNode(){};

    /** 
     *  @brief Specifies if the node is active or not
     *  @return True if the node is active, false otherwise.
     */
    bool isActive();

    /**
     * @brief This is the basic IK service method that will compute and return an IK solution.
     * @param A request message. See service definition for GetPositionIK for more information on this message.
     * @param The response message. See service definition for GetPositionIK for more information on this message.
     */
    bool ikService(kinematics_msgs::GetPositionIK::Request &request, 
                   kinematics_msgs::GetPositionIK::Response &response);

    /**
     * @brief This is the basic kinematics info service that will return information about the kinematics node.
     * @param A request message. See service definition for GetKinematicTreeInfo for more information on this message.
     * @param The response message. See service definition for GetKinematicTreeInfo for more information on this message.
     */
    bool ikQuery(kinematics_msgs::GetKinematicTreeInfo::Request &request, 
                 kinematics_msgs::GetKinematicTreeInfo::Response &response);

    /**
     * @brief This is the basic forward kinematics service that will return information about the kinematics node.
     * @param A request message. See service definition for GetPositionFK for more information on this message.
     * @param The response message. See service definition for GetPositionFK for more information on this message.
     */
    bool fkService(kinematics_msgs::GetPositionFK::Request &request, 
                   kinematics_msgs::GetPositionFK::Response &response);

    protected:

    urdf::Model robot_model_;

    bool active_;

    int free_angle_;

    double search_discretization_;

    double cost_multiplier_;

    std::string ik_service_name_;

    std::string fk_service_name_;

    std::string ik_query_name_;

    ros::NodeHandle node_handle_, root_handle_;

    ros::ServiceServer ik_service_;

    ros::ServiceServer ik_collision_service_;

    ros::ServiceServer ik_service_with_cost_;

    ros::ServiceServer fk_service_;

    ros::ServiceServer ik_query_;

    ros::ServiceClient check_state_validity_client_;

    ros::ServiceClient get_state_cost_client_;

    boost::shared_ptr<pr2_arm_ik::PR2ArmIKSolver> pr2_arm_ik_solver_;

    tf::TransformListener tf_;

    std::string control_topic_name_;

    KDL::Frame pose_desired_;

    std::string root_name_;

    int dimension_;

    /*    bool ikServiceWithCollision(kinematics_msgs::GetCollisionFreePositionIK::Request &request, 
                                kinematics_msgs::GetCollisionFreePositionIK::Response &response);

    bool ikServiceMinimumCost(kinematics_msgs::GetCollisionFreePositionIK::Request &request, kinematics_msgs::GetCollisionFreePositionIK::Response &response);

    void collisionCheck(const KDL::JntArray &jnt_array, const std::vector<std::string> &joint_names, bool &check);

    void getCollisionCost(const KDL::JntArray &jnt_array, const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, bool &valid, double &cost);
    */
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;

    KDL::Chain kdl_chain_;

    kinematics_msgs::KinematicTreeInfo chain_info_;
  };
}

#endif
