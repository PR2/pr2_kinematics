/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <pr2_arm_kinematics/pr2_arm_kinematics.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include "ros/ros.h"
#include <algorithm>
#include <numeric>


using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace pr2_arm_kinematics {

  static const std::string IK_SERVICE = "get_ik";
  static const std::string FK_SERVICE = "get_fk";
  static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
  static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

  PR2ArmKinematics::PR2ArmKinematics():  node_handle_("~"),dimension_(7)
  {
    urdf::Model robot_model;
    std::string tip_name, xml_string;

    while(!loadRobotModel(node_handle_,robot_model,root_name_,tip_name,xml_string) && node_handle_.ok())
    {
      ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
      ros::Duration(0.5).sleep();
    }

    ROS_INFO("Loading KDL Tree");
    if(!getKDLChain(xml_string,root_name_,tip_name,kdl_chain_))
    {
      active_ = false;
      ROS_ERROR("Could not load kdl tree");
    }
    ROS_INFO("Advertising services");
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    node_handle_.param<int>("free_angle",free_angle_,2);

    node_handle_.param<double>("search_discretization",search_discretization_,0.01);
    pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model,root_name_,tip_name, search_discretization_,free_angle_));
    if(!pr2_arm_ik_solver_->active_)
    {
      ROS_ERROR("Could not load ik");
      active_ = false;
    }
    else
    {

      pr2_arm_ik_solver_->getSolverInfo(ik_solver_info_);
      pr2_arm_kinematics::getKDLChainInfo(kdl_chain_,fk_solver_info_);
      fk_solver_info_.joint_names = ik_solver_info_.joint_names;

      for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
      {
        ROS_INFO("PR2Kinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
      }
      for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
      {
        ROS_INFO("PR2Kinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
      }
      for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
      {
        ROS_INFO("PR2Kinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
      }
      ROS_INFO("PR2Kinematics::active");
      active_ = true;
      fk_service_ = node_handle_.advertiseService(FK_SERVICE,&PR2ArmKinematics::getPositionFK,this);
      ik_service_ = node_handle_.advertiseService(IK_SERVICE,&PR2ArmKinematics::getPositionIK,this);

      ik_solver_info_service_ = node_handle_.advertiseService(IK_INFO_SERVICE,&PR2ArmKinematics::getIKSolverInfo,this);
      fk_solver_info_service_ = node_handle_.advertiseService(FK_INFO_SERVICE,&PR2ArmKinematics::getFKSolverInfo,this);

    }
  }

  bool PR2ArmKinematics::isActive()
  {
    if(active_)
      return true;
    return false;
  }

  bool PR2ArmKinematics::getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                                   kinematics_msgs::GetPositionIK::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("IK service not active");
      return true;
    }

    if(!checkIKService(request,response,ik_solver_info_))
      return true;

    geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    KDL::Frame pose_desired;
    if(!convertPoseToRootFrame(pose_msg_in,pose_desired,root_name_,tf_))
    {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      int tmp_index = getJointIndex(request.ik_request.ik_seed_state.joint_state.name[i],ik_solver_info_);
      if(tmp_index >=0)
      {
        jnt_pos_in(tmp_index) = request.ik_request.ik_seed_state.joint_state.position[i];
      }
      else
      {
        ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.ik_seed_state.joint_state.name[i].c_str());
      }
    }

    int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                       pose_desired,
                                                       jnt_pos_out,
                                                       request.timeout.toSec());
    if(ik_valid == pr2_arm_kinematics::TIMED_OUT)
       response.error_code.val = response.error_code.TIMED_OUT;
    if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
       response.error_code.val = response.error_code.NO_IK_SOLUTION;

    if(ik_valid >= 0)
    {
      response.solution.joint_state.name = ik_solver_info_.joint_names;
      response.solution.joint_state.position.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        response.solution.joint_state.position[i] = jnt_pos_out(i);
        ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
      }
      response.error_code.val = response.error_code.SUCCESS;
      return true;
    }
    else
    {
      ROS_DEBUG("An IK solution could not be found");   
      return true;
    }
  }

  bool PR2ArmKinematics::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("IK node not active");
      return true;
    }
    response.kinematic_solver_info = ik_solver_info_;
    return true;
  }

  bool PR2ArmKinematics::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("IK node not active");
      return true;
    }
    response.kinematic_solver_info = fk_solver_info_;
    return true;
  }

  bool PR2ArmKinematics::getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                                   kinematics_msgs::GetPositionFK::Response &response)
  {
    if(!active_)
    {
      ROS_ERROR("FK service not active");
      return true;
    }

    if(!checkFKService(request,response,fk_solver_info_))
      return true;

    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(dimension_);
    for(int i=0; i < (int) request.robot_state.joint_state.position.size(); i++) 
    {
      int tmp_index = getJointIndex(request.robot_state.joint_state.name[i],fk_solver_info_);
      if(tmp_index >=0)
        jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
    }

    response.pose_stamped.resize(request.fk_link_names.size());
    response.fk_link_names.resize(request.fk_link_names.size());

    bool valid = true;
    for(unsigned int i=0; i < request.fk_link_names.size(); i++)
    {
      ROS_DEBUG("End effector index: %d",pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,request.fk_link_names[i]));
      ROS_DEBUG("Chain indices: %d",kdl_chain_.getNrOfSegments());
      if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,request.fk_link_names[i])) >=0)
      {
        tf_pose.frame_id_ = root_name_;
        tf_pose.stamp_ = ros::Time();
        tf::PoseKDLToTF(p_out,tf_pose);
        try{
          tf_.transformPose(request.header.frame_id,tf_pose,tf_pose);
        }
        catch(...)
        {
          ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
          response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
          return false;
        }
        tf::poseStampedTFToMsg(tf_pose,pose);
        response.pose_stamped[i] = pose;
        response.fk_link_names[i] = request.fk_link_names[i];
        response.error_code.val = response.error_code.SUCCESS;
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s",request.fk_link_names[i].c_str());
        response.error_code.val = response.error_code.NO_FK_SOLUTION;
        valid = false;
      }
    }
    return true;
  }
} // namespace
