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

#include <pr2_arm_kinematics/pr2_arm_kinematics_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include "ros/ros.h"
#include <algorithm>
#include <numeric>

#include <pluginlib/class_list_macros.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

//register PR2ArmKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(pr2_arm_kinematics,PR2ArmKinematicsPlugin, pr2_arm_kinematics::PR2ArmKinematicsPlugin, kinematics::KinematicsBase)

namespace pr2_arm_kinematics {

PR2ArmKinematicsPlugin::PR2ArmKinematicsPlugin():active_(false){}

bool PR2ArmKinematicsPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

bool PR2ArmKinematicsPlugin::initialize(const std::string& group_name,
                                        const std::string& base_name,
                                        const std::string& tip_name,
                                        const double& search_discretization)
{
  setValues(group_name, base_name, tip_name,search_discretization);
  urdf::Model robot_model;
  std::string xml_string;
  ros::NodeHandle private_handle("~/"+group_name);
  dimension_ = 7;
  while(!loadRobotModel(private_handle,robot_model,xml_string) && private_handle.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
  }

  ROS_DEBUG("Loading KDL Tree");
  if(!getKDLChain(xml_string,base_name_,tip_name_,kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }
  ROS_DEBUG("Advertising services");
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  private_handle.param<int>("free_angle",free_angle_,2);

  pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model, base_name_,tip_name_, search_discretization_,free_angle_));
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
      ROS_DEBUG("PR2Kinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
    }
    for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_DEBUG("PR2KinematicsPlugin::active for %s",group_name.c_str());
    active_ = true;
  }    
  return active_;
}

bool PR2ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           int &error_code)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }
    
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJnt(jnt_pos_in,
                                               pose_desired,
                                               jnt_pos_out);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              std::vector<double> &solution,
                                              int &error_code)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE; 
    return false;
  }
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);
  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION) {
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              const unsigned int& redundancy,
                                              const double &consistency_limit,
                                              std::vector<double> &solution,
                                              int &error_code)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE; 
    return false;
  }
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  unsigned int old_free_angle = pr2_arm_ik_solver_->getFreeAngle();
  pr2_arm_ik_solver_->setFreeAngle(redundancy);
  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     consistency_limit,
                                                     jnt_pos_out,
                                                     timeout);
  pr2_arm_ik_solver_->setFreeAngle(old_free_angle);

  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code = kinematics::NO_IK_SOLUTION; 
    return false;
  }
}


void PR2ArmKinematicsPlugin::desiredPoseCallback(const KDL::JntArray& jnt_array, 
                                                 const KDL::Frame& ik_pose,
                                                 arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  int int_error_code;
  for(int i=0; i < dimension_; i++)
    ik_seed_state[i] = jnt_array(i);

  geometry_msgs::Pose ik_pose_msg;
  tf::poseKDLToMsg(ik_pose,ik_pose_msg);

  desiredPoseCallback_(ik_pose_msg,ik_seed_state,int_error_code);
  if(int_error_code)
    error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
  else
    error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;     
}


void PR2ArmKinematicsPlugin::jointSolutionCallback(const KDL::JntArray& jnt_array, 
                                                   const KDL::Frame& ik_pose,
                                                   arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  int int_error_code;
  for(int i=0; i < dimension_; i++)
    ik_seed_state[i] = jnt_array(i);

  geometry_msgs::Pose ik_pose_msg;
  tf::poseKDLToMsg(ik_pose,ik_pose_msg);

  solutionCallback_(ik_pose_msg,ik_seed_state,int_error_code);
  if(int_error_code > 0)
    error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
  else
    error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;     
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              std::vector<double> &solution,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                              int &error_code_int)  
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code_int = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  desiredPoseCallback_ = desired_pose_callback;
  solutionCallback_    = solution_callback;

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout,
                                                     error_code,
                                                     boost::bind(&PR2ArmKinematicsPlugin::desiredPoseCallback, this, _1, _2, _3),
                                                     boost::bind(&PR2ArmKinematicsPlugin::jointSolutionCallback, this, _1, _2, _3));
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
    return false;

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code_int = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code_int = error_code.val;
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              const unsigned int& redundancy,
                                              const double &consistency_limit,
                                              std::vector<double> &solution,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                              int &error_code_int)  
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code_int = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  desiredPoseCallback_ = desired_pose_callback;
  solutionCallback_    = solution_callback;

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  unsigned int old_free_angle = pr2_arm_ik_solver_->getFreeAngle();
  pr2_arm_ik_solver_->setFreeAngle(redundancy);
  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     consistency_limit,
                                                     timeout,
                                                     error_code,
                                                     boost::bind(&PR2ArmKinematicsPlugin::desiredPoseCallback, this, _1, _2, _3),
                                                     boost::bind(&PR2ArmKinematicsPlugin::jointSolutionCallback, this, _1, _2, _3));
  pr2_arm_ik_solver_->setFreeAngle(old_free_angle);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
    return false;

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code_int = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code_int = error_code.val;
    return false;
  }
}

bool PR2ArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  poses.resize(link_names.size());

  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    ROS_DEBUG("End effector index: %d",pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
    if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i])) >=0)
    {
      tf::poseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

} // namespace
