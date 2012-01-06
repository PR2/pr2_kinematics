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

#include <pr2_arm_kinematics_constraint_aware/pr2_arm_kinematics_constraint_aware.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include "ros/ros.h"
#include <algorithm>
#include <numeric>

#include <sensor_msgs/JointState.h>
#include <kinematics_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <planning_environment/models/model_utils.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace pr2_arm_kinematics {

static const std::string IK_WITH_COLLISION_SERVICE = "get_constraint_aware_ik";
static const double IK_DEFAULT_TIMEOUT = 10.0;

PR2ArmIKConstraintAware::PR2ArmIKConstraintAware(): PR2ArmKinematics(false)
{
  node_handle_.param<bool>("visualize_solution",visualize_solution_,true);
  node_handle_.param<std::string>("group", group_, std::string());
  ROS_DEBUG("Advertising services");

  if(!isActive())
  {
    ROS_ERROR("Could not load pr2_arm_kinematics_constraint_aware server");
  }
  else
  {
    ROS_INFO("Loading pr2_arm_kinematics_constraint_aware server");
  }
  collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");
  if(group_.empty()) {
    ROS_WARN("Must specify planning group in configuration");
  }
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(group_);
  if(joint_model_group == NULL) {
    ROS_WARN_STREAM("No joint group " << group_);
  }
  arm_links_ = joint_model_group->getGroupLinkNames();

  const planning_models::KinematicModel::LinkModel* end_effector_link = collision_models_interface_->getKinematicModel()->getLinkModel(ik_solver_info_.link_names.back());
  end_effector_collision_links_ = collision_models_interface_->getKinematicModel()->getChildLinkModelNames(end_effector_link);

  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("kinematics_collisions", 128);
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("kinematics_collisions_array", 128);

  advertiseIK();
}

void PR2ArmIKConstraintAware::advertiseIK()
{
  ik_collision_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_SERVICE,&PR2ArmIKConstraintAware::getConstraintAwarePositionIK,this);
  display_trajectory_publisher_ = root_handle_.advertise<arm_navigation_msgs::DisplayTrajectory>("ik_solution_display", 1);
}

bool PR2ArmIKConstraintAware::isReady(arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  if(!active_)
  {
    ROS_ERROR("IK service is not ready");
    return false;
  }
  if(!collision_models_interface_->isPlanningSceneSet()) {
    ROS_INFO("Planning scene not set");
    error_code.val = error_code.COLLISION_CHECKING_UNAVAILABLE;
    return false;
  }    
  error_code.val = error_code.SUCCESS;
  return true;
}
  
bool PR2ArmIKConstraintAware::transformPose(const std::string& des_frame,
					    const geometry_msgs::PoseStamped& pose_in,
					    geometry_msgs::PoseStamped& pose_out)
{
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
								  des_frame,
								  pose_in.header,
								  pose_in.pose,
								  pose_out)) {
    ROS_WARN_STREAM("Problem transforming pose");
    return false;
  }
  return true;
}


int PR2ArmIKConstraintAware::CartToJntSearch(const KDL::JntArray& q_in, 
                                             const KDL::Frame& p_in, 
                                             KDL::JntArray &q_out, 
                                             const double &timeout, 
                                             arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
{
  if(!isReady(error_code))
    return -1;
  bool ik_valid = (pr2_arm_ik_solver_->CartToJntSearch(q_in,
                                                       p_in,
                                                       q_out, 
                                                       timeout,
                                                       error_code,
                                                       boost::bind(&PR2ArmIKConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                       boost::bind(&PR2ArmIKConstraintAware::collisionCheck, this, _1, _2, _3)) >= 0);

  return ik_valid;
}

bool PR2ArmIKConstraintAware::getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                                            kinematics_msgs::GetPositionIK::Response &response)
{
  if(!isReady(response.error_code))
    return true;
  
  if(!checkIKService(request,response,ik_solver_info_))
    return true;

  collision_models_interface_->resetToStartState(*collision_models_interface_->getPlanningSceneState());

  geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;

  planning_environment::setRobotStateAndComputeTransforms(request.ik_request.robot_state, *collision_models_interface_->getPlanningSceneState());
  
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  root_name_,
                                                                  pose_msg_in.header,
                                                                  pose_msg_in.pose,
                                                                  request.ik_request.pose_stamped)) {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }
  return getPositionIKHelper(request, response);
}

bool PR2ArmIKConstraintAware::getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request_in,
                                                           kinematics_msgs::GetConstraintAwarePositionIK::Response &response)
{
  if(!isReady(response.error_code))
    return true;

  if(!checkConstraintAwareIKService(request_in,response,ik_solver_info_))
  {
    ROS_ERROR("IK service request is malformed");
    return true;
  }

  collision_models_interface_->resetToStartState(*collision_models_interface_->getPlanningSceneState());

  collision_models_interface_->disableCollisionsForNonUpdatedLinks(group_);
 
  ros::Time start_time = ros::Time::now();
  ROS_DEBUG("Received IK request is in the frame: %s",request_in.ik_request.pose_stamped.header.frame_id.c_str());

  ik_request_ = request_in.ik_request;
  constraints_ = request_in.constraints;

  geometry_msgs::PoseStamped pose_msg_in = ik_request_.pose_stamped;
  geometry_msgs::PoseStamped pose_msg_out;

  ROS_DEBUG_STREAM("Ik request in frame " << pose_msg_in.header << " is " 
                   << pose_msg_in.pose.position.x << " " 
                   << pose_msg_in.pose.position.y << " " 
                   << pose_msg_in.pose.position.z << " " 
                   << pose_msg_in.pose.orientation.x << " " 
                   << pose_msg_in.pose.orientation.y << " " 
                   << pose_msg_in.pose.orientation.z << " " 
                   << pose_msg_in.pose.orientation.w);

  planning_environment::setRobotStateAndComputeTransforms(request_in.ik_request.robot_state, *collision_models_interface_->getPlanningSceneState());
  
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  root_name_,
                                                                  pose_msg_in.header,
                                                                  pose_msg_in.pose,
                                                                  pose_msg_out)) {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }
  ik_request_.pose_stamped = pose_msg_out;

  ROS_DEBUG_STREAM("Ik request in frame " << root_name_ << " is " 
                   << pose_msg_out.pose.position.x << " " 
                   << pose_msg_out.pose.position.y << " " 
                   << pose_msg_out.pose.position.z << " " 
                   << pose_msg_out.pose.orientation.x << " " 
                   << pose_msg_out.pose.orientation.y << " " 
                   << pose_msg_out.pose.orientation.z << " " 
                   << pose_msg_out.pose.orientation.w);

  KDL::JntArray jnt_pos_out;
  KDL::JntArray jnt_pos_in;
  KDL::Frame p_in;
  tf::PoseMsgToKDL(pose_msg_out.pose,p_in);
  jnt_pos_in.resize(dimension_);
  jnt_pos_out.resize(dimension_);
  for(unsigned int i=0; i < request_in.ik_request.ik_seed_state.joint_state.name.size(); i++)
  {
    int tmp_index = pr2_arm_kinematics::getJointIndex(request_in.ik_request.ik_seed_state.joint_state.name[i],ik_solver_info_);
    if(tmp_index != -1) {
      ROS_DEBUG_STREAM("Setting name " << request_in.ik_request.ik_seed_state.joint_state.name[i]
                       << " index " << tmp_index 
                       << " to " << request_in.ik_request.ik_seed_state.joint_state.position[i]);
      jnt_pos_in(tmp_index) = request_in.ik_request.ik_seed_state.joint_state.position[i];
    }
  }

  ros::Time ik_solver_time = ros::Time::now();
  bool ik_valid = CartToJntSearch(jnt_pos_in,
                                  p_in,
                                  jnt_pos_out, 
                                  request_in.timeout.toSec(),
                                  response.error_code);

  if(ik_valid)
  {
    response.solution.joint_state.name = ik_solver_info_.joint_names;
    response.solution.joint_state.position.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      response.solution.joint_state.position[i] = jnt_pos_out(i);
      ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
    }
    /*
      if(visualize_solution_)
      {
      arm_navigation_msgs::DisplayTrajectory display_trajectory;
      display_trajectory.trajectory.joint_trajectory.points.resize(1);
      display_trajectory.trajectory.joint_trajectory.points[0].positions = response.solution.joint_state.position;
      display_trajectory.trajectory.joint_trajectory.joint_names = response.solution.joint_state.name;
      planning_monitor_->convertKinematicStateToRobotState(*kinematic_state_,display_trajectory.robot_state);
      display_trajectory_publisher_.publish(display_trajectory);
      }
    */
    ROS_DEBUG("IK service time: %f",(ros::Time::now()-start_time).toSec());
    response.error_code.val = response.error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG_STREAM("An IK solution could not be found " << response.error_code.val);
    // if(ros::Time::now()-last_planning_scene_drop_ > ros::Duration(5.0)) {
    //   last_planning_scene_drop_ = ros::Time::now();
    //   std::string filename = "bad_ik_";
    //   std::string str = boost::lexical_cast<std::string>(ros::Time::now().toSec());
    //   filename += str;
    //   collision_models_interface_->writePlanningSceneBag(filename,
    //                                                      collision_models_interface_->getLastPlanningScene());

    // }
    if(response.error_code.val != response.error_code.IK_LINK_IN_COLLISION) {
      sendEndEffectorPose(collision_models_interface_->getPlanningSceneState(),true);
      std_msgs::ColorRGBA col;
      col.a = .9;
      col.r = 1.0;
      col.b = 1.0;
      col.g = 0.0;
      visualization_msgs::MarkerArray arr;
      collision_models_interface_->getRobotMarkersGivenState(*collision_models_interface_->getPlanningSceneState(),
                                                             arr,
                                                             col,
                                                             "start_pose",
                                                             ros::Duration(0.0));
      vis_marker_array_publisher_.publish(arr);
    }
    return true;
  }
}

void PR2ArmIKConstraintAware::collisionCheck(const KDL::JntArray &jnt_array, 
                                             const KDL::Frame &p_in,
                                             arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  ros::Time n1 = ros::Time::now();

  std::map<std::string, double> joint_values;
  for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
  {
    joint_values[ik_solver_info_.joint_names[i]] = jnt_array(i);
  }
  collision_models_interface_->getPlanningSceneState()->setKinematicState(joint_values);
  if(collision_models_interface_->getPlanningSceneState() == NULL) {
    ROS_INFO_STREAM("Messed up");
  }
  if(collision_models_interface_->isKinematicStateInCollision(*(collision_models_interface_->getPlanningSceneState()))) {
    visualization_msgs::MarkerArray arr;
    std_msgs::ColorRGBA col;
    col.a = .9;
    col.r = 1.0;
    col.b = 0.0;
    col.g = 0.0;
    collision_models_interface_->getAllCollisionPointMarkers(*collision_models_interface_->getPlanningSceneState(),
                                                             arr,
                                                             col,
                                                             ros::Duration(0.0));
    vis_marker_array_publisher_.publish(arr);
    error_code.val = error_code.KINEMATICS_STATE_IN_COLLISION;
  } else {
    error_code.val = error_code.SUCCESS;
  }

  if(!planning_environment::doesKinematicStateObeyConstraints(*(collision_models_interface_->getPlanningSceneState()), 
                                                              constraints_)) {
    error_code.val = error_code.INVALID_GOAL_POSITION_CONSTRAINTS;
  }
  ROS_DEBUG_STREAM("Collision check took " << (ros::Time::now()-n1).toSec());
}

void PR2ArmIKConstraintAware::initialPoseCheck(const KDL::JntArray &jnt_array, 
                                               const KDL::Frame &p_in,
                                               arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  std::string kinematic_frame_id = pr2_arm_ik_solver_->getFrameId();
  std::string planning_frame_id = collision_models_interface_->getWorldFrameId();
  geometry_msgs::PoseStamped pose_stamped;
  tf::PoseKDLToMsg(p_in,pose_stamped.pose);
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = kinematic_frame_id;
   if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  planning_frame_id,
                                                                  pose_stamped.header,
                                                                  pose_stamped.pose,
                                                                  pose_stamped)) {
    ROS_ERROR_STREAM("Cannot transform from " << pose_stamped.header.frame_id << " to " << planning_frame_id);
  }
  //disabling all collision for arm links
  collision_space::EnvironmentModel::AllowedCollisionMatrix save_acm = collision_models_interface_->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = save_acm;
  for(unsigned int i = 0; i < arm_links_.size(); i++) {
    acm.changeEntry(arm_links_[i], true);
  }

  collision_models_interface_->setAlteredAllowedCollisionMatrix(acm);

  tf::Transform transform;
  tf::poseMsgToTF(pose_stamped.pose,transform);
  if(!collision_models_interface_->getPlanningSceneState()->hasLinkState(ik_request_.ik_link_name)) {
    ROS_ERROR("Could not find end effector root_link %s", ik_request_.ik_link_name.c_str());
    error_code.val = error_code.INVALID_LINK_NAME;
    return;
  }
  collision_models_interface_->getPlanningSceneState()->updateKinematicStateWithLinkAt(ik_request_.ik_link_name, transform);
  if(collision_models_interface_->isKinematicStateInCollision(*(collision_models_interface_->getPlanningSceneState()))) {
    visualization_msgs::MarkerArray arr;
    std_msgs::ColorRGBA col;
    col.a = .9;
    col.r = 1.0;
    col.b = 0.0;
    col.g = 0.0;
    collision_models_interface_->getAllCollisionPointMarkers(*collision_models_interface_->getPlanningSceneState(),
                                                             arr,
                                                             col,
                                                             ros::Duration(0.0));
    vis_marker_array_publisher_.publish(arr);
    error_code.val = error_code.IK_LINK_IN_COLLISION;
    ROS_DEBUG_STREAM("Initial pose check failing");
    sendEndEffectorPose(collision_models_interface_->getPlanningSceneState(), false);
  }
  else
    error_code.val = error_code.SUCCESS;
    
  collision_models_interface_->setAlteredAllowedCollisionMatrix(save_acm);
}

void PR2ArmIKConstraintAware::sendEndEffectorPose(const planning_models::KinematicState* state, bool valid) {
  visualization_msgs::MarkerArray hand_array;
  std_msgs::ColorRGBA col;
  col.a = .8;
  col.b = 0.0;
  
  if(valid) {
    col.g = 1.0;
  } else {
    col.r = 1.0;
  }
  
  collision_models_interface_->getRobotMarkersGivenState(*state,
                                                         hand_array,
                                                         col,
                                                         "end_effector",
                                                         ros::Duration(0.0),
                                                         &end_effector_collision_links_);
  vis_marker_array_publisher_.publish(hand_array);
}

void PR2ArmIKConstraintAware::printStringVec(const std::string &prefix, const std::vector<std::string> &string_vector)
{
  ROS_DEBUG("%s",prefix.c_str());
  for(unsigned int i=0; i < string_vector.size(); i++)
  {
    ROS_DEBUG("%s",string_vector[i].c_str());
  }
}
} // namespace

