/*********************************************************************
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
*********************************************************************/

/** \author E. Gil Jones */

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <planning_environment/models/collision_models.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <std_srvs/Empty.h>
#include <planning_environment/models/model_utils.h>

static const std::string ARM_FK_NAME = "/pr2_right_arm_kinematics/get_fk";
static const std::string ARM_IK_NAME = "/pr2_right_arm_kinematics/get_ik";
static const std::string ARM_COLLISION_IK_NAME = "/pr2_right_arm_kinematics/get_constraint_aware_ik";
static const std::string ARM_QUERY_NAME = "/pr2_right_arm_kinematics/get_ik_solver_info";
static const std::string SET_PLANNING_SCENE_DIFF_SERVICE="/environment_server/set_planning_scene_diff";

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

std::vector<double> generateRandomValues(std::vector<double> min_values,
                                         std::vector<double> max_values) {
  std::vector<double> ret_vec;
  for(unsigned int i = 0; i < min_values.size(); i++) {
    ret_vec.push_back(gen_rand(min_values[i], max_values[i]));
  }
  return ret_vec;
}

class TestConstraintAwareKinematics : public testing::Test
{
protected:

  virtual void SetUp() {
    collision_models_ = new planning_environment::CollisionModels("robot_description");

    ready_ = false;
    done_ = false;

    srand(time(NULL));

    ros::service::waitForService(ARM_QUERY_NAME);
    ros::service::waitForService(ARM_FK_NAME);
    ros::service::waitForService(ARM_COLLISION_IK_NAME);
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SERVICE);
    
    query_client_ = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(ARM_QUERY_NAME);
    fk_client_ = nh_.serviceClient<kinematics_msgs::GetPositionFK>(ARM_FK_NAME);
    ik_with_collision_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(ARM_COLLISION_IK_NAME);
    ik_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ARM_IK_NAME);
    set_planning_scene_diff_client_ = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_SERVICE);

    //first getting limits
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;
    
    bool service_call = query_client_.call(request, response);
    ASSERT_TRUE(service_call);
    ASSERT_TRUE(response.kinematic_solver_info.joint_names.size() == response.kinematic_solver_info.limits.size());
    ASSERT_TRUE(!response.kinematic_solver_info.link_names.empty());
    
    std::vector<double> min_limits, max_limits;
    joint_names_ = response.kinematic_solver_info.joint_names;
    for(unsigned int i=0; i< joint_names_.size(); i++) {
      min_limits_.push_back(response.kinematic_solver_info.limits[i].min_position);
      max_limits_.push_back(response.kinematic_solver_info.limits[i].max_position);
    }
    ik_link_name_ = response.kinematic_solver_info.link_names[0];

    arm_navigation_msgs::SetPlanningSceneDiff::Request set_req;

    //now dealing with planning scene
    arm_navigation_msgs::CollisionObject table;
    table.header.stamp = ros::Time::now();
    table.header.frame_id = "odom_combined";
    table.id = "table";
    table.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    table.shapes.resize(1);
    table.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    table.shapes[0].dimensions.resize(3);
    table.shapes[0].dimensions[0] = 1.0;
    table.shapes[0].dimensions[1] = 1.0;
    table.shapes[0].dimensions[1] = .05;
    table.poses.resize(1);
    table.poses[0].position.x = 1.0;
    table.poses[0].position.y = 0;
    table.poses[0].position.z = .5;
    table.poses[0].orientation.w = 1.0;

    set_req.planning_scene_diff.collision_objects.push_back(table);

    arm_navigation_msgs::AttachedCollisionObject att_box;

    att_box.object.header.stamp = ros::Time::now();
    att_box.object.header.frame_id = "r_gripper_palm_link";
    att_box.link_name = "r_gripper_palm_link";
    att_box.object.id = "att_box";
    att_box.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    att_box.object.shapes.resize(1);
    att_box.object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    att_box.object.shapes[0].dimensions.resize(3);
    att_box.object.shapes[0].dimensions[0] = .04;
    att_box.object.shapes[0].dimensions[1] = .04;
    att_box.object.shapes[0].dimensions[2] = .2;
    att_box.object.poses.resize(1);
    att_box.object.poses[0].position.x = .12;
    att_box.object.poses[0].position.y = 0;
    att_box.object.poses[0].position.z = 0;
    att_box.object.poses[0].orientation.x = 0;
    att_box.object.poses[0].orientation.y = 0;
    att_box.object.poses[0].orientation.z = 0;
    att_box.object.poses[0].orientation.w = 1;

    std::vector<std::string> touch_links;
    
    touch_links.push_back("r_gripper_palm_link");
    touch_links.push_back("r_gripper_l_finger_link");
    touch_links.push_back("r_gripper_r_finger_link");
    touch_links.push_back("r_gripper_l_finger_link");
    touch_links.push_back("r_gripper_l_finger_tip_link");
    att_box.touch_links = touch_links;

    set_req.planning_scene_diff.attached_collision_objects.push_back(att_box);

    arm_navigation_msgs::SetPlanningSceneDiff::Response set_res;
    ASSERT_TRUE(set_planning_scene_diff_client_.call(set_req, set_res));

    planning_scene_ = set_res.planning_scene;
  }

  virtual void TearDown()
  {
    delete collision_models_;
  }

protected:

  ros::NodeHandle nh_;

  ros::ServiceClient query_client_;
  ros::ServiceClient fk_client_;
  ros::ServiceClient ik_with_collision_client_;
  ros::ServiceClient ik_client_;
  ros::ServiceClient set_planning_scene_diff_client_;

  arm_navigation_msgs::PlanningScene planning_scene_;

  std::vector<std::string> joint_names_;
  std::vector<double> min_limits_, max_limits_;
  std::string ik_link_name_;

  planning_environment::CollisionModels* collision_models_;

  bool ready_;
  bool done_;
};

TEST_F(TestConstraintAwareKinematics, TestCollisions)
{
  kinematics_msgs::GetConstraintAwarePositionIK::Request ik_request;
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_response;
  
  ik_request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  ik_request.ik_request.ik_seed_state.joint_state.position.resize(joint_names_.size());
  ik_request.ik_request.ik_link_name = ik_link_name_;
  ik_request.timeout = ros::Duration(2.0);

  ik_request.ik_request.ik_link_name = "r_wrist_roll_link";
  ik_request.ik_request.pose_stamped.header.frame_id = "base_link";
  ik_request.ik_request.pose_stamped.pose.position.x = 0.52;
  ik_request.ik_request.pose_stamped.pose.position.y = -.2;
  ik_request.ik_request.pose_stamped.pose.position.z = .8;

  ik_request.ik_request.pose_stamped.pose.orientation.x = 0.0;
  ik_request.ik_request.pose_stamped.pose.orientation.y = 0.7071;
  ik_request.ik_request.pose_stamped.pose.orientation.z = 0.0;
  ik_request.ik_request.pose_stamped.pose.orientation.w = 0.7071;

  planning_models::KinematicState* kin_state = collision_models_->setPlanningScene(planning_scene_);

  for(unsigned int i = 0; i < 25; i++) {
    
    std::map<std::string, double> joint_value_map;
    
    //first finding a valid starting robot state
    while(true) {
      std::vector<double> vals = generateRandomValues(min_limits_, max_limits_);
      for(unsigned int i = 0; i < vals.size(); i++) {
        joint_value_map[joint_names_[i]] = vals[i];
      }
      kin_state->setKinematicState(joint_value_map);
      if(!collision_models_->isKinematicStateInCollision(*kin_state)) {
        break;
      }
    }
    
    planning_environment::convertKinematicStateToRobotState(*kin_state, ros::Time::now(), collision_models_->getWorldFrameId(),
                                                            planning_scene_.robot_state);

    ik_request.ik_request.ik_seed_state.joint_state.position = generateRandomValues(min_limits_, max_limits_);
    ASSERT_TRUE(ik_with_collision_client_.call(ik_request, ik_response));
    
    EXPECT_EQ(ik_response.error_code.val, ik_response.error_code.SUCCESS);
  }
  collision_models_->revertPlanningScene(kin_state);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_constraint_aware_pr2_kinematics");
    
  return RUN_ALL_TESTS();
}
