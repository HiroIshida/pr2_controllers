/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
/*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include "pr2_mechanism_controllers/unabstracted_base_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::UnabstractedBaseController, pr2_controller_interface::Controller)

namespace controller {

  const static double EPS = 1e-5;

UnabstractedBaseController::UnabstractedBaseController()
{
  //init variables
  new_cmd_available_ = false;
  pthread_mutex_init(&pr2_base_controller_lock_, NULL);
}

UnabstractedBaseController::~UnabstractedBaseController()
{
  direct_cmd_sub_.shutdown();
}

bool UnabstractedBaseController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  if(!base_kin_.init(robot,n))
    return false;
  node_ = n;

  //Get params from param server
  node_.param<double> ("max_translational_velocity", max_translational_velocity_,0.5);
  node_.param<double> ("max_rotational_velocity", max_rotational_velocity_, 10.0); //0.5
  node_.param<double> ("max_translational_acceleration/x", max_accel_.linear.x, .2);
  node_.param<double> ("max_translational_acceleration/y", max_accel_.linear.y, .2);
  node_.param<double> ("max_rotational_acceleration", max_accel_.angular.z, 10.0); //0.2
  node_.param<double> ("timeout", timeout_, 1.0);

  direct_cmd_sub_ = node_.subscribe<pr2_mechanism_controllers::BaseDirectCommand>("direct_command", 1, &UnabstractedBaseController::directCommandCallback, this);

  //casters
  caster_controller_.resize(base_kin_.num_casters_);
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    control_toolbox::Pid p_i_d;
    if(!p_i_d.init(ros::NodeHandle(node_, base_kin_.caster_[i].joint_name_+"/velocity_controller")))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kin_.caster_[i].joint_name_.c_str());
      return false;
    }

    caster_controller_[i].reset(new JointVelocityController());
    if(!caster_controller_[i]->init(base_kin_.robot_state_, base_kin_.caster_[i].joint_name_, p_i_d))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kin_.caster_[i].joint_name_.c_str());
      return false;
    }
    if (!caster_controller_[i]->joint_state_->calibrated_)
    {
      ROS_ERROR("Caster joint \"%s\" not calibrated (namespace: %s)",
                base_kin_.caster_[i].joint_name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
  }
  //wheels
  wheel_controller_.resize(base_kin_.num_wheels_);
  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    control_toolbox::Pid p_i_d;
    if(!p_i_d.init(ros::NodeHandle(node_,base_kin_.wheel_[j].joint_name_)))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kin_.wheel_[j].joint_name_.c_str());
      return false;
    }
    wheel_controller_[j].reset(new JointVelocityController());
   if(!wheel_controller_[j]->init(base_kin_.robot_state_, base_kin_.wheel_[j].joint_name_, p_i_d))
   {
      ROS_ERROR("Could not initialize joint velocity controller for %s",base_kin_.wheel_[j].joint_name_.c_str());
      return false;
   }
  }
  for(int i = 0; i < base_kin_.num_casters_; ++i)
  {
    if(!base_kin_.caster_[i].joint_->calibrated_)
    {
      ROS_ERROR("The Base controller could not start because the casters were not calibrated. Relaunch the base controller after you see the caster calibration finish.");
      return false; // Casters are not calibrated
    }
  }

  if (!((filters::MultiChannelFilterBase<double>&)caster_vel_filter_).configure(base_kin_.num_casters_, std::string("caster_velocity_filter"), node_)){
     ROS_ERROR("BaseController: could not configure velocity filters for casters");
     return false;
  }
  filtered_velocity_.resize(base_kin_.num_casters_);

  // In pr2_base_controller.cpp, this initialization process is called in the
  // constructor. But in this case, we need numbers of wheels and casters
  // thus the initialization must take place here.
  auto reset_cmd = [&, this](pr2_mechanism_controllers::BaseDirectCommand& cmd){
    cmd.wheel_vels = std::vector<double>(base_kin_.num_wheels_, 0.0);
    cmd.caster_vels = std::vector<double>(base_kin_.num_casters_, 0.0);
  };
  reset_cmd(direct_cmd_vel_);
  reset_cmd(direct_cmd_vel_t_);
  reset_cmd(desired_cmd_);
  return true;
}

// Set the base velocity command
void UnabstractedBaseController::setCommand(const pr2_mechanism_controllers::BaseDirectCommand &direct_cmd_vel)
{
  if(direct_cmd_vel.wheel_vels.size()!=base_kin_.num_wheels_){
     ROS_ERROR("BaseController: wheel's vel array requires %d elements, but obtained %lu.", 
         base_kin_.num_wheels_,
         direct_cmd_vel.wheel_vels.size());
     return;
  }
  if(direct_cmd_vel.caster_vels.size()!=base_kin_.num_casters_){
     ROS_ERROR("BaseController: caster's vel array requires %d elements, but obtained %lu.", 
         base_kin_.num_casters_,
         direct_cmd_vel.caster_vels.size());
     return;
  }

  for(int i = 0; i < base_kin_.num_casters_; i++)
  // TODO do some clamping procedure
  direct_cmd_vel_t_ = direct_cmd_vel;
  cmd_received_timestamp_ = base_kin_.robot_state_->getTime();
  new_cmd_available_ = true;
}

geometry_msgs::Twist UnabstractedBaseController::interpolateCommand(const geometry_msgs::Twist &start, const geometry_msgs::Twist &end, const geometry_msgs::Twist &max_rate, const double &dT)
{
  geometry_msgs::Twist result;
  geometry_msgs::Twist alpha;
  double delta(0), max_delta(0);

  delta = end.linear.x - start.linear.x;
  max_delta = max_rate.linear.x * dT;
  if(fabs(delta) <= max_delta || max_delta < EPS)
    alpha.linear.x = 1;
  else
    alpha.linear.x = max_delta / fabs(delta);

  delta = end.linear.y - start.linear.y;
  max_delta = max_rate.linear.y * dT;
  if(fabs(delta) <= max_delta || max_delta < EPS)
    alpha.linear.y = 1;
  else
    alpha.linear.y = max_delta / fabs(delta);

  delta = end.angular.z - start.angular.z;
  max_delta = max_rate.angular.z * dT;
  if(fabs(delta) <= max_delta || max_delta < EPS)
    alpha.angular.z = 1;
  else
    alpha.angular.z = max_delta / fabs(delta);

  double alpha_min = alpha.linear.x;
  if(alpha.linear.y < alpha_min)
    alpha_min = alpha.linear.y;
  if(alpha.angular.z < alpha_min)
    alpha_min = alpha.angular.z;

  result.linear.x = start.linear.x + alpha_min * (end.linear.x - start.linear.x);
  result.linear.y = start.linear.y + alpha_min * (end.linear.y - start.linear.y);
  result.angular.z = start.angular.z + alpha_min * (end.angular.z - start.angular.z);
  return result;
}

void UnabstractedBaseController::starting()
{
  last_time_ = base_kin_.robot_state_->getTime();
  cmd_received_timestamp_ = base_kin_.robot_state_->getTime();
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    caster_controller_[i]->starting();
  }
  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    wheel_controller_[j]->starting();
  }
}

void UnabstractedBaseController::update()
{
  ros::Time current_time = base_kin_.robot_state_->getTime();
  double dT = std::min<double>((current_time - last_time_).toSec(), base_kin_.MAX_DT_);

  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_base_controller_lock_) == 0)
    {
      desired_cmd_ = direct_cmd_vel_t_;
      new_cmd_available_ = false;
      pthread_mutex_unlock(&pr2_base_controller_lock_);
    }
  }

  if((current_time - cmd_received_timestamp_).toSec() > timeout_)
  {
    for(auto& vel : direct_cmd_vel_.caster_vels){
      vel = 0.0;
    }
    for(auto& vel : direct_cmd_vel_.wheel_vels){
      vel = 0.0;
    }
  }
  else{
    direct_cmd_vel_ = desired_cmd_; // TODO interpolate
    // cmd_vel_ = interpolateCommand(cmd_vel_, desired_vel_, max_accel_, dT);
  }

  computeJointCommands(dT);

  setJointCommands();

  updateJointControllers();

  last_time_ = current_time;

}

void UnabstractedBaseController::computeJointCommands(const double &dT)
{
  base_kin_.computeWheelPositions();

  computeDesiredCasterSteer(dT);

  computeDesiredWheelSpeeds();
}

void UnabstractedBaseController::setJointCommands()
{
  setDesiredCasterSteer();

  setDesiredWheelSpeeds();
}

void UnabstractedBaseController::computeDesiredCasterSteer(const double &dT)
{
  for(int i=0; i < base_kin_.num_casters_; i++){
    base_kin_.caster_[i].steer_velocity_desired_ = direct_cmd_vel_.caster_vels[i];
  }
}

void UnabstractedBaseController::setDesiredCasterSteer()
{
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    caster_controller_[i]->setCommand(base_kin_.caster_[i].steer_velocity_desired_);
  }
}

void UnabstractedBaseController::computeDesiredWheelSpeeds()
{
  for(int i = 0; i < (int) base_kin_.num_wheels_; i++)
  {
    base_kin_.wheel_[i].wheel_speed_cmd_ = direct_cmd_vel_.wheel_vels[i];
  }
}

void UnabstractedBaseController::setDesiredWheelSpeeds()
{
  for(int i = 0; i < (int) base_kin_.num_wheels_; i++)
  {
    wheel_controller_[i]->setCommand(base_kin_.wheel_[i].direction_multiplier_ * base_kin_.wheel_[i].wheel_speed_cmd_);
  }
}

void UnabstractedBaseController::updateJointControllers()
{
  for(int i = 0; i < base_kin_.num_wheels_; i++)
    wheel_controller_[i]->update();
  for(int i = 0; i < base_kin_.num_casters_; i++)
    caster_controller_[i]->update();
}

void UnabstractedBaseController::directCommandCallback(const pr2_mechanism_controllers::BaseDirectCommandConstPtr& msg)
{
  //currently do nothing
  pthread_mutex_lock(&pr2_base_controller_lock_);
  this->setCommand(*msg);
  pthread_mutex_unlock(&pr2_base_controller_lock_);
}
} // namespace
