/*
 * Dynamics Identification Toolbox
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file di_controller_plugin.cpp
   \author Roland Philippsen
*/

#include <pluginlib/class_list_macros.h>

using namespace std;


class DIController
  : public pr2_controller_interface::Controller
{
public:
  DIController();
  virtual ~DIController();
  
  virtual void update(void);
  virtual bool init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle &nn);
  
  uint64_t tick_;
  std::vector<pr2_mechanism_model::JointState *> controlled_joint_;

  //  ros::Publisher _pub_;
  //  dynamics_identification:: _msg_;
};


DIController::
DIController()
  : tick_(0)
{
}


DIController::
~DIController()
{
}


void DIController::
update(void)
{
  bool ok(true);
  size_t const ndof(controlled_joint_.size());
  double blah(0);
  
  for (size_t ii(0); ii < ndof; ++ii) {
    blah = controlled_joint_[ii]->position_;
    blah = controlled_joint_[ii]->velocity_;
    blah = controlled_joint_[ii]->measured_effort_;
  }
  
  double toto(0);
  if (ok) {
    for (size_t ii(0); ii < ndof; ++ii) {
      controlled_joint_[ii]->commanded_effort_ = toto;
    }
  }
  else {
    for (size_t ii(0); ii < ndof; ++ii) {
      controlled_joint_[ii]->commanded_effort_ = 0;
    }
  }
  
  ++tick_;
  
  // _msg_. = ;
  // _pub_.publish(_msg_);
  
}


bool DIController::
init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle & nn)
{
  try {
    
    string joints_param_name;
    if ( ! nn.getParam("joints_param_name", joints_param_name)) {
      joints_param_name = "joints";
    }
    
    ROS_INFO ("reading joints from parameter `%s'", joints_param_name.c_str());
    XmlRpc::XmlRpcValue joints_value;
    if ( ! nn.getParam(joints_param_name, joints_value)) {
      throw runtime_error("invalid joints_param_name `" + joints_param_name + "'");
    }
    if (0 == joints_value.size()) {
      throw runtime_error("empty joint list");
    }
    
    controlled_joint_.clear();	// paranoid
    for (int ii(0); ii < joints_value.size(); ++ii) {
      string const & name(static_cast<std::string const &>(joints_value[ii]));
      pr2_mechanism_model::JointState * joint(robot->getJointState(name));
      if ( ! joint) {
	throw runtime_error("no joint called `" + name + "'");
      }
      ROS_INFO ("adding joint `%s'", name.c_str());
      controlled_joint_.push_back(joint);
    }
    
  }
  
  catch (std::exception const & ee) {
    ROS_ERROR ("DIController::init(): EXCEPTION: %s", ee.what());
    // cleanup?
    return false;
  }

  catch (XmlRpc::XmlRpcException const & ee) {
    ROS_ERROR ("DIController::init(): XmlRpcException: %s", ee.getMessage().c_str());
    // cleanup?
    return false;
  }
  
  // _pub_ = nn.advertise<dynamics_identification::>("", 100);
  
  ROS_INFO ("ready to rock");
  return true;
}


PLUGINLIB_REGISTER_CLASS (DIController, DIController, pr2_controller_interface::Controller)
