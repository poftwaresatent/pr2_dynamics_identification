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
   \file di_analysis.cpp
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <dynamics_identification/Segment.h>
#include <dynamics_identification/Analysis.h>
#include <Eigen/LU>
#include <err.h>

using namespace dynamics_identification;
using namespace boost;
using namespace std;


struct analysis_s {
  size_t setup_id;
  size_t segment_id;
  
  // regression of x = A * exp(-t) + B * t + C
  // stored as phaseX_a[0] == A  phaseX_a[1] == B  phaseX_a[2] == C
  Eigen::Vector3d phase1_a;
  Eigen::Vector3d phase2_a;
  
  // B_minus is the B parameter of the phase where velocity < 0 and
  // B_plus where it is > 0
  double B_minus;
  double B_plus;
  
  // Estimated friction coefficients, assuming a simple static +
  // viscous model of the form f_actual = f_applied + f_static +
  // f_viscous where f_static = - sign(vel) * coeff_static and
  // f_viscous = - coeff_viscous * vel
  double friction_static;
  double friction_viscous;
  
  double estimated_inertia;
  
  // Estimated position, velocity, and acceleration, based on the
  // exponential curve fit, and estimated actual torque and resulting
  // inertia based on the friction model.
  vector<double> phase1_position;
  vector<double> phase1_velocity;
  vector<double> phase1_acceleration;
  vector<double> phase1_actual_torque;
  vector<double> phase2_position;
  vector<double> phase2_velocity;
  vector<double> phase2_acceleration;
  vector<double> phase2_actual_torque;
};


static ros::Subscriber segment_sub_;
static ros::Publisher analysis_pub_;


static void segment_cb(shared_ptr<Segment const> const & segment);


int main(int argc, char*argv[])
{
  ros::init(argc, argv, "di_analysis");
  ros::NodeHandle nn("~");
  
  segment_sub_ = nn.subscribe<Segment>("/di_segmentation/segment", 100, segment_cb);
  analysis_pub_ = nn.advertise<Analysis>("analysis", 100);
  
  ros::spin();
}


void segment_cb(shared_ptr<Segment const> const & segment)
{
  //analysis_pub_.publish(analysis_msg);
}
