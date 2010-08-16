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
  //////////////////////////////////////////////////
  // sanity checks
  
  size_t const ndata(segment->measured_milliseconds.size());
  if ((ndata != segment->measured_position.size())
      || (ndata != segment->seconds.size())
      || (segment->phase_begin.size() != 2)
      || (segment->phase_end.size() != 2)
      || (segment->phase_end[1] > ndata)
      || (segment->phase_end[1] <= segment->phase_begin[1])
      || (segment->phase_end[0] <= segment->phase_begin[0])
      || (fabs(segment->average_applied_torque) < 1e-6)
      || segment->smoothed_velocity.empty()) {
    ROS_ERROR ("segment_cb(): inconsistent segment data");
    return;
  }
  
  //////////////////////////////////////////////////
  // fit exponential curve onto each phase:
  // regression of x = A * exp(-t) + B * t + C
  // stored as aa[0] == A  aa[1] == B  aa[2] == C
  
  Eigen::VectorXd aa[2];
  for (size_t i_phase(0); i_phase < 2; ++i_phase) {
    size_t const npoints(segment->phase_end[i_phase] - segment->phase_begin[i_phase]);
    Eigen::VectorXd yy(npoints);
    Eigen::MatrixXd XX(npoints, 3);
    double const t0(segment->seconds[segment->phase_begin[i_phase]]);
    size_t i_regr(0);
    size_t i_in(segment->phase_begin[i_phase]);
    for (/**/; i_regr < npoints; ++i_regr, ++i_in) {
      yy[i_regr] = segment->measured_position[i_in];
      double const ts(segment->seconds[i_in] - t0);
      XX.coeffRef(i_regr, 0) = exp(-ts);
      XX.coeffRef(i_regr, 1) = ts;
      XX.coeffRef(i_regr, 2) = 1;
    }
    Eigen::MatrixXd toto(XX.transpose() * XX);
    aa[i_phase] = toto.inverse() * XX.transpose() * yy;
  }
  
  //////////////////////////////////////////////////
  // estimate static and viscous friction:
  // B_minus is the B parameter of the phase where velocity < 0
  // and B_plus where it is > 0
  
  double B_minus;
  double B_plus;
  if (segment->smoothed_velocity[0] > 0) {
    B_minus = aa[1].coeff(1);
    B_plus = aa[0].coeff(1);
  }
  else {
    B_minus = aa[0].coeff(1);
    B_plus = aa[1].coeff(1);
  }
  double const
    static_friction(segment->average_applied_torque * (B_minus - B_plus) / (B_minus + B_plus));
  if (static_friction < 0) {
    ROS_ERROR ("segment_cb(): unexpected static_friction = %g", static_friction);
    return;
  }
  double const B_bar((B_minus + B_plus) / 2);
  if (fabs(B_bar) < 1e-6) {
    ROS_ERROR ("segment_cb(): unexpected B_bar = %g", B_bar);
    return;
  }
  double const viscous_friction(segment->average_applied_torque / B_bar);
  if (viscous_friction < 0) {
    ROS_ERROR ("segment_cb(): unexpected viscous_friction = %g", viscous_friction);
    return;
  }
  
  //later// //////////////////////////////////////////////////
  //later// // estimated inertia
  
  //later// double const inertia(...);
  
  //////////////////////////////////////////////////
  // let's see what the fit looks like
  
  Analysis analysis_msg;
  analysis_msg.setup_id = segment->setup_id;
  analysis_msg.segment_id = segment->segment_id;
  analysis_msg.measured_position = segment->measured_position;
  analysis_msg.average_applied_torque = segment->average_applied_torque;
  analysis_msg.seconds = segment->seconds;
  analysis_msg.estimated_position.resize(ndata);
  analysis_msg.estimated_velocity.resize(ndata);
  analysis_msg.estimated_acceleration.resize(ndata);
  analysis_msg.estimated_actual_torque.resize(ndata);
  
  for (size_t i_phase(0); i_phase < 2; ++i_phase) {
    double const t0(segment->seconds[segment->phase_begin[i_phase]]);
    double const AA(aa[i_phase].coeff(0));
    double const BB(aa[i_phase].coeff(1));
    double const CC(aa[i_phase].coeff(2));
    double f_static;
    if (segment->smoothed_velocity[segment->phase_begin[i_phase]] > 0) {
      f_static = - static_friction;
    }
    else {
      f_static =   static_friction;
    }
    for (size_t i_point(segment->phase_begin[i_phase]);
	 i_point < segment->phase_end[i_phase]; ++i_point) {
      double const ts(segment->seconds[i_point] - t0);
      double const ae(AA * exp(-ts));
      analysis_msg.estimated_position[i_point] =       ae + BB * ts + CC;
      analysis_msg.estimated_velocity[i_point] =      -ae + BB;
      analysis_msg.estimated_acceleration[i_point] =   ae;
      analysis_msg.estimated_actual_torque[i_point] =
	analysis_msg.average_applied_torque
	+ f_static
	- analysis_msg.estimated_velocity[i_point] * viscous_friction;
    }
  }
  
  // fill in zeros in case there are gaps before/between/after the phases...
  for (size_t ii(0); ii < segment->phase_begin[0]; ++ii) {
    analysis_msg.estimated_position[ii] = 0;
    analysis_msg.estimated_velocity[ii] = 0;
    analysis_msg.estimated_acceleration[ii] = 0;
    analysis_msg.estimated_actual_torque[ii] = 0;
  }
  for (size_t ii(segment->phase_end[0]); ii < segment->phase_begin[1]; ++ii) {
    analysis_msg.estimated_position[ii] = 0;
    analysis_msg.estimated_velocity[ii] = 0;
    analysis_msg.estimated_acceleration[ii] = 0;
    analysis_msg.estimated_actual_torque[ii] = 0;
  }
  for (size_t ii(segment->phase_end[1]); ii < ndata; ++ii) {
    analysis_msg.estimated_position[ii] = 0;
    analysis_msg.estimated_velocity[ii] = 0;
    analysis_msg.estimated_acceleration[ii] = 0;
    analysis_msg.estimated_actual_torque[ii] = 0;
  }
  
  analysis_pub_.publish(analysis_msg);
}
