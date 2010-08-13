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
#include <dynamics_identification/Start.h>
#include <dynamics_identification/Data.h>
#include <dynamics_identification/Analysis.h>
#include <err.h>

#undef USE_SVD
#ifndef USE_SVD
# include <Eigen/LU>
#else // USE_SVD
# include <Eigen/SVD>
#endif // USE_SVD

using namespace dynamics_identification;
using namespace boost;

static ros::Subscriber data_sub_;
static ros::Publisher analysis_pub_;
static size_t smoothing_range_(3);

static void data_cb(shared_ptr<Data const> const & data);

int main(int argc, char*argv[])
{
  ros::init(argc, argv, "di_analysis");
  ros::NodeHandle nn("~");
  
  if (argc > 1) {
    int arg;
    if ((1 != sscanf(argv[1], "%i", &arg)) || (0 >= arg)) {
      errx(EXIT_FAILURE, "error reading argument `%s': positive number expected", argv[1]);
    }
    smoothing_range_ = arg;
  }
  
  data_sub_ = nn.subscribe<Data>("/di_controller/data", 1, data_cb);
  analysis_pub_ = nn.advertise<Analysis>("analysis", 1);
  
  ros::spin();
}
  
void data_cb(shared_ptr<Data const> const & data)
{
  size_t const twice_range(2 * smoothing_range_);
  size_t const regr_npoints(twice_range + 1);
  if (data->parameters.chunksize < regr_npoints) {
    ROS_ERROR ("insufficient data points for regression (%zu points, need at least %zu)",
	       data->parameters.chunksize, regr_npoints);
    return;
  }
  
  size_t const analysis_npoints(data->parameters.chunksize - twice_range);
  
  Analysis analysis_msg;
  analysis_msg.measurement_parameters = data->parameters;
  analysis_msg.smoothing_range = smoothing_range_;
  analysis_msg.milliseconds.resize(analysis_npoints);
  analysis_msg.measured_position.resize(analysis_npoints);
  analysis_msg.measured_velocity.resize(analysis_npoints);
  analysis_msg.applied_torque.resize(analysis_npoints);
  analysis_msg.regression_0.resize(analysis_npoints);
  analysis_msg.regression_1.resize(analysis_npoints);
  analysis_msg.regression_2.resize(analysis_npoints);
  analysis_msg.estimated_position.resize(analysis_npoints);
  analysis_msg.estimated_velocity.resize(analysis_npoints);
  analysis_msg.estimated_acceleration.resize(analysis_npoints);
  analysis_msg.inertia.resize(analysis_npoints);
  
  for (size_t i_out(0); i_out < analysis_npoints; ++i_out) {
    
    size_t i_in(i_out + smoothing_range_);
    analysis_msg.milliseconds[i_out] = data->milliseconds[i_in];
    analysis_msg.measured_position[i_out] = data->position[i_in];
    analysis_msg.measured_velocity[i_out] = data->velocity[i_in];
    analysis_msg.applied_torque[i_out] = data->applied_torque[i_in];
    
    // Polynomial regression of q(t) = a0 + a1*t + a2*t^2
    // - yy = vector of measured positions
    // - XX = matrix of time samples raised to power 0, 1, ...
    // - aa = vector of polynomial parameters a0, a1, ...
    Eigen::VectorXd yy(regr_npoints);
    Eigen::MatrixXd XX(regr_npoints, 3);
    i_in = i_out;
    double const t0(analysis_msg.milliseconds[i_out] * 1e-3);
    for (size_t i_regr(0); i_regr < regr_npoints; ++i_regr, ++i_in) {
      yy[i_regr] = data->position[i_in];
      double const ts((data->milliseconds[i_in] - analysis_msg.milliseconds[i_out]) * 1e-3);
      XX.coeffRef(i_regr, 0) = 1;
      XX.coeffRef(i_regr, 1) = ts;
      XX.coeffRef(i_regr, 2) = pow(ts, 2);
    }
    
#ifndef USE_SVD

    Eigen::MatrixXd toto(XX.transpose() * XX);
    Eigen::VectorXd const aa(toto.inverse() * XX.transpose() * yy);
    
#else // USE_SVD

    // Least-squares via SVD. Should be just a plain inversion though,
    // weird that it did not always work.
    Eigen::SVD<Eigen::MatrixXd> svd(XX.transpose() * XX);
    Eigen::MatrixXd toto(Eigen::MatrixXd::Zero(3, 3));
    Eigen::VectorXd const & sigma(svd.singularValues());
    for (size_t jj(0); jj < 3; ++jj) {
      if (sigma[jj] > 1e-3) {
	toto(jj, jj) = 1 / sigma[jj];
      }
    }
    Eigen::MatrixXd pseudoinv(svd.matrixV() * toto * svd.matrixU());
    Eigen::VectorXd const aa(pseudoinv * XX.transpose() * yy);

#endif // USE_SVD
    
    double const tnow(analysis_msg.milliseconds[i_out] * 1e-3);
    analysis_msg.regression_0[i_out] = aa[0];
    analysis_msg.regression_1[i_out] = aa[1];
    analysis_msg.regression_2[i_out] = aa[2];
    analysis_msg.estimated_position[i_out] = aa[0];
    analysis_msg.estimated_velocity[i_out] = aa[1];
    analysis_msg.estimated_acceleration[i_out] = 2 * aa[2];
    analysis_msg.inertia[i_out] = analysis_msg.applied_torque[i_out] / 2 / aa[2];
  }
  
  analysis_pub_.publish(analysis_msg);
}
