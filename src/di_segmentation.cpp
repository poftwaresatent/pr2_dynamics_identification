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
   \file di_segmentation.cpp
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <dynamics_identification/Start.h>
#include <dynamics_identification/Data.h>
#include <dynamics_identification/Segment.h>
#include <Eigen/LU>
#include <err.h>

using namespace dynamics_identification;
using namespace boost;
using namespace std;


struct setup_s {
  setup_s(Start const & start)
    : joint_name(start.joint_name),
      lower_position(start.lower_position),
      upper_position(start.upper_position),
      torque_magnitude(start.torque_magnitude)
  {
  }
  
  inline bool operator != (Start const & start) const
  {
    return (joint_name != start.joint_name)
      || (lower_position != start.lower_position)
      || (upper_position != start.upper_position)
      || (torque_magnitude != start.torque_magnitude);
  }
  
  string joint_name;
  float lower_position;
  float upper_position;
  float torque_magnitude;
};


struct segment_data_s {
  segment_data_s(size_t setup_id_, size_t segment_id_, size_t smoothing_offset_)
    : valid(false),
      setup_id(setup_id_),
      segment_id(segment_id_),
      average_applied_torque(0),
      smoothing_offset(smoothing_offset_),
      phase1_begin(0),
      phase1_end(0),
      phase2_begin(0),
      phase2_end(0)
  {
  }
  
  inline void toMsg(Segment & msg) {
    msg.valid = valid;
    msg.setup_id = setup_id;
    msg.segment_id = segment_id;
    msg.measured_milliseconds = measured_milliseconds;
    msg.measured_position = measured_position;
    msg.measured_velocity = measured_velocity;
    msg.applied_torque = applied_torque;
    msg.average_applied_torque = average_applied_torque;
    msg.seconds.clear();
    msg.seconds.insert(msg.seconds.begin(),
		       seconds.begin(),
		       seconds.end());
    msg.smoothing_offset = smoothing_offset;
    msg.smoothed_position.clear();
    msg.smoothed_position.insert(msg.smoothed_position.begin(),
				 smoothed_position.begin(),
				 smoothed_position.end());
    msg.smoothed_velocity.clear();
    msg.smoothed_velocity.insert(msg.smoothed_velocity.begin(),
				 smoothed_velocity.begin(),
				 smoothed_velocity.end());
    msg.smoothed_acceleration.clear();
    msg.smoothed_acceleration.insert(msg.smoothed_acceleration.begin(),
				     smoothed_acceleration.begin(),
				     smoothed_acceleration.end());
    msg.phase1_begin = phase1_begin;
    msg.phase1_end = phase1_end;
    msg.phase2_begin = phase2_begin;
    msg.phase2_end = phase2_end;
  }
  
  bool valid;
  size_t setup_id;
  size_t segment_id;
  
  // from Data msg
  vector<float> measured_milliseconds;
  vector<float> measured_position;
  vector<float> measured_velocity;
  vector<float> applied_torque;
  double average_applied_torque;
  
  // scaled from ms and shifted origin s.t. seconds[0] == 0  
  vector<double> seconds;
  
  // s.t. smoothed_position[ii+smoothing_offset] corresponds to measured_position[ii]
  size_t smoothing_offset;
  vector<double> smoothed_position;
  vector<double> smoothed_velocity;
  vector<double> smoothed_acceleration;
  
  // detected phase transitions (zero-crossings of smoothed_velocity)
  // - phase1: decelerating towards zero velocity
  // - phase2: accelerating towards upper/lower positional bound
  //
  // NOTE: For now we do not chop off data at the beginning and end of
  // the phases, so these four indices could be encoded as just
  // one. However, this way it's more explicit, and future compatible.
  //
  size_t phase1_begin;
  size_t phase1_end;
  size_t phase2_begin;
  size_t phase2_end;
};


typedef shared_ptr<setup_s> setup_p;
typedef shared_ptr<segment_data_s> segment_data_p;


class Segmentation {
public:
  explicit Segmentation(size_t smoothing_range);
  
  /**
     Append some data to the internal buffer, smoothing and segmenting
     as we go along. Any segments that get completed via the received
     data will be returned.
     
     \note Use getSetup() with the returned setup_id of the returned
     segment to retrieve the full setup description. Also note that
     the di_controller never sends data that mixes setups, so the
     processData method is a fairly straightforward accumulation
     function (except for the smoothing).
     
     \return A vector of newly completed segments. Can be empty, and
     should usually be fairly small (maybe maximum two elements in
     typical usage scenarios), so returning a vector by value should
     be okay performance-wise.
  */
  vector<segment_data_p> processData(Data const & data);
  
  setup_p getSetup(size_t id) const;
  
protected:
  static bool processSegment(segment_data_s & segment);
  
  size_t const smoothing_range_;
  vector<setup_p> setup_;
  segment_data_p buffer_;
};


static ros::Subscriber data_sub_;
static ros::Publisher segment_pub_;
static shared_ptr<Segmentation> segmentation_;


static void data_cb(shared_ptr<Data const> const & data);


int main(int argc, char*argv[])
{
  ros::init(argc, argv, "di_segmentation");
  ros::NodeHandle nn("~");
  
  size_t smoothing_range(30);
  if (argc > 1) {
    int arg;
    if ((1 != sscanf(argv[1], "%i", &arg)) || (0 >= arg)) {
      errx(EXIT_FAILURE, "error reading argument `%s': positive number expected", argv[1]);
    }
    smoothing_range = arg;
  }
  
  segmentation_.reset(new Segmentation(smoothing_range));
  data_sub_ = nn.subscribe<Data>("/di_controller/data", 1, data_cb);
  segment_pub_ = nn.advertise<Segment>("segment", 100);
  
  ros::spin();
}


void data_cb(shared_ptr<Data const> const & data)
{
  vector<segment_data_p> segments(segmentation_->processData(*data));
  for (vector<segment_data_p>::const_iterator is(segments.begin());
       is != segments.end(); ++is) {
    Segment segment_msg;
    (*is)->toMsg(segment_msg);
    segment_pub_.publish(segment_msg);
  }
}


Segmentation::
Segmentation(size_t smoothing_range)
  : smoothing_range_(smoothing_range)
{
}


vector<segment_data_p> Segmentation::
processData(Data const & data)
{
  vector<segment_data_p> completed_segments;
  
  //////////////////////////////////////////////////
  // reject faulty data
  
  if (data.applied_torque.empty()) {
    ROS_ERROR ("processData(): faulty data: empty torque data");
    return completed_segments;
  }
  if ((data.applied_torque.size() != data.milliseconds.size())
      || (data.applied_torque.size() != data.position.size())) {
    ROS_ERROR ("processData(): faulty data: mismatching sizes");
    return completed_segments;
  }
  
  //////////////////////////////////////////////////
  // detect initialization and setup change
  
  if (setup_.empty()) {
    ROS_INFO ("processData(): initializing buffer");
    buffer_.reset(new segment_data_s(0, 0, smoothing_range_));
    setup_.push_back(setup_p(new setup_s(data.parameters)));
  }
  else if (*setup_.back() != data.parameters) {
    ROS_INFO ("processData(): detected setup change");
    completed_segments.push_back(buffer_);
    buffer_.reset(new segment_data_s(setup_.size(), 0, smoothing_range_));
    setup_.push_back(setup_p(new setup_s(data.parameters)));
  }
  
  //////////////////////////////////////////////////
  // detect sign change in applied torque
  
  if (( ! buffer_->applied_torque.empty())
      && (((buffer_->applied_torque.back() > 0) && (data.applied_torque.front() < 0))
	  || ((buffer_->applied_torque.back() < 0) && (data.applied_torque.front() > 0)))) {
    ROS_INFO ("processData(): sign change in applied torque across data chunks");
    completed_segments.push_back(buffer_);
    buffer_.reset(new segment_data_s(buffer_->setup_id,
				     buffer_->segment_id + 1,
				     smoothing_range_));
  }
  
  size_t const datasize(data.applied_torque.size());
  ROS_INFO ("processData(): datasize is %zu", datasize);
  
  for (size_t at_start(0); at_start < datasize; /**/) {
    
    float at0(data.applied_torque[at_start]);
    if (0 == at0) {
      ROS_INFO ("processData(): skipping zero torque");
      for (++at_start; at_start < datasize; ++at_start) {
	at0 = data.applied_torque[at_start];
	if (0 != at0) {
	  break;
	}
      }
      if (at_start >= datasize) {
	ROS_INFO ("processData(): no non-zero torque left");
	break;
      }
    }
    ROS_INFO ("processData(): non-zero torque %g at index %zu", at0, at_start);
    
    size_t at_change(at_start + 1);
    if (at0 > 0) {
      for (/**/; at_change < datasize; ++at_change) {
	if (data.applied_torque[at_change] < 0) {
	  ROS_INFO ("processData(): torque sign change at index %zu", at_change);
	  break;
	}
      }
    }
    else {
      for (/**/; at_change < datasize; ++at_change) {
	if (data.applied_torque[at_change] > 0) {
	  ROS_INFO ("processData(): torque sign change at index %zu", at_change);
	  break;
	}
      }
    }
    
    ROS_INFO ("processData(): appending %zu entries to buffer of length %zu",
	      at_change - at_start, buffer_->measured_milliseconds.size());
    buffer_->measured_milliseconds.insert(buffer_->measured_milliseconds.end(),
					  data.milliseconds.begin() + at_start,
					  data.milliseconds.begin() + at_change);
    buffer_->measured_position.insert(buffer_->measured_position.end(),
				      data.position.begin() + at_start,
				      data.position.begin() + at_change);
    buffer_->measured_velocity.insert(buffer_->measured_velocity.end(),
				      data.velocity.begin() + at_start,
				      data.velocity.begin() + at_change);
    buffer_->applied_torque.insert(buffer_->applied_torque.end(),
				   data.applied_torque.begin() + at_start,
				   data.applied_torque.begin() + at_change);
    ROS_INFO ("processData(): buffer grown to %zu entries", buffer_->measured_milliseconds.size());
    
    if (at_change < datasize) {
      // sign change within
      ROS_INFO ("processData(): completed segment from %zu to %zu", at_start, at_change);
      completed_segments.push_back(buffer_);
      buffer_.reset(new segment_data_s(buffer_->setup_id,
				       buffer_->segment_id + 1,
				       smoothing_range_));
    }
    
    at_start = at_change;
  }
  
  //////////////////////////////////////////////////
  // We have extracted the segments based on sign changes of the
  // applied torque, now we just have to smooth out their velocity
  // data to detect their phases.
  
  for (vector<segment_data_p>::const_iterator is(completed_segments.begin());
       is != completed_segments.end(); ++is) {
    (*is)->valid = processSegment(**is);
  }
  
  return completed_segments;
}


bool Segmentation::
processSegment(segment_data_s & segment)
{
  ROS_INFO ("processSegment(): setup: %zu  segment: %zu", segment.setup_id, segment.segment_id);
  
  size_t const twice_range(2 * segment.smoothing_offset);
  size_t const regr_npoints(twice_range + 1);
  size_t const datasize(segment.measured_milliseconds.size());
  if (datasize < regr_npoints) {
    ROS_ERROR ("processSegment(): %zu data points, need at least %zu for regression",
	       datasize, regr_npoints);
    return false;
  }
  
  //////////////////////////////////////////////////
  // compute average applied torque and shifted timescale
  
  segment.average_applied_torque = 0;
  segment.seconds.resize(datasize);
  float const ms0(segment.measured_milliseconds[0]);
  for (size_t ii(0); ii < datasize; ++ii) {
    segment.average_applied_torque += segment.applied_torque[ii];
    segment.seconds[ii] = 1e-3 * (segment.measured_milliseconds[ii] - ms0);
  }
  segment.average_applied_torque /= datasize;
  ROS_INFO ("processSegment(): average applied torque %g", segment.average_applied_torque);
  
  //////////////////////////////////////////////////
  // compute smoothed trajectory for subsequent phase detection
  
  size_t const smoothed_npoints(datasize - twice_range);
  ROS_INFO ("processSegment(): smoothing %zu points", smoothed_npoints);
  
  segment.smoothed_position.resize(smoothed_npoints);
  segment.smoothed_velocity.resize(smoothed_npoints);
  segment.smoothed_acceleration.resize(smoothed_npoints);
  
  for (size_t i_out(0); i_out < smoothed_npoints; ++i_out) {
    
    // Polynomial regression of q(t) = a0 + a1*t + a2*t^2
    // - yy = vector of measured positions
    // - XX = matrix of time samples raised to power 0, 1, ...
    // - aa = vector of polynomial parameters a0, a1, ...
    Eigen::VectorXd yy(regr_npoints);
    Eigen::MatrixXd XX(regr_npoints, 3);
    size_t i_in(i_out);
    // The regression time origin is centered on smoothing window for
    // better results and for more straightforward equations for the
    // smoothed pos, vel, and acc further down.
    double const t0(segment.seconds[i_in + segment.smoothing_offset]);
    for (size_t i_regr(0); i_regr < regr_npoints; ++i_regr, ++i_in) {
      yy[i_regr] = segment.measured_position[i_in];
      double const ts(segment.seconds[i_in] - t0);
      XX.coeffRef(i_regr, 0) = 1;
      XX.coeffRef(i_regr, 1) = ts; // yes yes, ts will be zero on center sample, so what?
      XX.coeffRef(i_regr, 2) = pow(ts, 2);
    }
    Eigen::MatrixXd toto(XX.transpose() * XX);
    Eigen::VectorXd const aa(toto.inverse() * XX.transpose() * yy);
    
    segment.smoothed_position[i_out] = aa[0];
    segment.smoothed_velocity[i_out] = aa[1];
    segment.smoothed_acceleration[i_out] = 2 * aa[2];
  }
  
  //////////////////////////////////////////////////
  // detect zero-crossing of smoothed_velocity, assuming we only have
  // one such sign change (which is not guaranteed when segments do
  // not come from clean experiments, so let's at least flag such
  // segments as invalid).
  
  double vel0(segment.smoothed_velocity[0]);
  size_t vel_change(1);
  
  // ...make sure to skip over leading zeroes...
  if (0 == vel0) {
    ROS_INFO ("processSegment(): skipping zero velocity");
    size_t vel_start(1);
    for (/**/; vel_start < smoothed_npoints; ++vel_start) {
      vel0 = segment.smoothed_velocity[vel_start];
      if (0 != vel0) {
	break;
      }
    }
    if (vel_start >= smoothed_npoints) {
      ROS_ERROR ("processSegment(): no non-zero velocity, setup %zu, segment %zu",
		 segment.setup_id, segment.segment_id);
      return false;
    }
    ROS_INFO ("processSegment(): non-zero velocity %g at index %zu", vel0, vel_start);
    vel_change = vel_start + 1;
  }
  
  if (vel0 > 0) {
    for (/**/; vel_change < smoothed_npoints; ++vel_change) {
      if (segment.smoothed_velocity[vel_change] < 0) {
	ROS_INFO ("processSegment(): zero crossing to <0 at %zu +offset", vel_change);
	break;
      }
    }
  }
  else {
    for (/**/; vel_change < smoothed_npoints; ++vel_change) {
      if (segment.smoothed_velocity[vel_change] > 0) {
	ROS_INFO ("processSegment(): zero crossing to >0 at %zu +offset", vel_change);
	break;
      }
    }
  }
  if (vel_change >= smoothed_npoints) {
    ROS_ERROR ("smoothed velocity does not change sign, setup %zu, segment %zu",
	       segment.setup_id, segment.segment_id);
    return false;
  }
  
  size_t second_vel_change(vel_change + 1);
  // NOTE: invert the sign check (from `>' to `<') because we are
  // looking for a transition back to the original sign!
  if (vel0 < 0) {
    for (/**/; second_vel_change < smoothed_npoints; ++second_vel_change) {
      if (segment.smoothed_velocity[second_vel_change] < 0) {
	ROS_INFO ("processSegment(): 2nd zero crossing to <0 at %zu +offset", second_vel_change);
	break;
      }
    }
  }
  else {
    for (/**/; second_vel_change < smoothed_npoints; ++second_vel_change) {
      if (segment.smoothed_velocity[second_vel_change] > 0) {
	ROS_INFO ("processSegment(): 2nd zero crossing to >0 at %zu +offset", second_vel_change);
	break;
      }
    }
  }
  if (second_vel_change < smoothed_npoints) {

#define SECOND_CROSSING_IS_ERROR
#ifdef SECOND_CROSSING_IS_ERROR

    ROS_ERROR ("smoothed velocity changes sign more than once, setup %zu, segment %zu",
	       segment.setup_id, segment.segment_id);
    return false;

#else // SECOND_CROSSING_IS_ERROR

    ROS_WARN ("smoothed velocity changes sign more than once, setup %zu, segment %zu",
	      segment.setup_id, segment.segment_id);

#endif // SECOND_CROSSING_IS_ERROR
  }
  
  //////////////////////////////////////////////////
  // Hooray we passed all tests!
  
  ROS_INFO ("processSegment(): segment is valid");
  
  segment.valid = true;
  segment.phase1_begin = 0;
  segment.phase1_end = vel_change + segment.smoothing_offset;
  segment.phase2_begin = segment.phase1_end;
  segment.phase2_end = second_vel_change + segment.smoothing_offset; // == datasize most of the time
  
  return true;
}
