#include "atrv/atrv.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/***** Inline Functions *****/

namespace atrv_ {

inline void defaultInfoCallback(const std::string &msg) {
  std::cout << "ATRV Info: " << msg << std::endl;
}

inline void defaultExceptionCallback(const std::exception &error) {
  std::cerr << "ATRV Unhandled Exception: " << error.what();
  std::cerr << std::endl;
  throw(error);
}

}

using namespace atrv;
using namespace atrv_;
using namespace mdc2250;

/***** MDC2250 Class Functions *****/

ATRV::ATRV() {
  // Set default callbacks
  this->handle_exc = defaultExceptionCallback;
  this->info = defaultInfoCallback;

  // Hook into the MDC2250 logging
  front_mc_.setInfoHandler(boost::bind(&ATRV::info_cb_, this, _1, 1));
  front_mc_.setExceptionHandler(boost::bind(&ATRV::exc_cb_, this, _1, 1));
  rear_mc_.setInfoHandler(boost::bind(&ATRV::info_cb_, this, _1, 2));
  rear_mc_.setExceptionHandler(boost::bind(&ATRV::exc_cb_, this, _1, 1));

  // Set vehicle parameters
  track_width_ = 0.76; // meters
  wheel_radius_ = 0.203; // meters
  max_rpm_ = 3000; // rpm
  encoder_ppr_ = 1000; // ppr
}

ATRV::~ATRV() {
  this->disconnect();
}

void
ATRV::connect(std::string port1, std::string port2,
              size_t watchdog, bool echo)
{
  // Connect to both motor controllers, parallelize to speed up
  front_mc_error_ = "";
  rear_mc_error_ = "";
  boost::thread t1(
    boost::bind(&ATRV::connect_, this, 1, port1, watchdog, echo));
  boost::thread t2(
    boost::bind(&ATRV::connect_, this, 2, port2, watchdog, echo));
  t1.join();
  t2.join();
  if (!front_mc_error_.empty())
    throw(ConnectionFailedException("Front mdc2250: "+front_mc_error_));
  if (!rear_mc_error_.empty())
    throw(ConnectionFailedException("Rear mdc2250: "+rear_mc_error_));

  // Setup telemetry
  std::string telem = "C,FF,C,V,C,BA,C,T,C,A";
  size_t period = 10;
  front_mc_.setTelemetry(telem, period, boost::bind(&ATRV::parse_telemetry_,
                                                    this, 1, _1));
  rear_mc_.setTelemetry(telem, period, boost::bind(&ATRV::parse_telemetry_,
                                                   this, 2, _1));
}

void
ATRV::disconnect() {
  front_mc_error_ = "";
  rear_mc_error_ = "";
  boost::thread t1(boost::bind(&ATRV::disconnect_, this, 1));
  boost::thread t2(boost::bind(&ATRV::disconnect_, this, 2));
  t1.join();
  t2.join();
  if (!front_mc_error_.empty())
    throw(ConnectionFailedException("Front mdc2250: "+front_mc_error_));
  if (!rear_mc_error_.empty())
    throw(ConnectionFailedException("Rear mdc2250: "+rear_mc_error_));
}

void
ATRV::move(ssize_t linear_velocity, ssize_t angular_velocity) {
  // Calculate the required wheel velocities in rpm
  double lws, rws;
  lws = 2 * linear_velocity;
  lws += angular_velocity * this->track_width_;
  lws /= 4 * M_PI * this->wheel_radius_;
  lws *= 60; // Minutes to seconds
  rws = 2 * linear_velocity;
  rws -= angular_velocity * this->track_width_;
  rws /= 4 * M_PI * this->wheel_radius_;
  rws *= 60; // Minutes to seconds

  // Calculate rpm as an effort represented as a percentage of max_rpm_
  // (lws / max_rpm) * max_effort_value * gear_ratio
  boost::mutex::scoped_lock lock(move_mux);
  left_wheel_effort_ = (lws / (double)this->max_rpm_) * 1000.0 * 11.0;
  right_wheel_effort_ = (rws / (double)this->max_rpm_) * 1000.0 * 11.0;

  // Issue command
  this->front_mc_.commandMotors(left_wheel_effort_, right_wheel_effort_);
  this->rear_mc_.commandMotors(left_wheel_effort_, right_wheel_effort_);
}

void
ATRV::setTelemetryCallback (TelemetryCallback telemetry_callback,
                            mdc2250::queries::QueryType query_type)
{
  telemetry_cb_map_[query_type] = telemetry_callback;
}

void
ATRV::connect_(size_t mc_index, const std::string &port, size_t wd, bool echo)
{
  if (mc_index == 1) {
    try {
      front_mc_.connect(port, wd, echo);
    } catch (const std::exception &e) {
      front_mc_error_ = e.what();
    }
  } else if (mc_index == 2) {
    try {
      rear_mc_.connect(port, wd, echo);
    } catch (const std::exception &e) {
      rear_mc_error_ = e.what();
    }
  } else {
    front_mc_error_ = "Invalid mc_index, must be 1 or 2.";
  }
}

void
ATRV::disconnect_(size_t mc_index) {
  if (mc_index == 1) {
    try {
      front_mc_.disconnect();
    } catch (const std::exception &e) {
      front_mc_error_ = e.what();
    }
  } else if (mc_index == 2) {
    try {
      rear_mc_.disconnect();
    } catch (const std::exception &e) {
      rear_mc_error_ = e.what();
    }
  } else {
    front_mc_error_ = "Invalid mc_index, must be 1 or 2.";
  }
}

void
ATRV::parse_telemetry_(size_t motor_index, const std::string &msg) {
  using namespace queries;
  QueryType query_type = detect_response_type(msg);

  std::vector<long> data;
  decode_generic_response(msg, data);

  // Grab the ?FF fault flag
  if (query_type == fault_flag && data[0] == 16) {
    std::stringstream ss;
    if (motor_index == 1) {
      ss << "Front ";
    } else {
      ss << "Rear ";
    }
    ss << "motor controller has been e-stopped.";
    this->info(ss.str());
  }

  if (telemetry_cb_map_.count(query_type) > 0) {
    telemetry_cb_map_[query_type](motor_index, query_type, data);
  } else // No specific callback, try generic
  if (telemetry_cb_map_.count(any_query) > 0){ 
    telemetry_cb_map_[any_query](motor_index, query_type, data);
  }
}

void
ATRV::info_cb_(const std::string &msg, size_t mc_index) {
  std::stringstream ss;
  if (mc_index == 1) {
    ss << "Front";
  } else if (mc_index == 2) {
    ss << "Rear";
  }
  ss << " motor controller: " << msg;
  this->info(ss.str());
}

void
ATRV::exc_cb_(const std::exception &error, size_t mc_index) {
  ATRVException e(error.what(), mc_index);
  this->handle_exc(e);
}



