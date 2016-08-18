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

  this->connected = false;
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
    boost::bind(&ATRV::connect_, this, &front_mc_, 1, port1, watchdog, echo));
  boost::thread t2(
    boost::bind(&ATRV::connect_, this, &rear_mc_, 2, port2, watchdog, echo));
  t1.join();
  t2.join();
  if (!front_mc_error_.empty())
    throw(ConnectionFailedException("Front mdc2250: "+front_mc_error_));
  if (!rear_mc_error_.empty())
    throw(ConnectionFailedException("Rear mdc2250: "+rear_mc_error_));
  this->connected = true;
}

void
ATRV::disconnect() {
  this->connected = false;
  boost::mutex::scoped_lock lock(move_mux);
  front_mc_error_ = std::string("");
  rear_mc_error_ = std::string("");
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
ATRV::move(double linear_velocity, double angular_velocity) {
  if (!this->connected) {
    return;
  }
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

bool
ATRV::calculateOdometry(long &encoder1, long &encoder2, double &delta_time,
                        double &x, double &y, double &theta)
{
  // Divide the relative encoder counts by 11 for the gear ratio
  double left_wheel_speed = encoder1 / 11.0f;
  double right_wheel_speed = encoder2 / 11.0f;
  // Calculate the wheel speed from the delta_time
  left_wheel_speed /= delta_time;
  right_wheel_speed /= delta_time;
  // Divide by 1000 to convert from encoder_ticks_per_second to rps
  left_wheel_speed /= 1000.0f;
  right_wheel_speed /= 1000.0f;
  // Convert rps to mps for each wheel
  double wheel_circumference = this->wheel_radius_ * 2.0f * M_PI;
  left_wheel_speed *= wheel_circumference;
  right_wheel_speed *= wheel_circumference;
  // Calculate velocities
  double velocity = 0.0f;
  velocity += this->wheel_radius_ * right_wheel_speed;
  velocity += this->wheel_radius_ * left_wheel_speed;
  velocity /= 2.0f;
  double angular_velocity = 0.0f;
  angular_velocity -= this->wheel_radius_/this->track_width_ * left_wheel_speed;
  angular_velocity += this->wheel_radius_/this->track_width_ * right_wheel_speed;
  // Calculate new positions
  x += delta_time * velocity * cos(theta + (angular_velocity/2.0f) * delta_time);
  y += delta_time * velocity * sin(theta + (angular_velocity/2.0f) * delta_time);
  theta += angular_velocity * delta_time;
  return true;
}

void
ATRV::connect_(MDC2250 *mc, size_t i, const std::string &port,
               size_t wd, bool echo)
{
  if (i != 1 && i != 2) {
    front_mc_error_ = "Invalid mc_index, must be 1 or 2.";
    return;
  }
  std::string error_str = "";
  try {
    mc->connect(port, wd, echo);
    // Setup telemetry
    std::string telem = "C,FF,C,V,C,BA,C,T,C,A";
    // std::string telem = "CR,FF,CR,V,CR,BA,CR,T,CR,A";
    size_t period = 10;
    mc->setTelemetry(telem, period,
                    boost::bind(&ATRV::parse_telemetry_,this,i,_1));
    // Setup encoder ppr
    std::string fail_why = "";
    std::stringstream cmd;
    cmd << "^EPPR 1" << this->encoder_ppr_;
    if (error_str.empty() && mc->issueCommand(cmd.str(), fail_why)) {
      error_str = fail_why;
    }
    cmd.str("^EPPR 2");
    cmd << this->encoder_ppr_;
    if (error_str.empty() && mc->issueCommand(cmd.str(), fail_why)) {
      error_str = fail_why;
    }
    // Setup max rpm
    cmd.str("^MRPM 1");
    cmd << this->max_rpm_;
    if (error_str.empty() && mc->issueCommand(cmd.str(), fail_why)) {
      error_str = fail_why;
    }
    cmd.str("^MRPM 2");
    cmd << this->max_rpm_;
    if (error_str.empty() && mc->issueCommand(cmd.str(), fail_why)) {
      error_str = fail_why;
    }
  } catch (const std::exception &e) {
    error_str = e.what();
  }
  if (!error_str.empty() && i == 1) {
    front_mc_error_ = error_str;
  }
  if (!error_str.empty() && i == 2) {
    rear_mc_error_ = error_str;
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



