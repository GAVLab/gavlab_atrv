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

  // // Set vehicle parameters
  // track_width_ = 0.9985; // meters // 0.80321912 says geometry
  // lwheel_radius = 0.1946275; // meters // will change periodically
  // rwheel_radius = 0.19025; // meters // will change periodically
  // max_rpm_ = 1000; // rpm 
  // max_motor_velocity = 1.75; 
  // reduction_ratio = 0.5f/11.0f;
  // encoder_ppr_ = 2000; // ppr 
  // left_wheel_circumference = lwheel_radius * 2.0f * M_PI; 
  // right_wheel_circumference = rwheel_radius * 2.0f * M_PI; 

  // Set Caster Wheel Parameters
  track_width_ = 0.6731; // meters
  lwheel_radius = 0.20; // meters // will change periodically
  rwheel_radius = 0.20; // meters // will change periodically
  max_rpm_ = 1000; // rpm 
  max_motor_velocity = 1.75; 
  reduction_ratio = 0.5f/11.0f;
  encoder_ppr_ = 2000; // ppr 
  left_wheel_circumference = lwheel_radius * 2.0f * M_PI; 
  right_wheel_circumference = rwheel_radius * 2.0f * M_PI;

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
  lws /= 4 * M_PI * this->lwheel_radius;
  lws *= 60; // Minutes to seconds
  rws = 2 * linear_velocity;
  rws -= angular_velocity * this->track_width_;
  rws /= 4 * M_PI * this->rwheel_radius;
  rws *= 60; // Minutes to seconds

  /////////////////////////////////////////////////////////////////////////////////////
//   double l_rpm= ((linear_velocity)+angular_velocity*(track_width_/2))*60/(2*M_PI*wheel_radius_);
//   double r_rpm= ((linear_velocity)-angular_velocity*(track_width_/2))*60/(2*M_PI*wheel_radius_);

// ////////////////////////////////////////////////////////////////////////////////////////
  // Calculate rpm as an effort represented as a percentage of max_rpm_
  // (lws / max_rpm) * max_effort_value * gear_ratio
  

  // if (lws > (double)this->max_rpm_){
  //   lws = (double)this->max_rpm_;
  // }
  // if (lws < -(double)this->max_rpm_){
  //   lws = -(double)this->max_rpm_;
  // }
  //   if (rws > (double)this->max_rpm_){
  //   rws = (double)this->max_rpm_;
  // }
  // if (rws < -(double)this->max_rpm_){
  //   rws = -(double)this->max_rpm_;
  // }

  boost::mutex::scoped_lock lock(move_mux);
  left_wheel_effort_ = (lws / (double)this->max_rpm_) * 500.0 * 1.0/reduction_ratio;
  right_wheel_effort_ = (rws / (double)this->max_rpm_) * 500.0 * 1.0/reduction_ratio;  //1/.466 for pully ratio
  // left_wheel_effort_ = (((linear_velocity)+angular_velocity*(track_width_/2)) / max_motor_velocity) * 1000.0 * 11.0;
  // right_wheel_effort_ = (((linear_velocity)-angular_velocity*(track_width_/2)) / max_motor_velocity) * 1000.0 * 11.0;


  if (left_wheel_effort_ >= 1000){
    left_wheel_effort_ = 999;
  }
  if (left_wheel_effort_ <= -1000){
    left_wheel_effort_ = -999;
  }
    if (right_wheel_effort_ >= 1000){
    right_wheel_effort_ = 999;
  }
  if (right_wheel_effort_ <= -1000){
    right_wheel_effort_ = -999;
  }
  //std:: cout << "left_wheel_effort_: " << left_wheel_effort_ <<  std::endl;
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
  
  // Calculate Individual Wheel Speed in m/s
  double left_wheel_speed = ((double)encoder1/(double)(this->encoder_ppr_))*((double)(this->reduction_ratio)/delta_time)*(this->lwheel_radius*2.0f*M_PI);
  double right_wheel_speed = ((double)encoder2/(double)(this->encoder_ppr_))*((double)(this->reduction_ratio)/delta_time)*(this->rwheel_radius*2.0f*M_PI);
  
  // Calculate Linear Velocity
  double velocity = (left_wheel_speed + right_wheel_speed)/2.0f;

  // Calculate Angular Velocity
  double angular_velocity = (right_wheel_speed - left_wheel_speed)/(double)this->track_width_;

  // Update Position
  x += delta_time*velocity*cos(theta + (angular_velocity/2.0f)*delta_time);
  y += delta_time*velocity*sin(theta + (angular_velocity/2.0f)*delta_time);
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
    
    // Set to open loop mode
    // std::string fail_why = "";
    // std::stringstream cmd;
    // cmd.str("^MMOD 1");
    // cmd << 1;
    // if (error_str.empty() && mc->issueCommand(cmd.str(), fail_why)) {
    //   error_str = fail_why;
    // }
    // cmd.str("^MMOD 2");
    // cmd << 1;
    // if (error_str.empty() && mc->issueCommand(cmd.str(), fail_why)) {
    //   error_str = fail_why;
    // }
    
    // Setup encoder ppr
    std::string fail_why = "";
    std::stringstream cmd;
    cmd.str("^EPPR 1");
    cmd << this->encoder_ppr_;
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