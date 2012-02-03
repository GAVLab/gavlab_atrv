/*
 * The MIT License (MIT)
 * Copyright (c) 2011 William Woodall <wjwwood@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <sstream>
#include <cmath>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "gavlab_atrv_node/StampedEncoders.h"

#include "atrv/atrv.h"

// Convenience variables
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double radians_to_degrees = 180.0 / M_PI;
static double degrees_to_radians = M_PI / 180.0;

// Message Wrappers
void handleInfoMessages(const std::string &msg) {ROS_INFO("%s",msg.c_str());}

// ROS Node class
class ATRVNode {
public:
  ATRVNode() : n("~") {
    this->connected = false;
    this->first_odometry = true;
    this->odometry_x = 0.0;
    this->odometry_y = 0.0;
    this->odometry_w = 0.0;
  }

  ~ATRVNode() {
    this->disconnect();
  }

  void disconnect() {
    this->atrv_->disconnect();
    delete this->atrv_;
    this->atrv_ = NULL;
    this->connected = false;
  }

  void run() {
    if (!this->getParameters())
      return;

    this->setupROSComms();

    // Setup keep alive timer
    this->keep_alive_timer = n.createTimer(ros::Duration(1.0/20.0),
                                           &ATRVNode::keepAliveCallback,
                                           this);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    this->connected = false;
    while (ros::ok()) {
      try {
        ROS_INFO("Connecting to ATRV with front_port: %s, and rear_port: %s",
                 this->front_port.c_str(), this->rear_port.c_str());
        this->setupATRV();
        this->atrv_->connect(this->front_port, this->rear_port, 250, false);
        this->connected = true;
      } catch (const std::exception& e) {
        std::string e_msg(e.what());
        ROS_ERROR("Exception while connecting to the ATRV, "
                  "check your cables and power buttons:");
        ROS_ERROR("    %s", e_msg.c_str());
        this->connected = false;
      }
      if (ros::ok() && this->connected) {
        ROS_INFO("ATRV Ready.");
        while (ros::ok() && this->connected) {
          ros::Duration(0.1).sleep();
        }
      }
      if (ros::ok()) {
        // ROS is OK, but we aren't connected, wait then try again
        ROS_WARN("Not connected to the ATRV, will retry in "
                 "5 seconds...");
        // Wait 5 seconds, 0.1 seconds at a time
        for(size_t i = 0; i < 50; ++i) {
          ros::Duration(0.1).sleep();
          if (!ros::ok())
            break;
        }
      } // if
    } // while
  } // function

  void keepAliveCallback(const ros::TimerEvent& e) {
    if (ros::ok() && this->connected) {
      boost::mutex::scoped_lock lock(this->m_mutex);
      try {
        this->atrv_->move(this->linear_vel, this->angular_vel);
      } catch (std::exception& e) {
        std::string e_msg(e.what());
        ROS_ERROR("Error commanding ATRV: %s", e_msg.c_str());
        this->disconnect();
      }
    }
  }

  void motor_timeoutCallback(const ros::TimerEvent& e) {
    boost::mutex::scoped_lock lock(m_mutex);
    this->linear_vel = 0.0;
    this->angular_vel = 0.0;
  }

  void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!this->connected)
      return;
    boost::mutex::scoped_lock lock(m_mutex);
    double x = msg->linear.x, z = msg->angular.z;
    this->linear_vel = x;
    this->angular_vel = z;
    this->motor_timeout_timer =
      this->n.createTimer(ros::Duration(this->motor_timeout),
                          &ATRVNode::motor_timeoutCallback, this, true);
  }
private:
  // Functions
  bool getParameters() {
    // Get the serial ports
    n.param("front_port", this->front_port, std::string("/dev/default"));
    n.param("rear_port", this->rear_port, std::string("/dev/default"));

    // Get Setup Motor Timeout
    n.param("motor_timeout", this->motor_timeout, 0.5);

    // Get frame id parameter
    n.param("frame_id", this->frame_id, std::string("base_link"));
    this->front_encoder_msg.header.frame_id = this->frame_id;
    this->rear_encoder_msg.header.frame_id = this->frame_id;

    // Get option for enable/disable tf broadcasting
    n.param("broadcast_tf", this->broadcast_tf, true);

    return true;
  }

  void
  encoderCallback(size_t motor_index,
                  const mdc2250::queries::QueryType &query_type,
                  std::vector<long> &telemetry)
  {
    ros::Time now = ros::Time::now();
    using namespace mdc2250::queries;
    if (query_type != encoder_count_absolute || telemetry.size() != 2) {
      return;
    }
    // How long has it been?
    // double delta_t = 1.0/50.0;
    double delta_t = (now-this->last_time_).toSec();

    // Publish encoder data
    if (motor_index == 1) {
      this->front_encoder_msg.header.stamp = now;
      this->front_encoder_msg.left = telemetry[0];
      this->front_encoder_msg.right = telemetry[1];
      this->front_encoder_pub.publish(this->front_encoder_msg);
    } else if (motor_index == 2) {
      this->rear_encoder_msg.header.stamp = now;
      this->rear_encoder_msg.left = telemetry[0];
      this->rear_encoder_msg.right = telemetry[1];
      this->rear_encoder_pub.publish(this->rear_encoder_msg);
    }

    // Only use the rear motors for odometry
    if (motor_index == 1) {
      return;
    }

    return; // Odom is not complete

    // Store time for next iteration
    this->last_time_ = now;
    // Is this the encoder data?
    if (this->first_odometry) {
      this->first_odometry = false;
      this->lwc_ = telemetry[0];
      this->rwc_ = telemetry[1];
      this->last_time_ = now;
      return;
    }
    // Gear ratio of 11 to 1
    double lws = (telemetry[0] - this->lwc_) / 11.0;
    double rws = (telemetry[1] - this->rwc_) / 11.0;
    // Set previous telemetry for the next go around
    this->lwc_ = telemetry[0];
    this->rwc_ = telemetry[1];

    // Calculate the revolutions per second
    lws /= delta_t;
    rws /= delta_t;
    // Use vehicle geometry to calculate delta position
    
  }

  void
  telemetryCallback(size_t motor_index,
                  const mdc2250::queries::QueryType &query_type,
                  std::vector<long> &telemetry)
  {
    return;
  }

  void
  handleExceptions(const std::exception &error) {
    ROS_ERROR("ATRV: %s", error.what());
    this->disconnect();
  }

  void setupATRV() {
    if (this->atrv_ != NULL) {
      this->disconnect();
    }
    this->atrv_ = new atrv::ATRV();
    // Setup telemetry
    using namespace mdc2250::queries;
    this->atrv_->setTelemetryCallback(boost::bind(&ATRVNode::encoderCallback,
                                                 this, _1, _2, _3),
                                     encoder_count_absolute);
    this->atrv_->setTelemetryCallback(boost::bind(&ATRVNode::telemetryCallback,
                                                 this, _1, _2, _3),
                                     any_query);
    // Replace the info callback
    this->atrv_->setInfoHandler(handleInfoMessages);
    this->atrv_->setExceptionHandler(boost::bind(&ATRVNode::handleExceptions,
                                                this, _1));
  }

  void setupROSComms() {
    // Subscribe to command velocities
    this->cmd_velSubscriber =
      n.subscribe("cmd_vel", 1000, &ATRVNode::cmd_velCallback, this);
    // Advertise the Odometry Msg
    this->odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    // Advertise the encoders
    this->front_encoder_pub =
      n.advertise<gavlab_atrv_node::StampedEncoders>("front/encoders", 50);
    this->rear_encoder_pub =
      n.advertise<gavlab_atrv_node::StampedEncoders>("rear/encoders", 50);
  }

  // Variables
  ros::NodeHandle n;

  ros::Timer keep_alive_timer;

  ros::Subscriber cmd_velSubscriber;
  ros::Publisher odom_pub;
  ros::Publisher front_encoder_pub, rear_encoder_pub;
  tf::TransformBroadcaster odom_broadcaster;

  atrv::ATRV * atrv_;

  std::string front_port;
  std::string rear_port;

  double motor_timeout;
  ros::Timer motor_timeout_timer;

  std::string frame_id;
  bool broadcast_tf;

  double linear_vel;
  double angular_vel;

  bool connected;

  gavlab_atrv_node::StampedEncoders front_encoder_msg, rear_encoder_msg;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom_msg;

  bool first_odometry;
  long lwc_, rwc_;
  float odometry_x;
  float odometry_y;
  float odometry_w;
  ros::Time last_time_;

  boost::mutex m_mutex;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "atrv_node");
  
  ATRVNode atrv_node;
  
  atrv_node.run();
  
  return 0;
}