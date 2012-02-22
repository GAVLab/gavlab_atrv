/*                                                                                                                       
 * Copyright (c) 2010, Willow Garage, Inc.                                                                               
 * All rights reserved.                                                                                                  
 *                                                                                                                       
 * Redistribution and use in source and binary forms, with or without                                                    
 * modification, are permitted provided that the following conditions are met:                                           
 *                                                                                                                       
 *     * Redistributions of source code must retain the above copyright                                                  
 *       notice, this list of conditions and the following disclaimer.                                                   
 *     * Redistributions in binary form must reproduce the above copyright                                               
 *       notice, this list of conditions and the following disclaimer in the                                             
 *       documentation and/or other materials provided with the distribution.                                            
 *     * Neither the name of the Willow Garage, Inc. nor the names of its                                                
 *       contributors may be used to endorse or promote products derived from                                            
 *       this software without specific prior written permission.                                                        
 *                                                                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"                                           
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE                                             
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE                                            
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE                                              
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR                                                   
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF                                                  
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS                                              
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN                                               
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)                                               
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE                                            
 * POSSIBILITY OF SUCH DAMAGE.                                                                                           
 */

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

namespace throttle_pointcloud
{
typedef sensor_msgs::PointCloud2 PointCloud;
using sensor_msgs::Image;

class CloudThrottle : public nodelet::Nodelet
{
public:
  //Constructor
  CloudThrottle(): divisor_(1), count_(0), img_count_(0)
  {
  };

private:
  int divisor_;
  int count_, img_count_;
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("divisor", divisor_);
    if (divisor_ <= 0) {
      NODELET_ERROR("divisor must be positive.");
      exit(-1);
    }

    pub_ = nh.advertise<PointCloud>("cloud_out", 10);
    // img_pub_ = nh.advertise<Image>("camera_in", 10);
    sub_ = nh.subscribe<PointCloud>("cloud_in", 10, &CloudThrottle::callback, this);
    // img_sub_ = nh.subscribe<Image>("camera_out", 10, &CloudThrottle::img_callback, this);
  };

  void callback(const PointCloud::ConstPtr& cloud)
  {
    // If count has reached the divisor_, reset it
    if (count_ == divisor_ - 1) {
      count_ = 0;
      return;
    }

    // If count_ == 0 publish the cloud
    if (count_ == 0) {
      pub_.publish(cloud);
    }

    count_ += 1;
  }

/*
  void img_callback(const Image::ConstPtr& img)
  {
    NODELET_INFO("Here!");
    // If count has reached the divisor_, reset it
    if (img_count_ == divisor_*2.0 - 1) {
      img_count_ = 0;
      return;
    }

    // If count_ == 0 publish the cloud
    if (img_count_ == 0) {
      img_pub_.publish(img);
    }

    img_count_ += 1;
  }
*/

  ros::Publisher pub_, img_pub_;
  ros::Subscriber sub_, img_sub_;
  
};


PLUGINLIB_DECLARE_CLASS(throttle_pointcloud, CloudThrottle, throttle_pointcloud::CloudThrottle, nodelet::Nodelet);
}
