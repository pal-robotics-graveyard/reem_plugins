/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <limits>
#include <range_gazebo_plugin/gazebo_ros_sonar.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_sonar", GazeboRosSonar)

GazeboRosSonar::GazeboRosSonar(Entity *parent) : Controller(parent)
{
  sensor_ = dynamic_cast<RaySensor*>(parent);
  if (!sensor_) gzthrow("GazeboRosSonar controller requires a RaySensor as its parent");

  Param::Begin(&parameters);
  topic_param_ = new ParamT<std::string>("topicName", "sonar", false);
  frame_id_param_ = new ParamT<std::string>("frameId", "", false);
  radiation_param_ = new ParamT<std::string>("radiation","ultrasound",false);
  fov_param_ = new ParamT<double>("fov", 0.05, false);
  gaussian_noise_ = new ParamT<double>("gaussianNoise", 0.0, 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar::~GazeboRosSonar()
{
  delete topic_param_;
  delete frame_id_param_;
  delete radiation_param_;
  delete fov_param_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::LoadChild(XMLConfigNode *node)
{
  ROS_INFO("INFO: gazebo_ros_ir plugin loading" );
  topic_param_->Load(node);
  frame_id_param_->Load(node);
  radiation_param_->Load(node);
  fov_param_->Load(node);
  gaussian_noise_->Load(node);
}

///////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosSonar::InitChild()
{
  Range.header.frame_id = **frame_id_param_;

  if (**radiation_param_==std::string("ultrasound"))
      Range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  else
      Range.radiation_type = sensor_msgs::Range::INFRARED;

  Range.field_of_view = **fov_param_;
  Range.max_range = sensor_->GetMaxRange();
  Range.min_range = sensor_->GetMinRange();

  sensor_->SetActive(false);
  node_handle_ = new ros::NodeHandle("");
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(**topic_param_, 10);
}

////////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosSonar::GaussianKernel(double mu,double sigma)
{
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables
    // see wikipedia
    double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
    double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
    //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
    // we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSonar::UpdateChild()
{
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  Range.header.stamp.sec  = (Simulator::Instance()->GetSimTime()).sec;
  Range.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;
  Range.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();

  for(int i = 0; i < sensor_->GetRangeCount(); ++i) {
    double ray = sensor_->GetRange(i);
    if (ray < Range.range) Range.range = ray;
  }

  Range.range = std::min(Range.range + this->GaussianKernel(0,**gaussian_noise_), sensor_->GetMaxRange());
  publisher_.publish(Range);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosSonar::FiniChild()
{
  sensor_->SetActive(false);
  node_handle_->shutdown();
  delete node_handle_;
}

