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

#include <range_gazebo_plugin/gazebo_ros_sonar.h>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/physics.h>

using namespace gazebo;

GazeboRosSonar::GazeboRosSonar()
{
}

GazeboRosSonar::~GazeboRosSonar()
{
    sensor_->SetActive(false);
    event::Events::DisconnectWorldUpdateStart(updateConnection);
    node_handle_->shutdown();
    delete node_handle_;
}

void GazeboRosSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_DEBUG("Starting range gazebo plugin");

    // Get then name of the parent sensor
    sensor_ = boost::shared_dynamic_cast<sensors::RaySensor>(_parent);
    if (!sensor_)
    {
        gzthrow("GazeboRosSonar requires a Ray Sensor as its parent");
        return;
    }

    // Get the world name.
    std::string worldName = sensor_->GetWorldName();
    parent_ = gazebo::physics::get_world(worldName);

    this->namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
        this->namespace_ = _sdf->GetElement("robotNamespace")->GetValueString();

    if (!_sdf->HasElement("topicName"))
    {
        ROS_WARN("Range plugin missing <topicName>, defaults to sonar");
        this->topic_name_ = "sonar";
    }
    else
        this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

    if (!_sdf->HasElement("frameId"))
    {
        ROS_WARN("Range plugin missing <frameId>, defaults to sonar_base_link");
        this->frame_id_ = "sonar_base_link";
    }
    else
        this->frame_id_ = _sdf->GetElement("frameId")->GetValueString();

    if (!_sdf->HasElement("radiation"))
    {
        ROS_WARN("Range plugin missing <radiation>, defaults to ultrasound");
        this->radiation_ = "ultrasound";

    }
    else
        this->radiation_ = _sdf->GetElement("radiation")->GetValueString();

    if (!_sdf->HasElement("fov"))
    {
        ROS_WARN("Range plugin missing <fov>, defaults to 0.05");
        this->fov_ = 0.05;
    }
    else
        this->fov_ = _sdf->GetElement("fov")->GetValueDouble();

    if (!_sdf->HasElement("gaussianNoise"))
    {
        ROS_WARN("Range plugin missing <gaussianNoise>, defaults to 0.0");
        this->gaussian_noise_ = 0.0;
    }
    else
        this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->GetValueDouble();

    ROS_DEBUG("Loaded with values:   robotNamespace = %s, topicName = %s, frameId = %s, radiation = %s, fov = %f, noise = %f",
             this->namespace_.c_str(), this->topic_name_.c_str(),this->frame_id_.c_str(),this->radiation_.c_str(),
             this->fov_, this->gaussian_noise_);

    node_handle_ = new ros::NodeHandle(this->namespace_);

    range_.header.frame_id = frame_id_;
    if (radiation_==std::string("ultrasound"))
        range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
    else
        range_.radiation_type = sensor_msgs::Range::INFRARED;

    range_.field_of_view = fov_;
    range_.max_range = sensor_->GetRangeMax();
    range_.min_range = sensor_->GetRangeMin();


    node_handle_ = new ros::NodeHandle(namespace_);
    publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_name_, 1);

    updateConnection = sensor_->GetLaserShape()->ConnectNewLaserScans(boost::bind(&GazeboRosSonar::Update, this));

    // activate RaySensor
    sensor_->SetActive(true);

    ROS_DEBUG("%s is active!",frame_id_.c_str());
}


////////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosSonar::GaussianKernel(double mu,double sigma)
{
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables
    double U = (double)rand()/(double)RAND_MAX;             // normalized uniform random variable
    double V = (double)rand()/(double)RAND_MAX;             // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);

    // we'll just use X scale to our mu and sigma
    X = sigma * X + mu;
    return X;
}

void GazeboRosSonar::Update()
{
    //Activate sensor
    if (!sensor_->IsActive())
        sensor_->SetActive(true);

    range_.header.stamp.sec  = (parent_->GetSimTime()).sec;
    range_.header.stamp.nsec = (parent_->GetSimTime()).nsec;

    // find ray with minimal range
    range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();

    int num_ranges = sensor_->GetLaserShape()->GetSampleCount() * sensor_->GetLaserShape()->GetVerticalSampleCount();

    for(int i = 0; i < num_ranges; ++i) {
        double ray = sensor_->GetLaserShape()->GetRange(i);
        if (ray < range_.range) range_.range = ray;
    }

    // add Gaussian noise and limit to min/max range
    if (range_.range < range_.max_range)
        range_.range = std::min(range_.range + this->GaussianKernel(0,gaussian_noise_), sensor_->GetRangeMax());

    publisher_.publish(range_);

}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)

