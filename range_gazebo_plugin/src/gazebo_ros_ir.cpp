/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: IR controller.
 * Author: Saeed Anwar
 * Date: 22 Jul 2011
 */

#include <algorithm>
#include <assert.h>

#include <range_gazebo_plugin/gazebo_ros_ir.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/IRSensor.hh>
#include <gazebo/gz.h>

namespace gazebo
{

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_ir", GazeboRosIr);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIr::GazeboRosIr(Entity *parent)
    : Controller(parent)

{
  this->myParent = dynamic_cast<IRSensor*>(this->parent);
  if (!this->myParent)
    gzthrow("GazeboRosIr controller requires a IRSensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise", 0.0, 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->deprecatedTopicNameP = new ParamT<std::string>("deprecatedTopicName", "", 0);
  this->frameNameP = new ParamT<std::string>("frameName", "default_gazebo_ros_ir_frame", 0);
  this->maxRangeP = new ParamT<double>("maxRange", 1.45, 0);
  this->minRangeP = new ParamT<double>("minRange", 0.1, 0);
  this->fovP = new ParamT<double>("fov", 0.4, 0);
 
  Param::End();
  this->irConnectCount = 0;
  this->deprecatedIrConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIr::~GazeboRosIr()
{
  delete this->robotNamespaceP;
  delete this->gaussianNoiseP;
  delete this->topicNameP;
  delete this->maxRangeP;
  delete this->minRangeP;
  delete this->fovP;
  delete this->deprecatedTopicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIr::LoadChild(XMLConfigNode *node)
{
    this->irIface = dynamic_cast<libgazebo::IRIface*>(this->GetIface("irarray"));

  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  ROS_INFO("INFO: gazebo_ros_ir plugin loading" );

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  
  this->deprecatedTopicNameP->Load(node);
  this->deprecatedTopicName = this->deprecatedTopicNameP->GetValue();
  
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();
  
  this->gaussianNoiseP->Load(node);
  this->gaussianNoise = this->gaussianNoiseP->GetValue();

  this->maxRangeP->Load(node);
  this->maxRange = this->maxRangeP->GetValue();

  this->minRangeP->Load(node);
  this->minRange = this->minRangeP->GetValue();

  this->fovP->Load(node);
  this->fov = this->fovP->GetValue();
  
  if (this->topicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Range>(
      this->topicName,1,
      boost::bind( &GazeboRosIr::IrConnect,this),
      boost::bind( &GazeboRosIr::IrDisconnect,this), ros::VoidPtr(), &this->ir_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
#else
    this->pub_ = this->rosnode_->advertise<sensor_msgs::Range>(this->topicName,1,
      boost::bind( &GazeboRosIr::IrConnect, this),
      boost::bind( &GazeboRosIr::IrDisconnect, this));
#endif
  }

  if (this->deprecatedTopicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Range>(
      this->deprecatedTopicName,1,
      boost::bind( &GazeboRosIr::DeprecatedIrConnect,this),
      boost::bind( &GazeboRosIr::DeprecatedIrDisconnect,this), ros::VoidPtr(), &this->ir_queue_);
    this->deprecated_pub_ = this->rosnode_->advertise(ao);
#else
    this->deprecated_pub_ = this->rosnode_->advertise<sensor_msgs::Range>(this->deprecatedTopicName,1,
      boost::bind( &GazeboRosIr::DeprecatedIrConnect, this),
      boost::bind( &GazeboRosIr::DeprecatedIrDisconnect, this));
#endif
  }
}
/////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosIr::InitChild()
{
  // sensor generation off by default
  this->myParent->SetActive(false);
#ifdef USE_CBQ
  // start custom queue for IR
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIr::IrQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosIr::IrConnect()
{
  this->irConnectCount++;
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosIr::IrDisconnect()
{
  this->irConnectCount--;

  if (this->irConnectCount == 0 && this->deprecatedIrConnectCount == 0)
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosIr::DeprecatedIrConnect()
{
  ROS_WARN("you are subscribing to a deprecated ROS topic %s, please change your code/launch script to use new ROS topic %s",
           this->deprecatedTopicName.c_str(), this->topicName.c_str());
  this->deprecatedIrConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosIr::DeprecatedIrDisconnect()
{
  this->deprecatedIrConnectCount--;

  if (this->irConnectCount == 0 && this->deprecatedIrConnectCount == 0)
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosIr::FiniChild()
{
  this->rosnode_->shutdown();
  sleep(1);
#ifdef USE_CBQ
  this->callback_queue_thread_.join();
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosIr::GaussianKernel(double mu,double sigma)
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

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put IR data to the interface
void GazeboRosIr::IrQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->ir_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIr::UpdateChild()
{
  // as long as ros is connected, parent is active
  //ROS_ERROR("debug IR count %d",this->irConnectCount);
  if (!this->myParent->IsActive())
  {
    // do this first so there's chance for sensor to run 1 frame after activate
    if ((this->irConnectCount > 0 && this->topicName != "") ||
        (this->deprecatedIrConnectCount > 0 && this->deprecatedTopicName != ""))
      this->myParent->SetActive(true);
  }
  else
  {
    this->PutIRData();
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Put IR data to the interface
void GazeboRosIr::PutIRData()
{
  //double maxRange = 0.8; //this->myParent->maxRange; // this function is also not present in
  //double minRange = 0.1; //this->myParent->minRange;
  //double fieldofview = 0.4; //2*(this->myParent->maxAngle);
  //std::cout << maxRange << std::endl;
  
  int irCount = this->myParent->GetIRCount();
  //ROS_INFO("Nunber of IR: %i", irCount);
  
  double Range;
  float ranges[irCount];
  
  for (int i=0; i<irCount; i++)
  {
      ranges[i]=this->myParent->GetRange(i);
            
  }
  Range = ranges[0];
  for(int i = 0; i < irCount; i++)//loop through array
  {
	if(ranges[i] < Range) { Range = ranges[i]; } //find min
  }


  this->lock.lock();
  // Add Frame Name
  this->irMsg.header.frame_id = this->frameName;
  this->irMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  this->irMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  if (this->maxRange > 1.5)
	this->irMsg.radiation_type = sensor_msgs::Range::INFRARED;
  else
        this->irMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;

  this->irMsg.field_of_view = this->fov;
  this->irMsg.min_range = this->minRange;
  this->irMsg.max_range = this->maxRange;
  this->irMsg.range = std::min(Range + this->GaussianKernel(0,this->gaussianNoise), maxRange);

  // send data out via ros message
  if (this->irConnectCount > 0 && this->topicName != "")
      this->pub_.publish(this->irMsg);

  if (this->deprecatedIrConnectCount > 0 && this->deprecatedTopicName != "")
      this->deprecated_pub_.publish(this->irMsg);

  this->lock.unlock();

}

}
