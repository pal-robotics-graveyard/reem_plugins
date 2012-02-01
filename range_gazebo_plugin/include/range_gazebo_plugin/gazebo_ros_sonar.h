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
 * Desc: IR Controller
 * Author: Saeed Anwar
 * Date: 21 Jul 2011
 */

/**
    \brief ROS ir scan controller.
           \li Starts a ROS node if none exists.
           \li Simulates a ir range sensor and publish sensor_msgs::Range.msg over ROS.
           \li Example Usage:
  \verbatim
    <model:physical name="ir_model">
      <body:empty name="ir_body_name">
        <sensor:ir name="ir_sensor">
          <origin>0.0 0.0 0.0</origin>
          <rayCount>683</rayCount>
          <rangeCount>683</rangeCount>
          <laserCount>1</laserCount>
          <displayRays>false</displayRays>
          <minRange>0.05</minRange>
          <maxRange>10.0</maxRange>
          <updateRate>10.0</updateRate>
          <controller:gazebo_ros_ir name="ros_ir_sensor_controller" plugin="libgazebo_ros_ir.so">
            <gaussianNoise>0.005</gaussianNoise>
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>ir_scan</topicName>
            <frameName>ir_model</frameName>
            <interface:ir name="ros_ir_sensor_iface" />
          </controller:gazebo_ros_ir>
        </sensor:ir>
      </body:empty>
    </model:phyiscal>
  \endverbatim
           .
*/


#ifndef GAZEBO_ROS_SONAR_HH
#define GAZEBO_ROS_SONAR_HH

#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>
#include <gazebo/Time.hh>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <sensor_msgs/Range.h>

namespace gazebo
{

  class SonarSensor;



  /// \brief Sick LMS 200 lase controller.
  /// 
  /// This is a controller that simulates a ir array
  class GazeboRosSonar : public Controller
  {
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosSonar(Entity *parent);
  
  /// \brief Destructor
  public: virtual ~GazeboRosSonar();
  
  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);
  
  /// \brief Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();
  
  /// \brief Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();
  
  /// \brief Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();
  
  /// \brief Put IR data to the iface
  private: void PutSonarData();

  /// \brief Keep track of number of connctions
  private: int SonarConnectCount;
  private: void SonarConnect();
  private: void SonarDisconnect();
  private: int deprecatedSonarConnectCount;
  private: void DeprecatedSonarConnect();
  private: void DeprecatedSonarDisconnect();
   
  /// \brief The parent sensor
  private: SonarSensor *myParent;
	
  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;
  private: ros::Publisher deprecated_pub_;

  /// \brief ros message
  private: sensor_msgs::Range sonarMsg;
 
  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;
  private: ParamT<std::string> *deprecatedTopicNameP;
  private: std::string deprecatedTopicName;
 
  /// \brief frame transform name, should match link name
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

  /// \brief Gaussian noise
  private: ParamT<double> *gaussianNoiseP;
  private: double gaussianNoise;

  /// \brief Gaussian noise generator
  private: double GaussianKernel(double mu,double sigma);

  /// \brief Max Range
  private: ParamT<double> *maxRangeP;
  private: double maxRange;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;
 

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  private: libgazebo::SonarIface *irIface;
    
    
#ifdef USE_CBQ
  private: ros::CallbackQueue sonar_queue_;
  private: void SonarQueueThread();
  private: boost::thread callback_queue_thread_;
#endif

};

/** \} */
/// @}

}

#endif


