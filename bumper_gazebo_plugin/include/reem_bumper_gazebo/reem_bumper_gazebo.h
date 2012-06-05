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

#ifndef GAZEBO_ROS_BUMPER_CONTROLLER_HH
#define GAZEBO_ROS_BUMPER_CONTROLLER_HH

#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <sys/time.h>
#include <std_msgs/Bool.h>

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Param.hh>

// ros messages
#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <std_msgs/String.h>

#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>

namespace gazebo
{
  class ContactSensor;


  /// \brief A Bumper controller
  class My_Generic_Bumper : public Controller
  {
    /// Constructor
      public: My_Generic_Bumper(Entity *parent );

    /// Destructor
      public: virtual ~My_Generic_Bumper();

    /// Load the controller
    /// \param node XML config node
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// Init the controller
    protected: virtual void InitChild();

    /// Update the controller
    protected: virtual void UpdateChild();

    /// Finalize the controller
    protected: virtual void FiniChild();

    /// The parent Model
    private: ContactSensor *myParent;

    /// \brief Return information in this body (link) coordinate
    ///        if unavailable, use "/map" and global gazebo world frame
    private: Body *myFrame;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher contact_pub_;

    /// \brief set topic name of broadcast
    private: ParamT<std::string> *bumperTopicNameP;
    private: std::string bumperTopicName;

    private: ParamT<std::string> *frameNameP;
    private: std::string frameName;

    private: ParamT<std::string> *frameIdP;
    private: std::string frameId;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock;

    /// \brief broadcast some string for now.
    private: gazebo_msgs::ContactsState contactsStateMsg;
    private: std_msgs::Bool boolMsg;

    /// \brief for setting ROS name space
    private: ParamT<std::string> *robotNamespaceP;
    private: std::string robotNamespace;

    /// \brief Keep track of number of connctions
    private: int contactConnectCount;
    private: void ContactConnect();
    private: void ContactDisconnect();

#ifdef USE_CBQ
    private: ros::CallbackQueue contact_queue_;
    private: void ContactQueueThread();
    private: boost::thread callback_queue_thread_;
#endif

  };

  /** \} */
  /// \}

}

#endif
