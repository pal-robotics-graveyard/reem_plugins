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
 * Desc: Bumper Controller
 * Author: Nate Koenig mod by John Hsu
 * Date: 24 Sept 2008
 */
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sys/time.h>

#include <gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/plugins/ContactPlugin.hh>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <reem_msgs/Bumper.h>

namespace gazebo
{
  /// \brief A Bumper controller
  class ReemGazeboBumper : public gazebo::SensorPlugin
  {
    /// Constructor
    public: ReemGazeboBumper();

    /// Destructor
    public: ~ReemGazeboBumper();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// Update the controller
    protected: virtual void Update();

  private:

      /// \brief pointer to ros node
      ros::NodeHandle* rosnode_;
      ros::Publisher contact_pub_;

      /// \brief set topic name of broadcast
      std::string bumper_topic_name_;
      std::string frame_name_;

      gazebo::sensors::ContactSensorPtr sensor_;
      gazebo::physics::WorldPtr parent_;

      /// \brief broadcast some string for now.
      gazebo_msgs::ContactsState contact_state_msg_;

      /// \brief for setting ROS name space
      std::string robot_namespace_;

      /// \brief Keep track of number of connctions
      int contact_connect_count_;
      void ContactConnect();
      void ContactDisconnect();
      ros::CallbackQueue contact_queue_;
      void ContactQueueThread();
      boost::thread callback_queue_thread_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

  };

}
