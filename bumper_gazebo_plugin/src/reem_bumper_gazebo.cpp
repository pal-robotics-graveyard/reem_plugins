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

#include <reem_bumper_gazebo/reem_bumper_gazebo.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ContactSensor.hh>
#include <gazebo/World.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>
#include <gazebo/Geom.hh>

#include <reem_msgs/Bumper.h>


namespace gazebo
{

GZ_REGISTER_DYNAMIC_CONTROLLER("reem_bumper_gazebo", My_Generic_Bumper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
My_Generic_Bumper::My_Generic_Bumper(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<gazebo::ContactSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Bumper controller requires a Contact Sensor as its parent");

  Param::Begin(&this->parameters);
  this->bumperTopicNameP = new ParamT<std::string>("bumperTopicName", "", 1);
  this->frameNameP = new ParamT<std::string>("frameName", "world", 0);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->frameIdP = new ParamT<std::string>("frameId", "/", 0);
  Param::End();

  this->contactConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
My_Generic_Bumper::~My_Generic_Bumper()
{
  delete this->robotNamespaceP;
  delete this->frameNameP;
  delete this->bumperTopicNameP;
  delete this->rosnode_;
  delete this->frameIdP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void My_Generic_Bumper::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  // "publishing contact/collisions to this topic name: " << this->bumperTopicName << std::endl;
  this->bumperTopicNameP->Load(node);
  this->bumperTopicName = this->bumperTopicNameP->GetValue();
  ROS_DEBUG("publishing contact/collisions to topic name: %s",
           this->bumperTopicName.c_str());

  // "transform contact/collisions pose, forces to this body (link) name: " << this->frameName << std::endl;
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  this->frameIdP->Load(node);
  this->frameId = this->frameIdP->GetValue();


#ifdef USE_CBQ
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<reem_msgs::Bumper>(this->bumperTopicName+std::string("/state"),1,boost::bind( &My_Generic_Bumper::ContactConnect,this),
                                                                           boost::bind( &My_Generic_Bumper::ContactDisconnect,this), ros::VoidPtr(), &this->contact_queue_);
  this->contact_pub_ = this->rosnode_->advertise(ao);
#else
  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(this->bumperTopicName+std::string("/state"),1);
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void My_Generic_Bumper::ContactConnect()
{
  this->contactConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void My_Generic_Bumper::ContactDisconnect()
{
  this->contactConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void My_Generic_Bumper::InitChild()
{
  // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
  // since most bodies are constructed on the fly
  this->myFrame = NULL;
#ifdef USE_CBQ
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread( boost::bind( &My_Generic_Bumper::ContactQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void My_Generic_Bumper::UpdateChild()
{
  /// if frameName specified is "world", "/map" or "map" report back inertial values in the gazebo world
  if (this->myFrame == NULL && this->frameName != "world" && this->frameName != "/map" && this->frameName != "map")
  {
    // lock in case a model is being spawned
    boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
    // look through all models in the world, search for body name that matches frameName
    std::vector<Model*> all_models = World::Instance()->GetModels();
    for (std::vector<Model*>::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
    {
      if (*iter) this->myFrame = dynamic_cast<Body*>((*iter)->GetBody(this->frameName));
      if (this->myFrame) break;
    }

    // not found
    if (this->myFrame == NULL)
    {
      ROS_DEBUG("gazebo_ros_bumper plugin: frameName: %s does not exist yet, will not publish\n",this->frameName.c_str());
      return;
    }
  }

  boost::mutex::scoped_lock sclock(this->lock);

  reem_msgs::Bumper bumperMsg;

  bool in_contact;
  unsigned int num_contact_count = 0;
  int l = 0;
  std::string geom2_name;
  Vector3 body1ContactForce, body2ContactForce;
  Geom *geom1;

  // Count the number of geom-geom contacts
  for (unsigned int i=0; i < this->myParent->GetGeomCount(); i++)
    num_contact_count += this->myParent->GetGeomContactCount(i) > 0;

  // populate header
  //if (cur_body)
  //  this->contactMsg.header.frame_id = cur_body->GetName();  // @todo: transform results to the link name

  Time cur_time = Simulator::Instance()->GetSimTime();
  this->contactsStateMsg.header.frame_id = frameName;  // information are in inertial coordinates
  this->contactsStateMsg.header.stamp.sec = cur_time.sec;
  this->contactsStateMsg.header.stamp.nsec = cur_time.nsec;

  bumperMsg.header.frame_id = this->frameId;
  bumperMsg.header.stamp = ros::Time::now();
  bumperMsg.header.seq = 0;

  // set contact states size
  this->contactsStateMsg.states.clear();  // one contact_count per pair of geoms in contact (up to 64 contact points per pair of geoms)

  // get contact array sizes for this contact param.
  // each contact param has num_contact_count contact pairs,
  // each contact geom pair has up to 64 contact points
  //std::vector<Geom*> contact_geoms;
  int total_contact_points = 0;

  // get reference frame (body(link)) pose and subtract from it to get relative
  //   force, torque, position and normal vectors
  Pose3d pose, frame_pose;
  Quatern rot, frame_rot;
  Vector3 pos, frame_pos;
  if (this->myFrame)
  {
    frame_pose = this->myFrame->GetWorldPose(); // - this->myBody->GetCoMPose();
    frame_pos = frame_pose.pos;
    frame_rot = frame_pose.rot;
  }
  else
  {
    // no specific frames specified, use identity pose, keeping relative frame at inertial origin
    frame_pos = Vector3(0,0,0);
    frame_rot = Quatern(1,0,0,0); // gazebo u,x,y,z == identity
    frame_pose = Pose3d(frame_pos,frame_rot);
  }

  // For each geom that the sensor is monitoring
  for (unsigned int i=0; i < this->myParent->GetGeomCount(); i++)
  {
    in_contact = this->myParent->GetGeomContactCount(i) > 0;

    if (!in_contact)
      continue;

    // get pointer to the Geom
    geom1 = this->myParent->GetGeom(i);

    // Number of contacts for this geom ( this is the number of geom-geom
    // contacts)
    unsigned int contactCount = geom1->GetContactCount();

    total_contact_points += contactCount;

    // For each geom-geom contact
    for (unsigned int j=0; j < contactCount; j++)
    {
      Contact contact = geom1->GetContact(j);
      unsigned int pts = contact.positions.size();

      std::ostringstream stream;
      stream    << "touched!    i:" << l
                << "      my geom:" << contact.geom1->GetName()
                << "   other geom:" << contact.geom2->GetName()
                << "         time:" << contact.time
                << std::endl;

      gazebo_msgs::ContactState state;
      state.info = stream.str();
      state.geom1_name = contact.geom1->GetName();
      state.geom2_name = contact.geom2->GetName();

      state.wrenches.clear();
      state.contact_positions.clear();
      state.contact_normals.clear();
      state.depths.clear();

      // sum up all wrenches for each DOF
      geometry_msgs::Wrench total_wrench;
      total_wrench.force.x = 0;
      total_wrench.force.y = 0;
      total_wrench.force.z = 0;
      total_wrench.torque.x = 0;
      total_wrench.torque.y = 0;
      total_wrench.torque.z = 0;

      for (unsigned int k=0; k < pts; k++)
      {
        // rotate into user specified frame. frame_rot is identity if world is used.
        Vector3 force = frame_rot.RotateVectorReverse(Vector3(contact.forces[k].body1Force.x,
                                                       contact.forces[k].body1Force.y,
                                                       contact.forces[k].body1Force.z));
        Vector3 torque = frame_rot.RotateVectorReverse(Vector3(contact.forces[k].body1Torque.x,
                                                        contact.forces[k].body1Torque.y,
                                                        contact.forces[k].body1Torque.z));

        // set wrenches
        geometry_msgs::Wrench wrench;
        wrench.force.x  = force.x;
        wrench.force.y  = force.y;
        wrench.force.z  = force.z;
        wrench.torque.x = torque.x;
        wrench.torque.y = torque.y;
        wrench.torque.z = torque.z;
        state.wrenches.push_back(wrench);
        total_wrench.force.x  += wrench.force.x ;
        total_wrench.force.y  += wrench.force.y ;
        total_wrench.force.z  += wrench.force.z ;
        total_wrench.torque.x += wrench.torque.x;
        total_wrench.torque.y += wrench.torque.y;
        total_wrench.torque.z += wrench.torque.z;

        // transform contact positions into relative frame
        // set contact positions
        gazebo::Vector3 contact_position;
        contact_position = contact.positions[k] - frame_pos;
        contact_position = frame_rot.RotateVectorReverse(contact_position);
        geometry_msgs::Vector3 tmp;
        tmp.x = contact_position.x;
        tmp.y = contact_position.y;
        tmp.z = contact_position.z;
        state.contact_positions.push_back(tmp);

        // rotate normal into user specified frame. frame_rot is identity if world is used.
        Vector3 normal = frame_rot.RotateVectorReverse(Vector3(contact.normals[k].x,
                                                               contact.normals[k].y,
                                                               contact.normals[k].z));
        // set contact normals
        geometry_msgs::Vector3 contact_normal;
        contact_normal.x = normal.x;
        contact_normal.y = normal.y;
        contact_normal.z = normal.z;
        state.contact_normals.push_back(contact_normal);

        // set contact depth, interpenetration
        state.depths.push_back(contact.depths[k]);
      }
      state.total_wrench = total_wrench;
      this->contactsStateMsg.states.push_back(state);
      if (state.contact_positions.empty())
      {
          bumperMsg.is_pressed = false;
          boolMsg.data=false;
      }
      else
      {
          boolMsg.data=true;
          bumperMsg.is_pressed = true;
      }
    }
  }

 // this->contact_pub_.publish(boolMsg);

  //boolMsg.data=false;

   this->contact_pub_.publish(bumperMsg);

  bumperMsg.is_pressed = false;

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void My_Generic_Bumper::FiniChild()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void My_Generic_Bumper::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

}
