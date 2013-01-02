/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <reem_diffdrive_plugin/reem_diffdrive_plugin.h>
#include <boost/assign/list_of.hpp>

namespace gazebo
{

DiffDrivePlugin::DiffDrivePlugin()
{
}

DiffDrivePlugin::~DiffDrivePlugin()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();

    delete rosnode_;
    delete transform_broadcaster_;
}

// Load the controller
void DiffDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

    ROS_INFO("Loading reem diffdrive plugin");

    this->parent = _parent;
    this->world = _parent->GetWorld();

    if (!this->parent)
    {
        gzthrow("Differential_Position2d controller requires a Model as its parent");
    }

    this->robotNamespace = "";
    if (_sdf->HasElement("robotNamespace"))
            this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

    if (!_sdf->HasElement("leftJoint"))
    {
        ROS_WARN("Differential Drive plugin missing <leftJoint>, defaults to left_joint");
        this->leftJointName = "left_joint";
    }
    else
            this->leftJointName = _sdf->GetElement("leftJoint")->GetValueString();

    if (!_sdf->HasElement("rightJoint"))
    {
        ROS_WARN("Differential Drive plugin missing <rightJoint>, defaults to right_joint");
        this->rightJointName = "right_joint";
    }
    else
        this->rightJointName = _sdf->GetElement("rightJoint")->GetValueString();

    if (!_sdf->HasElement("wheelSeparation"))
    {
        ROS_WARN("Differential Drive plugin missing <wheelSeparation>, defaults to 0.34");
        this->wheelSeparation = 0.34;
    }
    else
        this->wheelSeparation = _sdf->GetElement("wheelSeparation")->GetValueDouble();

    if (!_sdf->HasElement("wheelDiameter"))
    {
        ROS_WARN("Differential Drive plugin missing <wheelDiameter>, defaults to 0.15");
        this->wheelDiameter = 0.15;
    }
    else
        this->wheelDiameter = _sdf->GetElement("wheelDiameter")->GetValueDouble();

    if (!_sdf->HasElement("torque"))
    {
        ROS_WARN("Differential Drive plugin missing <torque>, defaults to 5.0");
        this->torque = 5.0;
    }
    else
        this->torque = _sdf->GetElement("torque")->GetValueDouble();

    if (!_sdf->HasElement("cmdVelTopicName"))
    {
        ROS_WARN("Differential Drive plugin missing <cmdVelTopicName>, defaults to cmd_vel");
        this->cmdVelTopicName = "cmd_vel";
    }
    else
        this->cmdVelTopicName = _sdf->GetElement("cmdVelTopicName")->GetValueString();

    if (!_sdf->HasElement("odomTopicName"))
    {
        ROS_WARN("Differential Drive plugin missing <odomTopicName>, defaults to /base_odometry/odom");
        this->odomTopicName = "base_odometry/odom";
    }
    else
        this->odomTopicName = _sdf->GetElement("odomTopicName")->GetValueString();

    wheelSpeed[RIGHT_WHEEL] = 0;
    wheelSpeed[LEFT_WHEEL] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    joints[LEFT_WHEEL] = this->parent->GetJoint(leftJointName);
    joints[RIGHT_WHEEL] = this->parent->GetJoint(rightJointName);

    if (!joints[LEFT_WHEEL])
        gzthrow("The controller couldn't get left hinge joint");

    if (!joints[RIGHT_WHEEL])
        gzthrow("The controller couldn't get right hinge joint");

    // Initialize the ROS node and subscribe to cmd_vel
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "diff_drive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    rosnode_ = new ros::NodeHandle(this->robotNamespace);

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
            ros::SubscribeOptions::create<geometry_msgs::Twist>(this->cmdVelTopicName, 1,
                                                                boost::bind(&DiffDrivePlugin::cmdVelCallback,this, _1),ros::VoidPtr(),
                                                                &queue_);
    sub_ = rosnode_->subscribe(so);
    pub_ = rosnode_->advertise<nav_msgs::Odometry>(this->odomTopicName.c_str(), 1);

    // Initialize the controller
    // Reset odometric pose
    odomPose[0] = 0.0;
    odomPose[1] = 0.0;
    odomPose[2] = 0.0;

    odomVel[0] = 0.0;
    odomVel[1] = 0.0;
    odomVel[2] = 0.0;

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread(boost::bind(&DiffDrivePlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&DiffDrivePlugin::Update, this));

    ROS_INFO("reem diffdrive plugin load: DONE");
}

// Update the controller
void DiffDrivePlugin::Update()
{
    // TODO: Step should be in a parameter of this function
    double wd, ws;
    double d1, d2;
    double dr, da;
    double stepTime = this->world->GetPhysicsEngine()->GetStepTime();

    GetPositionCmd();

    wd = wheelDiameter;
    ws = wheelSeparation;

    // Distance travelled by front wheels
    d1 = stepTime * wd / 2 * joints[LEFT_WHEEL]->GetVelocity(0);
    d2 = stepTime * wd / 2 * joints[RIGHT_WHEEL]->GetVelocity(0);

    dr = (d1 + d2) / 2;
    da = (d1 - d2) / ws;

    // Compute odometric pose
    odomPose[0] += dr * cos(odomPose[2]);
    odomPose[1] += dr * sin(odomPose[2]);
    odomPose[2] += da;

    // Compute odometric instantaneous velocity
    odomVel[0] = dr / stepTime;
    odomVel[1] = 0.0;
    odomVel[2] = da / stepTime;

    joints[LEFT_WHEEL]->SetVelocity(0, wheelSpeed[LEFT_WHEEL] / (wheelDiameter / 2.0));
    joints[RIGHT_WHEEL]->SetVelocity(0, wheelSpeed[RIGHT_WHEEL] / (wheelDiameter / 2.0));

    joints[LEFT_WHEEL]->SetMaxForce(0, torque);
    joints[RIGHT_WHEEL]->SetMaxForce(0, torque);

    writePositionData();
    publishOdometry();
}

void DiffDrivePlugin::GetPositionCmd()
{
    lock.lock();

    double vr, va;

    vr = x_; //myIface->data->cmdVelocity.pos.x;
    va = rot_; //myIface->data->cmdVelocity.yaw;

    //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

    wheelSpeed[LEFT_WHEEL] = vr + va * wheelSeparation / 2.0;
    wheelSpeed[RIGHT_WHEEL] = vr - va * wheelSeparation / 2.0;

    lock.unlock();
}

void DiffDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    lock.lock();

    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;

    lock.unlock();
}

void DiffDrivePlugin::QueueThread()
{
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void DiffDrivePlugin::publishOdometry()
{
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, "odom");
    std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent->GetState().GetPose();

    btQuaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    btVector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                               current_time,
                                                               odom_frame,
                                                               base_footprint_frame));
    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;

    odom_.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ;

    math::Vector3 linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.linear.x = linear.x;
    odom_.twist.twist.linear.y = linear.y;
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    odom_.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    pub_.publish(odom_);
}

// Update the data in the interface
void DiffDrivePlugin::writePositionData()
{
    math::Pose orig_pose = this->parent->GetWorldPose();

    math::Pose new_pose = orig_pose;
    new_pose.pos.x = odomPose[0];
    new_pose.pos.y = odomPose[1];
    new_pose.rot.SetFromEuler(math::Vector3(0,0,odomPose[2]));

    this->parent->SetWorldPose( new_pose );
}

GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)

}
