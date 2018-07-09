#ifndef _STEP_PLUGIN_HH_
#define _STEP_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int32.h"

namespace gazebo
{
  class StepPlugin : public WorldPlugin
  {
    /// Constructor
    public: StepPlugin() {}

    /// The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_parent->GetName());
      this->worldPtr = _parent;

      // Create a topic name
      std::string topicName = "step_world";

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Int32>(
      topicName,
      1,
      boost::bind(&StepPlugin::Step, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
      std::thread(std::bind(&StepPlugin::QueueThread, this));

      worldPtr->SetPaused(true);
    }

    /// Handle an incoming message from ROS
    public: void Step(const std_msgs::Int32ConstPtr &_msg)
    {
      int steps = _msg->data;
      worldPtr->Step(steps);
    }

    /// ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// A node used for transport
    private: transport::NodePtr node;

    /// A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// Pointer to the model.
    private: physics::WorldPtr worldPtr;

    /// A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// A ROS subscriber
    private: ros::Subscriber rosSub;

    /// A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_WORLD_PLUGIN(StepPlugin)
}
#endif
