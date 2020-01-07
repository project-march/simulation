#include <functional>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cmath>
#include <march_shared_resources/Subgait.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <Subgait.pb.h>
#include <ros/ros.h>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
    typedef const boost::shared_ptr<
            const gazebo::msgs::Int>
            IntPtr;
    
    class ComController : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            this->model = _parent;

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->Name());

            // Subscribe to the topic, and register a callback
            this->sub = this->node->Subscribe("/march/random_topic", &ComController::OnPublish, this);
            std::cout << "Nothing special here" << '\n';
//            std::cout << this->sub << '\n';

            // Initialize ros, if it has not already bee initialized.
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
                    ros::SubscribeOptions::create<std_msgs::Float32>(
                            "/march/random_topic2",
                            1,
                            boost::bind(&ComController::OnRosMsg, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                    std::thread(std::bind(&ComController::QueueThread, this));
        }

        // Called by the world update start event
        public: void OnPublish(IntPtr &_msg)
        {
            std::cout << "Its a message!" << '\n';
            std::cout << "Its amazing!!" << '\n';
            std::cout << _msg->data() << '\n';
            /*std::cout << _msg.goal.current_subgait.name << '\n';
            this->time _since_start = this->model->GetWorld()->SimTime().Double();
            this->subgait_name = _msg.goal.current_subgait.name;*/
        }

        public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
        {
            std::cout << "Its a message!" << '\n';
            std::cout << "Its even more amazing!!" << '\n';
            std::cout << _msg->data << '\n';
        }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private:physics::ModelPtr model;

        /// \brief A node used for transport
        private: transport::NodePtr node;

        /// \brief A subscriber to a named topic.
        private: transport::SubscriberPtr sub;


        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ComController)
}
