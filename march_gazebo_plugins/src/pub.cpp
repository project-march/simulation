#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the  velodyne topic
    gazebo::transport::PublisherPtr pub =
            node->Advertise<gazebo::msgs::Int>("/march/random_topic");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Create a a vector3 message
    gazebo::msgs::Int msg;
    msg.set_data(std::stoi(_argv[1]));

//    // Set the velocity in the x-component
//    gazebo::msgs::Set(&msg, ignition::math::Int(std::stoi(_argv[1])));

    // Send the message
    pub->Publish(msg);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
