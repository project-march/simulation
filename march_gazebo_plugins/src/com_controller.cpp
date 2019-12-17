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

namespace gazebo
{
    typedef const boost::shared_ptr<
            const march_gazebo_messages::msgs::GaitGoal>
            GaitGoalPtr;
    
    class ComController : public ModelPlugin
    {
        public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            // Store the pointer to the model
            this->model = _parent;
            this->leg_leg_distance = 0.5;
            this->swing_duration = 1.2;
            this->swing_step_size = 0.8;
            this->open_duration = 1.7;
            this->open_step_size = 0.4;

            this->subgait_name = "standing";
            this->p_x = 400;
            this->d_x = 10;
            this->p_y = 400;
            this->d_y = 10;
            this->time_since_start = 0;
            this->error_x_last_timestep = 0;
            this->error_y_last_timestep = 0;

            // Apply a small linear velocity to the model.
            for (auto const& i : this->model->GetLinks()) {
                std::cout << i->GetName() << "\n";
            }

            this->foot_left = this->model->GetLink("ankle_plate_left");
            this->foot_right = this->model->GetLink("ankle_plate_right");

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ComController::OnUpdate, this));

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init();

            // Subscribe to the topic, and register a callback
            this->sub = this->node->Subscribe("/march/gait/schedule/goal", &ComController::OnPublish, this);
            std::cout << "Nothing special here" << '\n';
            std::cout << this->sub << '\n';
        }

        // Called by the world update start event
        void OnPublish(GaitGoalPtr &_msg)
        {
            std::cout << "Its a message!" << '\n';
            std::cout << "Its amazing!!" << '\n';
            std::cout << _msg->goal().current_subgait().name() << '\n';
            /*std::cout << _msg.goal.current_subgait.name << '\n';
            this->time _since_start = this->model->GetWorld()->SimTime().Double();
            this->subgait_name = _msg.goal.current_subgait.name;*/
        }

        // Called by the world update start event
        ignition::math::v4::Vector3<double> GetCom() {
            ignition::math::v4::Pose3<double> com(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            for (auto const& link : this->model->GetLinks()) {
                com += link->WorldCoGPose();
            }
            return com.Pos() / 8.0;
        }

            // Called by the world update start event
        void OnUpdate()
            {
                std::cout << "print" << '\n';
                std::cout << this->sub << '\n';
                double time = this->model->GetWorld()->SimTime().Double();
                double goal_position_x;
                double goal_position_y;

                auto model_com = this->GetCom();
                auto foot_left_pose = this->foot_left->WorldCoGPose().Pos();
                auto foot_right_pose = this->foot_right->WorldCoGPose().Pos();

//                std::cout << model_com.Z() << '\n';

                if (subgait_name == "standing"){
                    goal_position_x = foot_left_pose.X();
                    goal_position_y = foot_left_pose.Y();
                }
                else if (subgait_name == "right_open") {
                    goal_position_x = foot_left_pose.X() + 0.5 * (foot_right_pose.X() - foot_left_pose.X()) *
                                      (1 - std::cos(3.14*(time - time_since_start)/open_duration));
                    goal_position_y = foot_left_pose.Y() + 0.5 * (time - time_since_start) *
                                      open_step_size / open_duration;
                }
                else if (subgait_name == "right_swing") {
                    goal_position_x = foot_left_pose.X() + 0.5 * (foot_right_pose.X() - foot_left_pose.X()) *
                                      (1 - std::cos(3.14*(time - time_since_start)/swing_duration));
                    goal_position_y = foot_left_pose.Y() - 0.25 * swing_step_size +
                                      0.5 * (time - time_since_start) * swing_step_size / swing_duration;
                }
                else if (subgait_name == "left_swing") {
                    goal_position_x = foot_right_pose.X() + 0.5 * (foot_left_pose.X() - foot_right_pose.X()) *
                                      (1 - std::cos(3.14*(time - time_since_start)/swing_duration));
                    goal_position_y = foot_right_pose.Y() - 0.25 * swing_step_size +
                                      0.5 * (time - time_since_start) * swing_step_size / swing_duration;
                }
                else {
                    std::cout << subgait_name << '\n';
                    goal_position_x = model_com.X();
                    goal_position_y = model_com.Y();
                }

                double error_x = model_com.X() - goal_position_x;
                double error_y = model_com.Y() - goal_position_y;

                double F_x = -this->p_x * error_x - this->d_x * (error_x - this->error_x_last_timestep);
                double F_y = -this->p_y * error_y - this->d_y * (error_y - this->error_y_last_timestep);
//                std::cout << F_x << "  " << F_y << "\n";
                const ignition::math::v4::Vector3<double> vec(F_x, F_y, 0.0);
//                this->model->GetLink("base_link")->AddForce(vec);
            }

            // Pointer to the model
        private:
        physics::ModelPtr model;
        physics::LinkPtr foot_left;
        physics::LinkPtr foot_right;
        double time_since_start;
        double swing_duration;
        double swing_step_size;
        double open_duration;
        double open_step_size;
        double leg_leg_distance;
        double p_x;
        double d_x;
        double p_y;
        double d_y;
        double error_x_last_timestep;
        double error_y_last_timestep;
        std::string subgait_name;
        transport::NodePtr node;
        transport::SubscriberPtr sub;

            // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ComController)
}
