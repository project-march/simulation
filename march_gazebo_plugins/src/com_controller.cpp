#include <functional>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cmath>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <march_shared_resources/GaitActionGoal.h>

namespace gazebo
{

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

            // Store the pointer to the model
            this->model = _parent;
            this->leg_leg_distance = 0.5;
            this->swing_step_size = 0.8;
            this->subgait_duration = 1.5;
            this->open_step_size = 0.4;

            this->subgait_name = "home_stand";
            this->p_x = 200;
            this->d_x = 0;
            this->p_y = 200;
            this->d_y = 0;
            this->subgait_start_time = 0;
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

            // Create our ROS node.
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<march_shared_resources::GaitActionGoal>(
                            "/march/gait/schedule/goal",
                            1,
                            boost::bind(&ComController::OnRosMsg, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                    std::thread(std::bind(&ComController::QueueThread, this));
        }

        public: void OnRosMsg(const march_shared_resources::GaitActionGoalConstPtr &_msg)
        {
            std::cout << "Its a message!" << '\n';
            std::cout << _msg->goal.current_subgait.name << '\n';
            this->subgait_name = _msg->goal.current_subgait.name;
            this->subgait_duration = _msg->goal.current_subgait.duration.sec + 0.000000001 * _msg->goal.current_subgait.duration.nsec;
            this->subgait_start_time = this->model->GetWorld()->SimTime().Double();
            std::cout << this->subgait_duration << '\n';

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

        // Called by the world update start event
        private: double GetMass() {
            double mass;
            for (auto const& link : this->model->GetLinks()) {
                mass += link->GetInertial()->Mass();
            }
            return mass;
        }

        // Called by the world update start event
        private: ignition::math::v4::Vector3<double> GetCom() {
            ignition::math::v4::Vector3<double> com(0.0, 0.0, 0.0);
            for (auto const& link : this->model->GetLinks()) {
                com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
            }
            return com / this->GetMass();
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            // Note: the exo moves in the negative x direction, and the right leg is in the positive y direction
            double time = this->model->GetWorld()->SimTime().Double();

            auto model_com = this->GetCom();
            auto foot_left_pose = this->foot_left->WorldCoGPose().Pos();
            auto foot_right_pose = this->foot_right->WorldCoGPose().Pos();

            double goal_position_y = 0.5 * (foot_left_pose.Y() + foot_right_pose.Y());
            double goal_position_x = 0.5 * (foot_left_pose.X() + foot_right_pose.X()) - 0.1;
            double error_x = model_com.X() - goal_position_x;
            double error_y = model_com.Y() - goal_position_y;

            double F_x = -this->p_x * error_x - this->d_x * (error_x - this->error_x_last_timestep);
            double F_y = -this->p_y * error_y - this->d_y * (error_y - this->error_y_last_timestep);
            const ignition::math::v4::Vector3<double> vec(F_x, F_y, 0.0);

            std::cout << F_x << "    " << F_y << "\n";


            double F_z = 9.81 * 0.5 * 0.95 * this->GetMass();
            if (time - subgait_start_time > 0.8 * subgait_duration){
                F_z = F_z * (5 - 5 * (time - subgait_start_time)/subgait_duration);
            }
            const ignition::math::v4::Vector3<double> Fvecz(0.0, 0.0, F_z);

            if (subgait_name == "home_stand"){
//                goal_position_x = foot_left_pose.X();
//                goal_position_y = foot_left_pose.Y();
//                this->model->GetLink("hip_aa_frame_right_side")->AddForce(Fvecz);
//                this->model->GetLink("hip_aa_frame_right_side")->AddForce(Fvecz);
//                this->model->GetLink("upper_leg_left")->AddForce(vec);
            }
            else if (subgait_name == "right_open") {
//                goal_position_x = foot_left_pose.X() - 0.5 * (time - subgait_start_time) *
//                                                       open_step_size / subgait_duration;
//                goal_position_y = foot_left_pose.Y() + 0.5 * (foot_right_pose.Y() - foot_left_pose.Y()) *
//                                                       (1 - std::cos(3.14*(time - subgait_start_time)/subgait_duration));
                this->model->GetLink("hip_aa_frame_right_side")->AddForce(Fvecz);
                this->model->GetLink("upper_leg_left")->AddForce(vec);
            }
            else if (subgait_name == "right_swing") {
//                goal_position_x = foot_left_pose.X() + 0.25 * swing_step_size -
//                                                       0.5 * (time - subgait_start_time) * swing_step_size / subgait_duration;
//                goal_position_y = foot_left_pose.Y() + 0.5 * (foot_right_pose.Y() - foot_left_pose.Y()) *
//                                                       (1 - std::cos(3.14*(time - subgait_start_time)/subgait_duration));
                this->model->GetLink("hip_aa_frame_right_side")->AddForce(Fvecz);
                this->model->GetLink("upper_leg_left")->AddForce(vec);
            }
            else if (subgait_name == "left_swing") {
//                goal_position_x = foot_right_pose.X() + 0.25 * swing_step_size -
//                                                        0.5 * (time - subgait_start_time) * swing_step_size / subgait_duration;
//                goal_position_y = foot_right_pose.Y() + 0.5 * (foot_left_pose.Y() - foot_right_pose.Y()) *
//                                                        (1 - std::cos(3.14*(time - subgait_start_time)/subgait_duration));
                this->model->GetLink("hip_aa_frame_left_side")->AddForce(Fvecz);
                this->model->GetLink("upper_leg_right")->AddForce(vec);
            }
            else {
                std::cout << subgait_name << '\n';
            }



        }

    private:
        physics::ModelPtr model;
        physics::LinkPtr foot_left;
        physics::LinkPtr foot_right;
        double subgait_start_time;
        double swing_step_size;
        double subgait_duration;
        double open_step_size;
        double leg_leg_distance;
        double p_x;
        double d_x;
        double p_y;
        double d_y;
        double error_x_last_timestep;
        double error_y_last_timestep;
        std::string subgait_name;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ComController)
}
