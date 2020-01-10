#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <march_shared_resources/GaitActionGoal.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace gazebo {

class ComController : public ModelPlugin {

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM(
          "A ROS node for Gazebo has not been initialized, unable to load "
          "plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
             "the gazebo_ros package)");
      return;
    }

    // Store the pointer to the model
    this->model = _parent;
    this->leg_leg_distance = 0.5;
    this->swing_step_size = 0.8;
    this->subgait_duration = 1.5;
    this->open_step_size = 0.4;

    this->subgait_name = "home_stand";
    this->p_pitch = 60;
    this->d_pitch = 10;
    this->p_roll = 150;
    this->d_roll = 7.5;
    this->p_yaw = 500;
    this->d_yaw = 25;
    this->subgait_start_time = 0;
    this->error_x_last_timestep = 0;
    this->error_y_last_timestep = 0;
    this->error_yaw_last_timestep = 0;

    // Apply a small linear velocity to the model.
    for (auto const &i : this->model->GetLinks()) {
      std::cout << i->GetName() << "\n";
    }

    this->foot_left = this->model->GetLink("ankle_plate_left");
    this->foot_right = this->model->GetLink("ankle_plate_right");
    this->base_link = this->model->GetLink("base_link");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ComController::OnUpdate, this));

    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<march_shared_resources::GaitActionGoal>(
            "/march/gait/schedule/goal", 1,
            boost::bind(&ComController::OnRosMsg, this, _1), ros::VoidPtr(),
            &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&ComController::QueueThread, this));
  }

public:
  void OnRosMsg(const march_shared_resources::GaitActionGoalConstPtr &_msg) {
    std::cout << "Its a message!" << '\n';
    std::cout << _msg->goal.current_subgait.name << '\n';
    this->subgait_name = _msg->goal.current_subgait.name;
    this->subgait_duration =
        _msg->goal.current_subgait.duration.sec +
        0.000000001 * _msg->goal.current_subgait.duration.nsec;
    this->subgait_start_time = this->model->GetWorld()->SimTime().Double();
    std::cout << this->subgait_duration << '\n';
  }

  /// \brief ROS helper function that processes messages
private:
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  // Called by the world update start event
private:
  double GetMass() {
    double mass;
    for (auto const &link : this->model->GetLinks()) {
      mass += link->GetInertial()->Mass();
    }
    return mass;
  }

  // Called by the world update start event
private:
  ignition::math::v4::Vector3<double> GetCom() {
    ignition::math::v4::Vector3<double> com(0.0, 0.0, 0.0);
    for (auto const &link : this->model->GetLinks()) {
      com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
    }
    return com / this->GetMass();
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    // Note: the exo moves in the negative x direction, and the right leg is in
    // the positive y direction
    double time = this->model->GetWorld()->SimTime().Double();

    auto model_com = this->GetCom();
    auto foot_left_pose = this->foot_left->WorldCoGPose().Pos();
    auto foot_right_pose = this->foot_right->WorldCoGPose().Pos();

    double goal_position_x = 0.5 * (foot_left_pose.X() + foot_right_pose.X()) - 0.05;
    double goal_position_y;

      if (subgait_name == "home_stand"){
                goal_position_x = foot_left_pose.X();
                goal_position_y = foot_left_pose.Y();
      }
      else if (subgait_name == "right_open") {
                goal_position_x = foot_left_pose.X() - 0.5 * (time - subgait_start_time) *
                                                       open_step_size / subgait_duration;
                goal_position_y = foot_left_pose.Y();
      }
      else if (subgait_name == "right_swing") {
                goal_position_x = foot_left_pose.X() + 0.25 * swing_step_size -
                                                       0.5 * (time - subgait_start_time) * swing_step_size / subgait_duration;
                goal_position_y = foot_left_pose.Y();
      }
      else if (subgait_name == "left_swing") {
                goal_position_x = foot_right_pose.X() + 0.25 * swing_step_size -
                                                        0.5 * (time - subgait_start_time) * swing_step_size / subgait_duration;
                goal_position_y = foot_right_pose.Y();
      }
      else {
          std::cout << subgait_name << '\n';
      }





//    if (subgait_name == "home_stand") {
//      goal_position_y =
//          foot_left_pose.Y();
//    } else if (subgait_name == "right_open") {
//        goal_position_y =
//                foot_left_pose.Y();
//    } else if (subgait_name == "right_swing") {
//        goal_position_y =
//                foot_left_pose.Y();
//    } else if (subgait_name == "left_swing") {
//      goal_position_y =
//                foot_right_pose.Y();
//    } else {
//      std::cout << subgait_name << '\n';
//    }

    double goal_roll = 0.5 * (foot_left_pose.Y() + foot_right_pose.Y());
    double error_x = model_com.X() - goal_position_x;
    double error_y = model_com.Y() - goal_position_y;
    double error_yaw = foot_left->WorldPose().Rot().Z();

    double T_pitch = -this->p_pitch * error_x -
                     this->d_pitch * (error_x - this->error_x_last_timestep);
    double T_roll = this->p_roll * error_y +
                    this->d_roll * (error_y - this->error_y_last_timestep);
    double T_yaw = -this->p_yaw * error_yaw -
                   this->d_yaw * (error_yaw - this->error_yaw_last_timestep);

    this->error_x_last_timestep = error_x;
    this->error_y_last_timestep = error_y;
    this->error_yaw_last_timestep = error_yaw;

    const ignition::math::v4::Vector3<double> torque_all(
        0, T_pitch, T_yaw); // -roll, pitch, -yaw
    const ignition::math::v4::Vector3<double> torque_stable(
        T_roll, 0, 0); // -roll, pitch, -yaw

    for (auto const &link : this->model->GetLinks()) {
      link->AddTorque(torque_all);
      if (subgait_name == "left_swing") {
        if (link->GetName().find("right") != std::string::npos) {
          link->AddTorque(torque_stable);
        }
      } else {
        if (link->GetName().find("left") != std::string::npos) {
          link->AddTorque(torque_stable);
        }
      }
    }
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr foot_left;
  physics::LinkPtr foot_right;
  physics::LinkPtr base_link;
  double subgait_start_time;
  double swing_step_size;
  double subgait_duration;
  double open_step_size;
  double leg_leg_distance;
  double p_yaw;
  double d_yaw;
  double p_pitch;
  double d_pitch;
  double p_roll;
  double d_roll;
  double error_x_last_timestep;
  double error_y_last_timestep;
  double error_yaw_last_timestep;
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
} // namespace gazebo
