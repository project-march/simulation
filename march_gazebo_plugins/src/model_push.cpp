#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Apply a small linear velocity to the model.
            for (auto const& i : this->model->GetLinks()) {
                std::cout << i->GetName() << "\n";
            }

            this->foot_left = this->model->GetLink("ankle_plate_left");
            this->foot_right = this->model->GetLink("ankle_plate_right");

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ModelPush::OnUpdate, this));
        }

            // Called by the world update start event
        void OnUpdate()
            {
            }

            // Pointer to the model
        private:
        physics::ModelPtr model;
        physics::LinkPtr foot_left;
        physics::LinkPtr foot_right;

            // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
