#include "cartpole_user_command.h"
#include <drake/systems/framework/basic_vector.h>

UserCommand::UserCommand(drake::multibody::MultibodyPlant<double> *plant, const std::shared_ptr<drake::geometry::Meshcat> meshcat) : meshcat_(meshcat)
{
    nq_ = plant->num_positions();
    nv_ = plant->num_velocities();
    this->DeclareVectorOutputPort("desire_qv", drake::systems::BasicVector<double>(nq_+nv_), &UserCommand::OutputDesire);
}

void UserCommand::OutputDesire(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const {
    Eigen::VectorXd desire_state(nq_ + nv_);
    desire_state << meshcat_->GetSliderValue("Desire Target"), 0.0, 0.0, 0.0;
    output->SetFromVector(desire_state);
}