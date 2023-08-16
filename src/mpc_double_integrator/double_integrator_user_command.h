#ifndef USER_COMMAND_H_
#define USER_COMMAND_H_

#include <iostream>
//drake
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/geometry/meshcat.h"

class UserCommand final : public drake::systems::LeafSystem<double> {
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UserCommand)

    UserCommand(drake::multibody::MultibodyPlant<double> *plant, const std::shared_ptr<drake::geometry::Meshcat> meshcat);

    void OutputDesire(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const;

private:
    std::shared_ptr<drake::geometry::Meshcat> meshcat_;
    uint32_t nq_;
    uint32_t nv_;
};

#endif