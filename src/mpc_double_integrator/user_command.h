#ifndef USER_COMMAND_H_
#define USER_COMMAND_H_

#include <iostream>
#include <drake/common/drake_copyable.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <gflags/gflags.h>
//drake
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/framework/leaf_system.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/common/eigen_types.h"
#include "drake/common/schema/transform.h"

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