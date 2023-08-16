#ifndef CARTPOLE_MPC_CONTROLLER_H_
#define CARTPOLE_MPC_CONTROLLER_H_

// system
#include <iostream>
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
// boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
//ocs2
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>

class My_MPC_Controller final : public drake::systems::LeafSystem<double> {
    public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(My_MPC_Controller)

    My_MPC_Controller(drake::multibody::MultibodyPlant<double> *plant, const std::string& taskFile, const std::string& librayFolder, bool verbose);

    void CalcU(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const;

    private:
    // member helper function
    ocs2::TargetTrajectories ReconstructTargetTrajectory(const double time, Eigen::VectorXd desire_state) const;
    void PrintDebugInfo (const ocs2::PrimalSolution primalSolution) const;
    void SetRobotStateInputDim();

    drake::multibody::MultibodyPlant<double> *plant_;
    // std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;

    uint32_t nq_;
    uint32_t nv_;
    uint32_t na_;

    std::unique_ptr<drake::systems::Context<double>> contextPtr_;
    Eigen::Matrix<double, -1, 1> initialState_{2};
    Eigen::Matrix<double, -1, 1> finalGoal_{2};

    ocs2::ddp::Settings ddpSettings_;
    ocs2::mpc::Settings mpcSettings_;

    ocs2::OptimalControlProblem problem_;
    std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

    std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
    std::unique_ptr<ocs2::Initializer> linearSystemInitializerPtr_;

    std::size_t STATE_DIM;
    std::size_t INPUT_DIM;

    std::shared_ptr<ocs2::GaussNewtonDDP_MPC> mpcPtr_;
    std::shared_ptr<ocs2::benchmark::RepeatedTimer> mpcTimerPtr_;
};


#endif // MPC_CONTROLLER_H_