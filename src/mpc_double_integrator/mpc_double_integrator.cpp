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
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
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

DEFINE_double(dt, 0.01, "system control update time");
DEFINE_double(simulation_time, 2.0, "simulation time");

class My_MPC_Controller final : public drake::systems::LeafSystem<double> {
    public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(My_MPC_Controller)

    My_MPC_Controller(drake::multibody::MultibodyPlant<double> &plant, const std::string& taskFile, const std::string& librayFolder, bool verbose);

    void CalcU(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const;

    private:

    ocs2::TargetTrajectories reconstructTargetTrajectory(const double time, Eigen::VectorXd desire_state) const;

    std::unique_ptr<drake::systems::Context<double>> plant_context_;
    std::unique_ptr<drake::systems::Context<double>> contextPtr_;
    Eigen::Matrix<double, -1, 1> initialState_{2};
    Eigen::Matrix<double, -1, 1> finalGoal_{2};

    ocs2::ddp::Settings ddpSettings_;
    ocs2::mpc::Settings mpcSettings_;

    ocs2::OptimalControlProblem problem_;
    std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

    std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
    std::unique_ptr<ocs2::Initializer> linearSystemInitializerPtr_;

    const std::size_t STATE_DIM = 2;
    const std::size_t INPUT_DIM = 1;

    std::shared_ptr<ocs2::GaussNewtonDDP_MPC> mpcPtr_;
    std::shared_ptr<ocs2::benchmark::RepeatedTimer> mpcTimerPtr_;
};

My_MPC_Controller::My_MPC_Controller(drake::multibody::MultibodyPlant<double> &plant, const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
    // preparation
    plant_context_ = plant.CreateDefaultContext();
    uint32_t nq = plant.num_positions();
    uint32_t nv = plant.num_velocities();
    uint32_t na = plant.num_actuated_dofs();
    this->DeclareVectorInputPort("desire_state", drake::systems::BasicVector<double>(nq + nv));
    this->DeclareVectorInputPort("state", drake::systems::BasicVector<double>(nq + nv));
    this->DeclareVectorOutputPort("u", drake::systems::BasicVector<double>(na), &My_MPC_Controller::CalcU);

    // check task file exist
    boost::filesystem::path taskFilePath(taskFile);
    if (boost::filesystem::exists(taskFilePath)) {
        std::cerr << "[DoubleIntegratorInterface] Loading task file: " << taskFilePath << std::endl;
    } else {
        throw std::invalid_argument("[DoubleIntegratorInterface] Task file not found: " + taskFilePath.string());
    }
    // create library folder
    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    std::cerr << "[DoubleIntegratorInterface] Generated library path: " << libraryFolderPath << std::endl;

    // Default initial condition and final goal
    ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
    ocs2::loadData::loadEigenMatrix(taskFile, "finalGoal", finalGoal_);

    // DDP-MPC settings
    ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp", verbose);
    mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);

    // ros interface
    referenceManagerPtr_.reset(new ocs2::ReferenceManager);

    // Optimal Control Problem
    ocs2::matrix_t Q(STATE_DIM, STATE_DIM);
    ocs2::matrix_t R(INPUT_DIM, INPUT_DIM);
    ocs2::matrix_t Qf(STATE_DIM, STATE_DIM);
    ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
    ocs2::loadData::loadEigenMatrix(taskFile, "R", R);
    ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
    problem_.costPtr->add("cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));
    problem_.finalCostPtr->add("finalCost", std::make_unique<ocs2::QuadraticStateCost>(Qf));

    // Double Integrator System Dynamics
    const ocs2::matrix_t A_ = (ocs2::matrix_t(STATE_DIM, STATE_DIM) << 0.0, 1.0, 0.0, 0.0).finished();
    const ocs2::matrix_t B_ = (ocs2::matrix_t(STATE_DIM, INPUT_DIM) << 0.0, 1.0).finished();
    problem_.dynamicsPtr.reset(new ocs2::LinearSystemDynamics(A_, B_));

    // Rollout
    auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);
    rolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));
    linearSystemInitializerPtr_.reset(new ocs2::DefaultInitializer(INPUT_DIM));

    // mpc
    mpcPtr_ = std::make_shared<ocs2::GaussNewtonDDP_MPC>(mpcSettings_, ddpSettings_, *rolloutPtr_, problem_, *linearSystemInitializerPtr_);
    mpcPtr_->getSolverPtr()->setReferenceManager(referenceManagerPtr_);
    mpcPtr_->reset();
    ocs2::TargetTrajectories initTargetTrajectory = reconstructTargetTrajectory(0.0, Eigen::VectorXd::Constant(2, 0.0));
    mpcPtr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectory);

    //mpc timer
    mpcTimerPtr_ = std::make_shared<ocs2::benchmark::RepeatedTimer>();
    mpcTimerPtr_->reset();
}

ocs2::TargetTrajectories My_MPC_Controller::reconstructTargetTrajectory(const double time, Eigen::VectorXd desire_state) const{
    std::size_t N = 1;
    std::vector<double> desiredTimeTrajectory(N);
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> desiredStateTrajectory(N);
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> desireInputTrajectory(N);
    
    for (std::size_t i = 0; i < N; i++) {
        desiredTimeTrajectory[i] = time;
        desiredStateTrajectory[i] = desire_state;
        desireInputTrajectory[i] = Eigen::VectorXd::Constant(1, 0.0);
    }
    return {desiredTimeTrajectory, desiredStateTrajectory, desireInputTrajectory};
}

void My_MPC_Controller::CalcU(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const {
    Eigen::VectorXd real_state(2), real_desire_state(2);
    auto state = get_input_port(1).Eval(context);
    auto state_desire = get_input_port(0).Eval(context);
    real_state << state.segment(7, 1), state.segment(15, 1);
    real_desire_state << state_desire.segment(7, 1), state_desire.segment(15, 1);

    // struct for ocs2 mpc preparation
    mpcTimerPtr_->startTimer();
    auto time = context.get_time();

    ocs2::TargetTrajectories initTargetTrajectory = reconstructTargetTrajectory(time, state_desire);
    mpcPtr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectory);

    bool controllerIsUpdated = mpcPtr_->run(time, real_state);
    if(!controllerIsUpdated) {
        return;
    }
    double finalTime = time + mpcPtr_->settings().solutionTimeWindow_;
    if (mpcPtr_->settings().solutionTimeWindow_ < 0) {
        finalTime = mpcPtr_->getSolverPtr()->getFinalTime();
    }
    ocs2::PrimalSolution primalSolution;
    mpcPtr_->getSolverPtr()->getPrimalSolution(finalTime, &primalSolution);

    mpcTimerPtr_->endTimer();

    // 处理
    double timeWindow = mpcPtr_->settings().solutionTimeWindow_;
    if (mpcPtr_->settings().solutionTimeWindow_ < 0) {
        timeWindow = mpcPtr_->getSolverPtr()->getFinalTime() - time;
    }
    if (timeWindow < 2.0 * mpcTimerPtr_->getAverageInMilliseconds() * 1e-3) {
        std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!\n";
    }

    const std::size_t N = primalSolution.timeTrajectory_.size();

#if 1
    // time
    std::cout << "time: " << std::endl;
    for (auto t : primalSolution.timeTrajectory_) {
        std::cout << t << std::endl;
    }

    // state
    std::cout << "\nmpc solution state" << std::endl;
    
    for (std::size_t k = 0; k < N; k++) {
        for (std::size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++) {
            std::cout << primalSolution.stateTrajectory_[k](j) << " ";
        }
        std::cout << std::endl;
    }

    // input
    std::cout << "\nmpc solution input" << std::endl;
    for(std::size_t k = 0; k < N; k++) {
        for (std::size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) {
            std::cout <<  primalSolution.inputTrajectory_[k](j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "\n\n";

#endif

    output->SetFromVector(primalSolution.inputTrajectory_[0]);
}

std::string GetProgramPath() {
    char buffer[FILENAME_MAX];
    if(getcwd(buffer, FILENAME_MAX) != NULL) {
        std::string path = buffer;
        return path;
    }
    exit(-1);
}

// drake main process
int DoMain(const std::string exec_path) {
    drake::systems::DiagramBuilder<double> builder;
    auto scene_graph = builder.AddSystem<drake::geometry::SceneGraph<double>>();
    // configure multiplant
    auto plant = builder.AddSystem<drake::multibody::MultibodyPlant>(FLAGS_dt);
    plant->RegisterAsSourceForSceneGraph(scene_graph);
    std::string running_path = GetProgramPath();
    const std::string model_name = "/home/wujiayang/learning_drake/my_drake_test/src/mpc_double_integrator/double_integrator.urdf";
    drake::multibody::Parser(plant).AddModelFromFile(model_name);
    plant->Finalize();
    uint32_t nq = plant->num_positions();
    uint32_t nv = plant->num_velocities();
    uint32_t na = plant->num_actuators();
    std::cout << "nq: " << nq << " nv: " << nv << " na: " << na << "\n";
    // reset gravity vector
    plant->mutable_gravity_field().set_gravity_vector(Eigen::Vector3d(0, 0, 0));

    // get system build done 
    builder.Connect(plant->get_geometry_poses_output_port(), scene_graph->get_source_pose_port(plant->get_source_id().value()));
    builder.Connect(scene_graph->get_query_output_port(), plant->get_geometry_query_input_port());
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();
    auto& visualizer = drake::geometry::MeshcatVisualizerd::AddToBuilder(&builder, *scene_graph, meshcat);

    // controller constructor setting 
    const std::string taskFile = "/home/wujiayang/learning_drake/my_drake_test/src/mpc_double_integrator/task.info";
    const std::string LibraryPath = "/home/wujiayang/learning_drake/my_drake_test/src/mpc_double_integrator/auto_generated";
    auto controller = builder.AddSystem<My_MPC_Controller>(*plant, taskFile, LibraryPath, false);
    builder.Connect(plant->get_state_output_port(), controller->get_input_port(1));
    builder.Connect(controller->get_output_port(0), plant->get_actuation_input_port());
    auto diagram = builder.Build();

    // prepare to start the simulation
    drake::systems::Simulator<double> simulator(*diagram);
    auto& plant_context = diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
    auto& controller_context = diagram->GetMutableSubsystemContext(*controller, &simulator.get_mutable_context());
    drake::VectorX<double> q0 = drake::VectorX<double>::Zero(9);
    drake::VectorX<double> v0 = drake::VectorX<double>::Zero(8);
    q0 << 1.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0,
          0.0, 0.0;
    v0 << 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 
          0.0, 0.0;
    plant->SetPositions(&plant_context, q0);
    plant->SetVelocities(&plant_context, v0);
    drake::VectorX<double> desire_state = drake::VectorX<double>::Zero(q0.size() + v0.size());
    desire_state << 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    10.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0;
    controller->get_input_port(0).FixValue(&controller_context, desire_state);

    // start simulation
    simulator.Initialize();
    simulator.set_target_realtime_rate(1.0);
    visualizer.StartRecording();
    simulator.AdvanceTo(FLAGS_simulation_time);
    visualizer.StopRecording();
    visualizer.PublishRecording();
    return 0;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    const std::string exec_path = argv[0];
    return DoMain(exec_path);
}