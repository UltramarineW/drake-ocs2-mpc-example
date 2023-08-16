#include "cartpole_mpc_controller.h"
#include <memory>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/misc/LoadData.h>
#include "ocs2_core/penalties/Penalties.h"
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include "ocs2_oc/synchronized_module/ReferenceManager.h"
#include "cartpole_dynamics.h"
#include "cartpole_parameters.h"

My_MPC_Controller::My_MPC_Controller(drake::multibody::MultibodyPlant<double> *plant, const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
    // preparation
    plant_ = plant;
    plant_context_ = plant_->CreateDefaultContext();
    SetRobotStateInputDim();
    
    this->DeclareVectorInputPort("desire_state", drake::systems::BasicVector<double>(STATE_DIM));
    this->DeclareVectorInputPort("state", drake::systems::BasicVector<double>(STATE_DIM));
    this->DeclareVectorOutputPort("u", drake::systems::BasicVector<double>(INPUT_DIM), &My_MPC_Controller::CalcU);

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
    // ocs2::loadData::loadEigenMatrix(taskFile, "finalGoal", finalGoal_);

    // DDP-MPC settings
    ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp", verbose);
    mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);

    // reference interface
    referenceManagerPtr_ = std::make_shared<ocs2::ReferenceManager>();

    // Optimal Control Problem
    ocs2::matrix_t Q(STATE_DIM, STATE_DIM);
    ocs2::matrix_t R(INPUT_DIM, INPUT_DIM);
    ocs2::matrix_t Qf(STATE_DIM, STATE_DIM);
    ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
    ocs2::loadData::loadEigenMatrix(taskFile, "R", R);
    ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", Qf);

    if (verbose) {
        std::cerr << "Q:  \n" << Q << "\n";
        std::cerr << "R:  \n" << R << "\n";
        std::cerr << "Q_final:\n" << Qf << "\n";
    }

    problem_.costPtr->add("cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));
    problem_.finalCostPtr->add("finalCost", std::make_unique<ocs2::QuadraticStateCost>(Qf));

    std::cerr << "Start loading dynamics" << std::endl;
    // Cartpole System Dynamics
    CartPoleParameters cartPoleParameters;
    cartPoleParameters.loadSettings(taskFile, "cartpole_parameters", verbose);
    problem_.dynamicsPtr.reset(new CartPoleSystemDynamics(cartPoleParameters, libraryFolder, STATE_DIM, INPUT_DIM, verbose));
    std::cerr << "End loading dynamics" << std::endl;
    // Constraints
    auto getPenalty = [&]() {
        using penalty_type = ocs2::augmented::SlacknessSquaredHingePenalty;
        penalty_type::Config boundsConfig;
        ocs2::loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, verbose);
        return penalty_type::create(boundsConfig);
    };
    auto getConstraint = [&]() {
        using vector_t = ocs2::vector_t;
        using matrix_t = ocs2::matrix_t;
        constexpr size_t numIneqConstraint = 2;
        const vector_t e = (vector_t(numIneqConstraint) << cartPoleParameters.maxInput_, cartPoleParameters.maxInput_).finished();
        const vector_t D = (vector_t(numIneqConstraint) << 1.0, -1.0).finished();
        const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
        return std::make_unique<ocs2::LinearStateInputConstraint>(e, C, D);
    };
    problem_.inequalityLagrangianPtr->add("InputLimits", ocs2::create(getConstraint(), getPenalty()));

    // Rollout
    auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);
    rolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));
    linearSystemInitializerPtr_.reset(new ocs2::DefaultInitializer(INPUT_DIM));

    // mpc
    mpcPtr_ = std::make_shared<ocs2::GaussNewtonDDP_MPC>(mpcSettings_, ddpSettings_, *rolloutPtr_, problem_, *linearSystemInitializerPtr_);
    mpcPtr_->getSolverPtr()->setReferenceManager(referenceManagerPtr_);
    mpcPtr_->reset();
    ocs2::TargetTrajectories initTargetTrajectory = ReconstructTargetTrajectory(0.0, Eigen::VectorXd::Constant(2, 0.0));
    mpcPtr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectory);

    //mpc timer
    mpcTimerPtr_ = std::make_shared<ocs2::benchmark::RepeatedTimer>();
    mpcTimerPtr_->reset();
}

ocs2::TargetTrajectories My_MPC_Controller::ReconstructTargetTrajectory(const double time, Eigen::VectorXd desire_state) const{
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

void My_MPC_Controller::PrintDebugInfo (const ocs2::PrimalSolution primalSolution) const{
    const std::size_t N = primalSolution.timeTrajectory_.size();

    std::cout << "mpc solution" << std::endl;
    for (std::size_t k = 0; k < N; k++) {
        std::cout << "time: " << primalSolution.timeTrajectory_[k] << "  state: ";
        for (std::size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++) {
            std::cout << primalSolution.stateTrajectory_[k](j) << " ";
        }
        std::cout << "input: ";
        for (std::size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) {
            std::cout <<  primalSolution.inputTrajectory_[k](j) << " ";
        }
        std::cout << "\n" << std::endl;
    }
    std::cout << "\n\n" << std::endl;
}

void My_MPC_Controller::SetRobotStateInputDim() {
    nq_ = plant_->num_positions();
    nv_ = plant_->num_velocities();
    na_ = plant_->num_actuated_dofs();
    STATE_DIM = nq_ + nv_;
    INPUT_DIM = na_;
}

void My_MPC_Controller::CalcU(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const {
    auto& state = get_input_port(1).Eval(context);
    auto& state_desire = get_input_port(0).Eval(context);

    Eigen::VectorXd reverse_state(STATE_DIM), reverse_desire_state(STATE_DIM);
    reverse_state << state(1), state(0), state(3), state(2);
    reverse_desire_state << state_desire(1), state_desire(0), state_desire(3), state_desire(2);

    const Eigen::VectorXd& qv(state);
    plant_->SetPositionsAndVelocities(plant_context_.get(), qv);
    Eigen::Vector3d p_WoP_W(3, 5, 7);
    const drake::multibody::SpatialMomentum<double> spatial_momentum = plant_->CalcSpatialMomentumInWorldAboutPoint(*plant_context_, p_WoP_W);
    // std::cout << "SpacialMomentum" << spatial_momentum << std::endl;

    // struct for ocs2 mpc preparation
    mpcTimerPtr_->startTimer();
    auto time = context.get_time();

    ocs2::TargetTrajectories target_trajectory = ReconstructTargetTrajectory(time, reverse_desire_state);
    mpcPtr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(target_trajectory);

    // start mpc computation
    bool controllerIsUpdated = mpcPtr_->run(time, reverse_state);
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

    // reprocess
    double timeWindow = mpcPtr_->settings().solutionTimeWindow_;
    if (mpcPtr_->settings().solutionTimeWindow_ < 0) {
        timeWindow = mpcPtr_->getSolverPtr()->getFinalTime() - time;
    }
    if (timeWindow < 2.0 * mpcTimerPtr_->getAverageInMilliseconds() * 1e-3) {
        std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!\n";
    }

    // PrintDebugInfo(primalSolution);
    // std::cout << primalSolution.inputTrajectory_[0] << std::endl;

    output->SetFromVector(primalSolution.inputTrajectory_[0]);
}