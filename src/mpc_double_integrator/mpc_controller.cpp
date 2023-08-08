#include "mpc_controller.h"

My_MPC_Controller::My_MPC_Controller(drake::multibody::MultibodyPlant<double> *plant, const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
    // preparation
    // plant_.reset(plant);
    plant_ = plant;
    plant_context_ = plant_->CreateDefaultContext();
    uint32_t nq = plant_->num_positions();
    uint32_t nv = plant_->num_velocities();
    uint32_t na = plant_->num_actuated_dofs();
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
    // ocs2::loadData::loadEigenMatrix(taskFile, "finalGoal", finalGoal_);

    // DDP-MPC settings
    ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp", verbose);
    mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);

    // reference interface
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

void My_MPC_Controller::CalcU(const drake::systems::Context<double>& context, drake::systems::BasicVector<double> *output) const {
    Eigen::VectorXd real_state(2), real_desire_state(2);
    auto& state = get_input_port(1).Eval(context);
    auto& state_desire = get_input_port(0).Eval(context);


    real_state << state.segment(0, 1), state.segment(2, 1);
    real_desire_state << state_desire.segment(0, 1), state_desire.segment(2, 1);

    const Eigen::VectorXd& qv(state);
    plant_->SetPositionsAndVelocities(plant_context_.get(), qv);
    Eigen::Vector3d p_WoP_W(3, 5, 7);
    const drake::multibody::SpatialMomentum<double> spatial_momentum = plant_->CalcSpatialMomentumInWorldAboutPoint(*plant_context_, p_WoP_W);
    // std::cout << "SpacialMomentum" << spatial_momentum << std::endl;

    // struct for ocs2 mpc preparation
    mpcTimerPtr_->startTimer();
    auto time = context.get_time();

    ocs2::TargetTrajectories initTargetTrajectory = ReconstructTargetTrajectory(time, real_desire_state);
    mpcPtr_->getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectory);

    // start mpc computation
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