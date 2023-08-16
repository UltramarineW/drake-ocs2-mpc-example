#ifndef CARTPOLE_DYNAMICS_H_
#define CARTPOLE_DYNAMICS_H_

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include "cartpole_parameters.h"


class CartPoleSystemDynamics : public ocs2::SystemDynamicsBaseAD {
public:
  CartPoleSystemDynamics(const CartPoleParameters& cartPoleParameters, const std::string& libraryFolder,uint32_t STATE_DIM, uint32_t INPUT_DIM, bool verbose)
      : param_(cartPoleParameters) {
    STATE_DIM_ = STATE_DIM;
    INPUT_DIM_ = INPUT_DIM;
    initialize(STATE_DIM_, INPUT_DIM_, "cartpole_dynamics", libraryFolder, true, verbose);
  }

  ~CartPoleSystemDynamics() override = default;

  CartPoleSystemDynamics(const CartPoleSystemDynamics& rhs) = default;

  CartPoleSystemDynamics* clone() const override { return new CartPoleSystemDynamics(*this); }

    ocs2::ad_vector_t systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                            const ocs2::ad_vector_t& parameters) const override {
    const ocs2::ad_scalar_t cosTheta = cos(state(0));
    const ocs2::ad_scalar_t sinTheta = sin(state(0));

    // Inertia tensor
    Eigen::Matrix<ocs2::ad_scalar_t, 2, 2> I;
    I << static_cast<ocs2::ad_scalar_t>(param_.poleSteinerMoi_), 
         static_cast<ocs2::ad_scalar_t>(param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
         static_cast<ocs2::ad_scalar_t>(param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
         static_cast<ocs2::ad_scalar_t>(param_.cartMass_ + param_.poleMass_);

    // RHS
    Eigen::Matrix<ocs2::ad_scalar_t, 2, 1> rhs(param_.poleMass_ * param_.poleHalfLength_ * param_.gravity_ * sinTheta,
                                         input(0) + param_.poleMass_ * param_.poleHalfLength_ * pow(state(2), 2) * sinTheta);

    // dxdt
    ocs2::ad_vector_t stateDerivative(STATE_DIM_);
    stateDerivative << state.tail<2>(), I.inverse() * rhs;
    return stateDerivative;
  }

 private:
  CartPoleParameters param_;
  uint32_t STATE_DIM_;
  uint32_t INPUT_DIM_;
};

#endif