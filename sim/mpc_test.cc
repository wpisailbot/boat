#include "gtest/gtest.h"

#include "sim_physics.h"
#include "sim_inter.h"

namespace sailbot {
namespace sim {
namespace testing {

class SimpleMPC : public Node {
 public:
 private:
  void Iterate() override {
    const double kdtctrl = 0.01;
    const double horizon = 3;
    dlib::find_min_bobyqa(std::bind(&TrivialDynamics::ObjectiveFun, &ctrl_dyn_,
                                    std::placeholders::_1, X_, kdtctrl, horizon,
                                    simple_ctrl, StepCost, EndCost));
  }
  static TrivialDynamics::VectorInputs
  SimpleCtrl(TrivialDynamics::VectorStates X) {
    // For this, we've got a simple job: Do something that doesn't instantly
    // sink the boat. Ideally, this control algorithm should be something
    // reasonable enough to run without MPC if necessary.
    // TODO(james): Tune to make sense.

    // For now, don't worry any about what to do with the sail, just worry about
    // the movable ballast.
    TrivialDynamics::VectorInputs U;
    U(0, 0) = 0;
    // For the movable ballast, do basic PID-type control.
    // Attempt to drive the boat to 0 heel.
    double heel = X(0, 0);
    double heeldot = X(4, 0);
    double deltab = X(8, 0);
    double deltabdot = X(9, 0);
    // TODO(james): Create reasonable constants.
    double move = -kH * heel - kD * deltabdot - kHD * heeldot;
    // Essentially, we say if the algorithm tries to extend the ballast more in
    // the current direction, then it will get less authority as it approaches
    // full extension and will always push to full extension rather than going
    // past it. However, once it tries to pull in again, give it full authority.
    bool pushing_out = (move > 0 && deltab > 0) || (move < 0 && deltab < 0);
    U(1, 0) = move * (pushing_out ? std::cos(deltab) : 1);
    U(1, 0) = std::min(kMaxBallastU, std::max(-kMaxBallastU, U(1, 0)));
    return U;
  }
  static double StepCost(TrivialDynamics::VectorStates X,
                         TrivialDynamics::VectorInputs U) {
    double heel = X(0, 0);
    double heeldot = X(4, 0);
    double deltab = X(8, 0);
    double deltabdot = X(9, 0);
    double deltasdot = U(0, 0);
    double deltabtau = U(1, 0);
    double absheel = std::abs(heel);
    double absdeltab = std::abs(deltab);

    double cost = 0;
    // Penalize normal heel
    cost += kNormalHeel * heel * heel;
    // Penalize excessive heel TODO(james): Cheaper function than pow?
    cost +=
        absheel > kTolerableHeel ? kExcessiveHeel * std::pow(2, absheel) : 0;
    // Penalize heel movement TODO(james) Penalize more when heel is capsizing
    // rather than righting boat?
    cost += kHeelDot * heeldot * heeldot;
    // Penalize keeping the ballast way out. Shouldn't be too big a factor.
    // Although we do want to make sure that we have spare authority.
    cost += kBallastPos * deltab * deltab;
    // Penalize moving the ballast past full extension (very dangerous).
    // TODO(james): Remove discontinuity
    cost += absdeltab > kTolerableBallast ? kExtremeBallast : 0;
    // Penalize swinging the ballast & sail around like crazy.
    cost += kBallastVel * deltabdot * deltabdot;
    cost += kSailVel * deltasdot * deltasdot;
    // Penalize high ballast torque.
    cost += kBallastTorque * deltabtau * deltabtau;
    // Penalize poor angle-of-attack choice. TODO(james): Implement
    cost += kSailTrim * (alpha - alphadesired) ** 2;
  }
  const double kMaxBallastU = 1; // TODO: Reasonable value.
  TrivialDynamics ctrl_dyn_;
  TrivialDynamics::VectorStates X_;
};

class MPCTest : public ::testing::Test {
 public:
  MPCTest() : sim_dyn_(0.001), ctrl_dyn_(0.001) {}

  void SetUp() override {
  }
  void TearDown() override {
  }
 private:
  SimulatorNode sim_dyn_;
};

}  // namespace testing
}  // namespace sim
}  // namespace sailbot
