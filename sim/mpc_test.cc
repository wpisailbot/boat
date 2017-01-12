#include "gtest/gtest.h"

#include "sim_physics.h"
#include "sim_inter.h"
#include "dlib/dlib/optimization/optimization_bobyqa.h"

namespace sailbot {
namespace sim {
namespace testing {

class SimpleMPC : public Node {
 public:
  SimpleMPC() : Node(0.02), ctrl_dyn_(0.02) {}
 private:
  void Iterate() override {
    const double kdtctrl = 0.05;
    const double horizon = 2;
    const double rho_begin = .1;
    const double rho_end = 0.001;
    constexpr int N = 15;
    const double npt = 2 * TrivialDynamics::kNumUInputs * N + 1;
    const double max_evals = 1000;
    typedef dlib::matrix<double, TrivialDynamics::kNumUInputs * N, 1> XType;
    XType X0;
    X0 *= 0;
    XType lower;
    for (int i = 0; i < N; ++i) {
      lower(2*i, 0) = -1;
      lower(2*i+1, 0) = -kMaxBallastU;
    }
    XType upper = -lower;

    std::function<double(TrivialDynamics::VectorStates,
                         TrivialDynamics::VectorInputs)> step_cost_fun =
        std::bind(&SimpleMPC::StepCost, this, std::placeholders::_1,
                  std::placeholders::_2);
    std::function<double(TrivialDynamics::VectorStates)> end_cost_fun =
        std::bind(&SimpleMPC::EndCost, this, std::placeholders::_1);
    std::function<double(XType)> obj_fun =
        std::bind(&TrivialDynamics::ObjectiveFun<N>, &ctrl_dyn_,
                  std::placeholders::_1, X_, kdtctrl, horizon,
                  &SimpleMPC::SimpleCtrl, step_cost_fun, end_cost_fun);
    dlib::find_min_bobyqa(obj_fun, X0, npt, lower,
                          upper, rho_begin, rho_end, max_evals);
    LOG(INFO) << "X0: " << X0;
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

    const double kH = 30;
    const double kD = 7;
    const double kHD = 7;

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
  double StepCost(TrivialDynamics::VectorStates X,
                         TrivialDynamics::VectorInputs U) {
    double heel = X(0, 0);
    double heeldot = X(4, 0);
    double deltab = X(8, 0);
    double deltabdot = X(9, 0);
    double deltasdot = U(0, 0);
    double deltabtau = U(1, 0);
    double absheel = std::abs(heel);
    double absdeltab = std::abs(deltab);

    // Various constants.
    const double kNormalHeel = 1; // Everything relative to this.
    const double kHeelDot = 0.;
    const double kBallastPos = 0.2;
    const double kBallastVel = 0.2;
    const double kSailVel = .1;
    const double kBallastTorque = .3;

    double cost = 0;
    // TODO(james): Uncomment/implement more complicated components.
    // Penalize normal heel
    cost += kNormalHeel * heel * heel;
    // Penalize excessive heel TODO(james): Cheaper function than pow?
    //cost +=
    //    absheel > kTolerableHeel ? kExcessiveHeel * std::pow(2, absheel) : 0;
    // Penalize heel movement TODO(james) Penalize more when heel is capsizing
    // rather than righting boat?
    cost += kHeelDot * heeldot * heeldot;
    // Penalize keeping the ballast way out. Shouldn't be too big a factor.
    // Although we do want to make sure that we have spare authority.
    cost += kBallastPos * deltab * deltab;
    // Penalize moving the ballast past full extension (very dangerous).
    // TODO(james): Remove discontinuity
    //cost += absdeltab > kTolerableBallast ? kExtremeBallast : 0;
    // Penalize swinging the ballast & sail around like crazy.
    cost += kBallastVel * deltabdot * deltabdot;
    cost += kSailVel * deltasdot * deltasdot;
    // Penalize high ballast torque.
    cost += kBallastTorque * deltabtau * deltabtau;
    // Penalize poor angle-of-attack choice. TODO(james): Implement
    //cost +=
    //    kSailTrim * std::pow((ctrl_dyn_.get_alpha_sail() - alphadesired), 2);
    return cost;
  }

  double EndCost(TrivialDynamics::VectorStates X) {
    const double kEndScale = 1;
    return kEndScale * StepCost(X, TrivialDynamics::VectorInputs::Zero());
  };
  static constexpr double kMaxBallastU = 100;
  TrivialDynamics ctrl_dyn_;
  TrivialDynamics::VectorStates X_;
};

constexpr double SimpleMPC::kMaxBallastU;

class MPCTest : public ::testing::Test {
 public:
  MPCTest() : sim_dyn_() {}

  void SetUp() override {
    util::ClockManager::SetFakeClock(true);
    std::thread clock_thread(&util::ClockManager::Run, 0);
    clock_thread.detach();
    sim_dyn_.Run();
    mpc_.Run();
  }
  void TearDown() override {
  }
 private:
  SimulatorNode sim_dyn_;
  SimpleMPC mpc_;
};

TEST_F(MPCTest, SimpleTest) {
  util::ClockInstance clock;
  EXPECT_TRUE(false);
  clock.SleepUntil(clock.Time() + std::chrono::seconds(1));
  EXPECT_TRUE(false);
}

int main(int argc, char **argv) {
  util::Init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  std::cout << "Hello, World!";
  return RUN_ALL_TESTS();
}

}  // namespace testing
}  // namespace sim
}  // namespace sailbot
