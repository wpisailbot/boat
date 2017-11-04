#include "adaptive.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "control/util.h"

namespace sailbot {
namespace control {

constexpr float AdaptiveControl::dt;

ControlPhysics::ControlPhysics() {
  constexpr double rhoair = 1.225; // Density of air, kg / m^3
  constexpr double rhowater = 1000.0; // Density of water, kg / m^3
  consts_ = {.sail = {.A = 2.0,
                      .rho = rhoair,
                      .C = 1.4,
                      .K = 3,
                      .drageps = 0.0,
                      .maxalpha = M_PI_2},
             .keel = {.A = 0.3,
                      .rho = rhowater,
                      .C = 1.4,
                      .K = 8.0,
                      .drageps = 0.1,
                      .maxalpha = 0.25},
             .rudder = {.A = 0.04,
                        .rho = rhowater,
                        .C = 1.7,
                        .K = 5.0,
                        .drageps = 0.05,
                        .maxalpha = 0.25},
             .hs = 1.5,
             .hk = -0.7,
             .hr = 0.0,
             .rs = 0.1,
             .rk = 0.0,
             .rr = -0.9,
             .ls = 0.35,
             .lr = 0.0,
             .hb = -0.7,
             .m = 25.0,
             .g = 9.8,
             .J = 10.0,
             .Blon = 15.0,
             .Blat = 25.0,
             .Bomega = 500.0,
             .taubias = 0.0};
  // beta = Blon, Bomega, Ar, rs, taubias, 1.0
  betamin_ << 0.0, 0.0, 0.01, -1.0, -10.0, 1.0;
  betamax_ << 1000.0, 10000.0, 1.0, 1.0, 10.0, 1.0;
}

void ControlPhysics::IncrementBeta(const MatrixBeta &diff) {
  MatrixBeta b = beta();
  b += diff;
  // For sundry reasons, Eigen only provides the max/min operators for arrays...
  b = b.array().max(betamin_).min(betamax_).matrix();
  set_beta(b);
}

double ControlPhysics::RudderForTorque(double taus, double tauk, double taugoal,
                                       double heel, double thetac,
                                       double vc) const {
  double cthetac = std::max(std::cos(thetac), 0.5);
  double denom = 0.5 * consts_.rudder.rho * consts_.rudder.A * vc * vc *
                 consts_.rudder.K * std::cos(thetac) * std::cos(heel) *
                 consts_.rr;
  return (taugoal - taus - tauk - consts_.taubias) / denom - thetac;
}

void ControlPhysics::SailAirfoil(double alpha, double v, double *force,
                                 double *ang) const {
  const double absalpha = std::abs(alpha);
  const double signalpha = util::Sign(alpha);
  if (force != nullptr) {
    double trimalpha = std::max(absalpha - 0.0, 0.0);
    const double mag =
        consts_.sail.C * (1.0 - std::exp(-consts_.sail.K * trimalpha));
    *force = 0.5 * consts_.sail.rho * consts_.sail.A * v * v * mag;
  }
  if (ang != nullptr) {
    *ang = (M_PI_2 - std::abs(alpha)) * signalpha;
  }
}

void ControlPhysics::KeelAirfoil(double alpha, double v, double *force,
                                 double *ang) const {
  const double absalpha = std::abs(alpha);
  const double signalpha = util::Sign(alpha);
  if (force != nullptr) {
    const double mag = consts_.keel.K * std::min(absalpha, consts_.keel.maxalpha);
    *force = 0.5 * consts_.keel.rho * consts_.keel.A * v * v * mag;
  }
  if (ang != nullptr) {
    *ang = (M_PI_2 - consts_.keel.drageps) * signalpha;
  }
}

void ControlPhysics::RudderAirfoil(double alpha, double v, double *force,
                                   double *ang) const {
  const double absalpha = std::abs(alpha);
  const double signalpha = util::Sign(alpha);
  if (force != nullptr) {
    const double mag =
        consts_.rudder.K * std::min(absalpha, consts_.rudder.maxalpha);
    *force = 0.5 * consts_.rudder.rho * consts_.rudder.A * v * v * mag;
  }
  if (ang != nullptr) {
    *ang = (M_PI_2 - consts_.rudder.drageps) * signalpha;
  }
}

void ControlPhysics::SailForce(double thetaw, double vw, double deltas,
                               double *Fs, double *gammas) const {
  double alpha = -util::norm_angle(thetaw + deltas + M_PI);
  double ang;
  SailAirfoil(alpha, vw, Fs, &ang);
  if (gammas != nullptr) {
    *gammas = util::norm_angle(ang - thetaw);
  }
}

void ControlPhysics::KeelForce(double thetac, double vc, double *Fk,
                               double *gammak) const {
  double alpha = -util::norm_angle(thetac);
  double ang;
  KeelAirfoil(alpha, vc, Fk, &ang);
  if (gammak != nullptr) {
    *gammak = util::norm_angle(ang - thetac + M_PI);
  }
}

void ControlPhysics::RudderForce(double thetac, double vc, double deltar,
                                 double *Fr, double *gammar) const {
  double alpha = -util::norm_angle(thetac + deltar);
  double ang;
  RudderAirfoil(alpha, vc, Fr, &ang);
  if (gammar != nullptr) {
    *gammar = util::norm_angle(ang - thetac + M_PI);
  }
}

double ControlPhysics::SailTorque(double Fs, double gammas, double deltas,
                                  double heel) const {
  return Fs * ((consts_.rs - consts_.ls * std::cos(deltas)) * std::sin(gammas) *
                   std::cos(heel) +
               consts_.hk * std::cos(gammas) * std::sin(heel));
}

double ControlPhysics::KeelTorque(double Fk, double gammak, double heel) const {
  return Fk * (consts_.rk * std::sin(gammak) * std::cos(heel) +
               consts_.hk * std::cos(gammak) * std::sin(heel));
}

double ControlPhysics::RudderTorque(double Fr, double gammar, double heel) const {
  return Fr * consts_.rr * std::sin(gammar) * std::cos(heel);
}

double ControlPhysics::CalcHeel(double Fs, double gammas, double Fk,
                                double gammak) const {
  double tanheel =
      (Fs * consts_.hs * std::sin(gammas) + Fk * consts_.hk * std::sin(gammak)) /
      (consts_.hb * consts_.m * consts_.g);
  return std::atan(tanheel);
}

void ControlPhysics::NetForce(double thetaw, double vw, double thetac,
                              double vc, double deltas, double deltar,
                              double omega, double *Flon, double *Flat,
                              double *taunet, double *newheel,
                              MatrixY *Y) const {
  double Fs, gammas, Fk, gammak, Fr, gammar;
  SailForce(thetaw, vw, deltas, &Fs, &gammas);
  KeelForce(thetac, vc, &Fk, &gammak);
  RudderForce(thetac, vc, deltar, &Fr, &gammar);
  double heel = CalcHeel(Fs, gammas, Fk, gammak);
  if (newheel != nullptr) *newheel = heel;
  double taus = SailTorque(Fs, gammas, deltas, heel);
  double tauk = KeelTorque(Fk, gammak, heel);
  double taur = RudderTorque(Fr, gammar, heel);

  double YFlonBlon = -vc * std::abs(vc) * std::cos(thetac);
  double YFlonAr = Fr * std::cos(gammar) / consts_.rudder.A;
  double YFlonconst = Fs * std::cos(gammas) + Fk * std::cos(gammak);
  Eigen::Matrix<double, 1, 6> YFlon;
  YFlon << YFlonBlon, 0.0, YFlonAr, 0.0, 0.0, YFlonconst;

  if (Flon != nullptr) *Flon = YFlon * beta();
  if (Flat != nullptr) {
    double FBlat = consts_.Blat * vc * std::sin(thetac);
    *Flat =
        (Fs * std::sin(gammas) + Fk * std::sin(gammak) + Fr * std::sin(gammar)) *
        std::cos(heel) + FBlat;
  }

  double YtauBomega = -omega * std::abs(omega);
  double YtauAr = taur / consts_.rudder.A;
  double Ytaurs = Fs * std::sin(gammas) * std::cos(heel);
  double Ytauconst = tauk + (taus - Ytaurs * consts_.rs);
  Eigen::Matrix<double, 1, 6> Ytau;
  Ytau << 0.0, YtauBomega, YtauAr, Ytaurs, 1.0, Ytauconst;

  if (taunet != nullptr) *taunet = Ytau * beta();

  if (Y != nullptr) {
    Y->row(0) = YFlon;
    Y->row(1) = Ytau;
  }
}

bool ControlPhysics::GlobalMaxForceForTorque(double thetaw, double vw,
                                             double thetac, double vc,
                                             double taug, Constraint constraint,
                                             int nsteps, double *deltas,
                                             double *deltar) const {
  CHECK_NOTNULL(deltas);
  CHECK_NOTNULL(deltar);
  double minds, maxds;
  {
    double maxsail = std::abs(util::norm_angle(M_PI - thetaw));
    double minsail = std::max(maxsail - M_PI_2, 0.0);
    maxsail = std::min(maxsail, M_PI_2);
    minds = thetaw > 0.0 ? minsail : -maxsail;
    maxds = thetaw < 0.0 ? -minsail : maxsail;
  }
  double mindr = util::Clip(-consts().rudder.maxalpha - thetac, -0.5, 0.);
  double maxdr = util::Clip(consts().rudder.maxalpha - thetac, 0., 0.5);

  double mincost = std::numeric_limits<double>::infinity();
  bool success = false;
  double deltadeltas = (maxds - minds) / std::max(1, nsteps - 1);

  double taus, tauk, taur, taue, heel;
  for (int ii = 0; ii < nsteps; ++ii) {
    double trialds = minds + (double)ii * deltadeltas;
    double Fs, gammas, Fk, gammak, Fr, gammar;
    SailForce(thetaw, vw, trialds, &Fs, &gammas);
    KeelForce(thetac, vc, &Fk, &gammak);
    heel = CalcHeel(Fs, gammas, Fk, gammak);
    taus = SailTorque(Fs, gammas, trialds, heel);
    tauk = KeelTorque(Fk, gammak, heel);
    double trialdr = RudderForTorque(taus, tauk, taug, heel, thetac, vc);
    switch (constraint) {
      case kQuadratic:
        trialdr = ClipRudder(trialdr, thetac);
        break;
      // For kStarboard and kPort, trialdr may end up outside of the allowable
      // bounds, in which case this particular iteration of the for loop has
      // produced an invalid result.
      case kStarboard:
        trialdr = std::max(trialdr, maxdr);
        break;
      case kPort:
        trialdr = std::min(trialdr, mindr);
        break;
    }

    if (ClipRudder(trialdr, thetac) != trialdr) {
      // Invalid rudder result.
      continue;
    }

    success = true;

    RudderForce(thetac, vc, trialdr, &Fr, &gammar);
    taur = RudderTorque(Fr, gammar, heel);
    double Flon =
        Fs * std::cos(gammas) + Fk * std::cos(gammak) + Fr * std::cos(gammar);

    taue = taus + tauk + taur + consts_.taubias;

    double cost = -Qf * Flon;
    switch (constraint) {
      case kQuadratic:
        cost += Qtaueq * (taue - taug) * (taue - taug);
        break;
      case kStarboard:
        cost += Qtaumax * taue;
        break;
      case kPort:
        cost += -Qtaumax * taue;
        break;
    }

    if (cost < mincost) {
      mincost = cost;
      *deltas = trialds;
      *deltar = trialdr;
    }
  }

  return success;
}

AdaptiveControl::AdaptiveControl()
    : Node(dt),
      sail_msg_(AllocateMessage<msg::SailCmd>()),
      rudder_msg_(AllocateMessage<msg::RudderCmd>()),
      boat_state_(AllocateMessage<msg::BoatState>()),
      consts_msg_(AllocateMessage<msg::ControllerConstants>()),
      heading_(-2.0 * M_PI / 4.0),
      sail_cmd_("sail_cmd", true),
      rudder_cmd_("rudder_cmd", true),
      consts_queue_("control_consts", true) {

    consts_msg_->set_winch_kp(13);
    consts_msg_->set_rudder_kp(20.0);
    consts_msg_->set_qf(1.0);
    {
      std::unique_lock<std::mutex> l(consts_mutex_);
      consts_queue_.send(consts_msg_);
    }

  Kbeta.diagonal() << 0.0, 0.0, 0.0, 0.004, 0.0, 0.;

  RegisterHandler<msg::BoatState>("boat_state", [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> l(boat_state_mutex_);
    *boat_state_ = msg;
    double vx = boat_state_->vel().x();
    double vy = boat_state_->vel().y();
    yaw_ = boat_state_->euler().yaw();
    omega_ = boat_state_->omega().z();
   // double heel = boat_state_->euler().roll();
    thetac_ = -util::norm_angle(std::atan2(vy, vx) - yaw_);
    vc_ = std::sqrt(vx * vx + vy * vy);
  });
  RegisterHandler<msg::Vector3f>("wind", [this](const msg::Vector3f &msg) {
    double wx = msg.x();
    double wy = msg.y();
    thetaw_ = util::norm_angle(std::atan2(-wy, wx));
    // TODO(james): Account for wind gradient properly, rather than by multiplying
    // by a hard-coded constant.
    vw_ = std::sqrt(wy * wy + wx * wx) * 1.6;
  });
  RegisterHandler<msg::HeadingCmd>("heading_cmd", [this](const msg::HeadingCmd &msg) {
    if (msg.has_heading()) {
      heading_ = msg.heading();
    }
  });
  RegisterHandler<msg::ControllerConstants>(
      "control_consts", [this](const msg::ControllerConstants &msg) {
    std::unique_lock<std::mutex> l(consts_mutex_);
    *consts_msg_ = msg;
    physics_.Qf = msg.qf();
  });
  RegisterHandler<msg::SBUS>("sbus_value", [this](const msg::SBUS &sbus) {
    std::unique_lock<std::mutex> l(boat_state_mutex_);
  });
}

ControlPhysics::MatrixBeta AdaptiveControl::Adaptor(double deltas,
                                                    double deltar) const {
  ControlPhysics::MatrixY Y;
  physics_.NetForce(thetaw_, vw_, thetac_, vc_, deltas, deltar, /*omega=*/0.0,
                    nullptr, nullptr, nullptr, nullptr, &Y);
  Eigen::Vector2d sigma = /*qtildedot + Lambda * */ Eigen::Vector2d(
      0.0, util::norm_angle(heading_ - yaw_));
  ControlPhysics::MatrixBeta betadot = -Kbeta * Y.transpose() * sigma;
  return betadot;
}

bool AdaptiveControl::Controller(double *deltas, double *deltar) {
  CHECK_NOTNULL(deltas);
  CHECK_NOTNULL(deltar);
  double taue = consts_msg_->rudder_kp() * util::norm_angle(heading_ - yaw_) -
                0 * 300.0 * omega_;
  ControlPhysics::Constraint constraint = ControlPhysics::kQuadratic;
//  constraint = ControlPhysics::kPort;
//  taue = -100.0;
  if (!physics_.GlobalMaxForceForTorque(thetaw_, vw_, thetac_, vc_, taue,
                                        constraint,
                                        /*nsteps=*/20, deltas, deltar)) {
    return false;
  }
  ControlPhysics::MatrixBeta betadot = Adaptor(*deltas, *deltar);
  physics_.IncrementBeta(betadot * dt);
  return true;
}

void AdaptiveControl::Iterate() {
  std::unique_lock<std::mutex> l(boat_state_mutex_);
  std::unique_lock<std::mutex> lc(consts_mutex_);

  double deltas, deltar;
  if (!Controller(&deltas, &deltar)) {
    deltas = 0.0;
    deltar = 0.0;
  }
  // We can't control sign of deltas ;(
  deltas = std::abs(deltas);

  double cursail = boat_state_->internal().sail();
  double sail_err = util::norm_angle(deltas - cursail);
  sail_msg_->set_voltage(consts_msg_->winch_kp() * sail_err /
                         std::sqrt(std::abs(sail_err)));
  sail_msg_->set_pos(deltas);

  rudder_msg_->set_pos(deltar);

  sail_cmd_.send(sail_msg_);
  rudder_cmd_.send(rudder_msg_);
  consts_queue_.send(consts_msg_);
}

}  // control
}  // sailbot
