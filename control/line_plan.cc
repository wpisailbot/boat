#include "line_plan.h"
#include "control/util.h"
#include "util/proto_util.h"
#include <math.h>
#include <algorithm>
#include "gflags/gflags.h"

namespace sailbot {
namespace control {

constexpr float LinePlan::dt;
constexpr float LinePlan::kTackCost;
constexpr float LinePlan::kTurnCost;
constexpr float LinePlan::kUpwindCost;
constexpr float LinePlan::kUpwindApproxCost;
constexpr float LinePlan::kSailableReach;
constexpr float LinePlan::kGateWidth;
constexpr float LinePlan::kObstacleCost;
constexpr float LinePlan::kPreTurnScale;
constexpr float LinePlan::kAlphaCrossCost;
//constexpr int LinePlan::kMaxLegpts;
constexpr int LinePlan::kMaxTotalpts;
constexpr int LinePlan::kMaxLegs;

DEFINE_string(initial_waypoints, "waypoints.pba",
              "The initial set of waypoints to use when starting up");
DEFINE_string(initial_obstacles, "obstacles.pba",
              "The initial set of waypoints to use when starting up");

LinePlan::LinePlan()
    : Node(dt), lonlat_ref_(0.0, 0.0), lonlat_scale_(0.0, 0.0),
      pathpoints_lonlat_msg_(new msg::WaypointList),//AllocateMessage<msg::WaypointList>()),
      pathpoints_queue_("planner_points", true),
      heading_msg_(AllocateMessage<msg::HeadingCmd>()),
      heading_cmd_("heading_cmd", true),
      obstacles_out_msg_(AllocateMessage<msg::Obstacles>()),
      obstacles_queue_("planner_obstacles", true),
      waypoints_out_msg_(AllocateMessage<msg::WaypointList>()),
      waypoints_queue_("waypoints", true) {
  RegisterHandler<msg::WaypointList>(
      "waypoints",
      [this](const msg::WaypointList &msg) { ReceiveWaypoints(msg); });
  RegisterHandler<msg::Obstacles>(
      "planner_obstacles",
      [this](const msg::Obstacles &msg) { ReceiveObstacles(msg); });

  RegisterHandler<msg::ControlMode>("control_mode",
                                    [this](const msg::ControlMode &msg) {
    if (msg.has_tacker()) {
      tack_mode_ = msg.tacker();
    }
  });


  RegisterHandler<msg::BoatState>(
      "boat_state",
      [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    prev_boat_pos_ = boat_pos_;
    prev_boat_pos_lonlat_ = boat_pos_lonlat_;
    boat_pos_lonlat_ << msg.pos().x(), msg.pos().y();
    boat_pos_ = LonLatToFrame(boat_pos_lonlat_);
    yaw_ = msg.euler().yaw();
    // If our position has changed from the last reference by more than 100m,
    // reset.
    if (util::GPSDistance(msg.pos().y(), msg.pos().x(), lonlat_ref_.y(),
                          lonlat_ref_.x()) > 100.0) {
      ResetRef(boat_pos_lonlat_);
    }
    // Update for if we've crossed a gate.
    UpdateWaypointInc();
  });

  RegisterHandler<msg::Vector3f>("true_wind", [this](const msg::Vector3f &msg) {
    wind_dir_ = std::atan2(msg.y(), msg.x());
  });

  ReadWaypointsAndObstacles();
}

void LinePlan::Iterate() {
  if ((cnt_++ % 10) == 0) {
    obstacles_queue_.send(obstacles_out_msg_);
    waypoints_queue_.send(waypoints_out_msg_);
  }
  LOG(INFO) << "Iterate start";
  if (tack_mode_ != msg::ControlMode_TACKER_LINE_PLAN) {
    pathpoints_lonlat_msg_->clear_points();
    pathpoints_queue_.send(pathpoints_lonlat_msg_);
    return;
  }
  if (lonlat_scale_.squaredNorm() < 1e-10) {
    // If we haven't yet setup our reference points, don't do anything.
    return;
  }
  double heading = GetGoalHeading();
  heading_msg_->set_heading(heading);
  heading_cmd_.send(heading_msg_);
  pathpoints_queue_.send(pathpoints_lonlat_msg_);
  LOG(INFO) << "Iterate end";
}

void LinePlan::ResetRef(const Point &newlonlatref) {
  lonlat_ref_ = newlonlatref;
  double xscale, yscale;
  util::GPSLatLonScale(lonlat_ref_.y(), &yscale, &xscale);
  lonlat_scale_.x() = xscale;
  lonlat_scale_.y() = yscale;

  boat_pos_ = LonLatToFrame(boat_pos_lonlat_);
  prev_boat_pos_ = LonLatToFrame(prev_boat_pos_lonlat_);
  UpdateObstacles();
  UpdateWaypoints();
}

void LinePlan::UpdateObstacles() {
  obstacles_.clear();
  obstacles_.reserve(obstacles_lonlat_.size());
  // Iterate through every obstacle and point and transform:
  for (size_t ii = 0; ii < obstacles_lonlat_.size(); ++ii) {
    obstacles_.push_back(obstacles_lonlat_[ii]);
    for (size_t jj = 0; jj < obstacles_[ii].pts().size(); ++jj) {
      obstacles_[ii].mutable_pts()->at(jj) =
          LonLatToFrame(obstacles_[ii].pt(jj));
    }
    obstacles_[ii].ProcessPts();
  }
}

void LinePlan::UpdateWaypoints() {
  // Iterate through every waypoint and convert appropriately.
  size_t Nway = waypoints_lonlat_.size();
  if (Nway == 1) {
    waypoints_.clear();
    Point pt = LonLatToFrame(waypoints_lonlat_[0]);
    waypoints_.push_back({pt, pt});
    next_waypoint_ = 0;
    return;
  }
  if (Nway <= 1) return;
  waypoints_.resize(Nway);
  // None of these are true "gates".
  are_gates_.clear();
  are_gates_.resize(Nway, false);

  Point curpt = LonLatToFrame(waypoints_lonlat_[0]);
  waypoints_[0] = std::make_pair(curpt, curpt); // Create sane first waypoint.
  Point nextpt = LonLatToFrame(waypoints_lonlat_[1]);

  for (size_t ii = 1; ii < Nway-1; ++ii) {
    Point prevpt = curpt;
    curpt = nextpt;
    nextpt = LonLatToFrame(waypoints_lonlat_[ii+1]);
    waypoints_[ii] = MakeGateFromWaypoints(curpt, prevpt, nextpt);
  }
  if (repeat_waypoints_) {
    Point prevpt = curpt;
    curpt = nextpt;
    nextpt = LonLatToFrame(waypoints_lonlat_[0]);
    waypoints_[Nway-1] = MakeGateFromWaypoints(curpt, prevpt, nextpt);

    prevpt = curpt;
    curpt = nextpt;
    nextpt = LonLatToFrame(waypoints_lonlat_[1]);
    waypoints_[0] = MakeGateFromWaypoints(curpt, prevpt, nextpt);
  } else {
    // Make final waypoint a point rather than gate.
    waypoints_[Nway-1] = std::make_pair(nextpt, nextpt);
  }
}

// Produces a gate at the waypoint gatept given previous and next waypoints.
// Assumes we are going around a convex polygon, so will place gate on outside
// of the (prevpt, gatept, nextpt) segments.
// static
std::pair<Point, Point> LinePlan::MakeGateFromWaypoints(const Point &gatept,
                                                        const Point &prevpt,
                                                        const Point &nextpt) {
  Point preleg = gatept - prevpt;
  Point postleg = nextpt - gatept;
  double prev2gatedist = preleg.norm();
  double next2gatedist = postleg.norm();
  // Minimum distance before we decide that things are close enough to be
  // treated as the same. Primarily here to avoid numerical instability.
  double mindist = 1e-6;
  Point gateseg;
  bool centered = false;
  if (prev2gatedist < mindist && next2gatedist < mindist) {
    // The points are all on top of each other, just give a horizontal gate
    Point offset(kGateWidth / 2.0, 0.0);
    return {gatept - offset, gatept + offset};
  } else if (prev2gatedist < mindist) {
    // For these two, center the gate and use a gate perendicular to
    // the one valid segment.
    centered = true;
    postleg /= next2gatedist;
    gateseg << -postleg.y(), postleg.x();
  } else if (next2gatedist < mindist) {
    centered = true;
    preleg /= prev2gatedist;
    gateseg << -preleg.y(), preleg.x();
  } else {
    centered = false; // Strictly unnecessary
    preleg /= prev2gatedist;
    postleg /= next2gatedist;
    gateseg = preleg - postleg;
    double gatenorm = gateseg.norm();
    if (gatenorm < mindist) {
      // In this case, preleg == postleg, and so we are going
      // straight; treat same as centered scenarios.
      centered = true;
      gateseg << -preleg.y(), preleg.x();
      // preleg is already normalized
    } else {
      gateseg /= gatenorm;
    }
  }
  gateseg *= kGateWidth;

  Point gatestart = gatept;
  Point gateend = gatept + gateseg;
  if (centered) {
    gatestart -= gateseg / 2.0;
    gateend -= gateseg / 2.0;
  }
  return {gatestart, gateend};
}

void LinePlan::UpdateWaypointInc() {
  if (waypoints_.size() == 0) {
    return;
  }
  if (!repeat_waypoints_ && next_waypoint_ == waypoints_.size() - 1) {
    // If we are already on the last waypoint, don't do anything.
    return;
  }
  // For now, just update if we have crossed sides of the gate line AND both
  // prev pos and current pos project to the gate.
  const Vector2d &gate_start = waypoints_[next_waypoint_].first;
  const Vector2d &gate_end = waypoints_[next_waypoint_].second;

  bool passed_line = false;
  if (are_gates_[next_waypoint_]) {
    passed_line = ProjectsToLine(gate_start, gate_end, prev_boat_pos_) &&
                  ProjectsToLine(gate_start, gate_end, boat_pos_);
  } else {
    if ((gate_start - gate_end).norm() < 1e-6) {
      // If the "gate" is really a point, just pass if we go within 5 meters.
      passed_line = (boat_pos_ - gate_start).norm() < 5;
    } else {
      passed_line = ProjectsToRay(gate_start, gate_end, prev_boat_pos_) &&
                    ProjectsToRay(gate_start, gate_end, boat_pos_);
    }
  }
  if (passed_line) {
    // Check that distances are different signs:
    double d0 = DistToLine(gate_start, gate_end, prev_boat_pos_);
    double d1 = DistToLine(gate_start, gate_end, boat_pos_);
    if (d0 * d1 <= 0) {
      ++next_waypoint_;
      // For if we are repeating
      next_waypoint_ = next_waypoint_ % waypoints_.size();
    }
  }
}

double LinePlan::GetGoalHeading() {
  // TODO(james): Do more than just return heading; also need to be able to tell
  // when we've hit a tack point and go with it.
  std::unique_lock<std::mutex> lck(data_mutex_);
  if (waypoints_.size() < 1) {
    return yaw_;
  }
  int future_idx = next_waypoint_ + 1;
  future_idx = repeat_waypoints_
                   ? future_idx % waypoints_.size()
                   : std::min((int)(waypoints_.size() - 1), future_idx);
  std::vector<std::vector<Vector2d>> paths;
  std::vector<double> alphas;
  Vector2d nextpt =
      0.5 * (waypoints_[future_idx].first + waypoints_[future_idx].second);
  int Nlegs = std::max(
      1, repeat_waypoints_
             ? kMaxLegs
             : std::min(kMaxLegs, (int)waypoints_.size() - next_waypoint_));
  std::vector<std::pair<Point, Point>> plan_for_gates;
  for (int ii = 0; ii < Nlegs; ++ii) {
    plan_for_gates.push_back(
        waypoints_[(next_waypoint_ + ii) % waypoints_.size()]);
  }
  FindMultiplePath(boat_pos_, plan_for_gates, nextpt, wind_dir_, obstacles_,
           yaw_, &paths, &alphas);

  std::vector<Point> path;
  for (int ii = 0; ii < paths.size(); ++ii) {
    auto &leg = paths[ii];
    CHECK_LE(1.0, leg.size())
        << "The path from FindPath MUST always contain at "
           "least one point (our current position)";

    double alpha = alphas[ii];

    path.insert(path.end(), leg.begin(), leg.end());

    // Because the returned path doesn't include the last point, add it.
    path.push_back((1.0 - alpha) * plan_for_gates[ii].first +
                   alpha * plan_for_gates[ii].second);
    pathpoints_lonlat_msg_->clear_points();
    for (const auto &pt : path) {
      Point ptll = FrameToLonLat(pt);
      msg::Waypoint *msg_pt = pathpoints_lonlat_msg_->add_points();
      msg_pt->set_x(ptll.x());
      msg_pt->set_y(ptll.y());
    }
  }

  double heading = util::atan2(path[1] - path[0]);
  return heading;
}

void LinePlan::FindPath(const Vector2d &startpt,
                        const std::pair<Vector2d, Vector2d> &gate,
                        const Vector2d &nextpt, double winddir,
                        const std::vector<Polygon> &obstacles, double cur_yaw,
                        std::vector<Vector2d> *tackpts, double *alpha) {
  CHECK_NOTNULL(tackpts);
  CHECK_NOTNULL(alpha);
  std::vector<double> alphas;
  std::vector<std::vector<Vector2d>> tacks;
  FindMultiplePath(startpt, {gate}, nextpt, winddir, obstacles, cur_yaw, &tacks,
                   &alphas);
  *alpha = alphas[0];
  *tackpts = tacks[0];
#if 1
#else
  CHECK_NOTNULL(tackpts);
  CHECK_NOTNULL(alpha);
  // Lowest cost thus far
  double lowest_cost = std::numeric_limits<double>::infinity();
  double dcost = -lowest_cost;
  // For initial endpt, use halfway along gate:
  Vector2d nominal_endpt = 0.5 * (gate.first + gate.second);

  *tackpts = {startpt};

  int Npts = 0;

  while (Npts < kMaxNpts) {// && dcost < -1e-2) {
    std::vector<std::vector<Vector2d>> paths;
    GenerateHypotheses(startpt, nominal_endpt, winddir, Npts, &paths);
    double nptscost = lowest_cost;
    bool any_viable = false;

    for (std::vector<Vector2d> &path : paths) {
      double iteralpha, itercost;
      bool viable;
      OptimizeTacks(gate, nextpt, winddir, obstacles, cur_yaw, &path,
                    &iteralpha, &itercost, &viable);
      VLOG(1) << "alpha: " << iteralpha << " viable: " << viable
              << " cost: " << itercost << " path: ";
      for (const auto &pt : path) {
        VLOG(1) << pt.transpose();
      }
      any_viable = viable || any_viable;
      // Use this path if it is both the cheapest and isn't disqualified
      // for some reason (e.g., upwind legs).
      if (viable && itercost < nptscost) {
        nptscost = itercost;
        *tackpts = path;
        *alpha = iteralpha;
      }
    }

    VLOG(1) << "cost: " << lowest_cost;
    //LOG(INFO) << "cost: " << lowest_cost;

    if (any_viable) {
      // Will be zero if no improvements were found
      dcost = nptscost - lowest_cost;
      if (dcost < 0) {
        lowest_cost = nptscost;
      }
    }

    ++Npts;
  }
#endif
}

void LinePlan::FindMultiplePath(
    const Vector2d &startpt,
    const std::vector<std::pair<Vector2d, Vector2d>> &gates,
    const Vector2d &nextpt, double winddir,
    const std::vector<Polygon> &obstacles, double cur_yaw,
    std::vector<std::vector<Vector2d>> *tacks, std::vector<double> *alphas) {
  CHECK_NOTNULL(tacks);
  CHECK_NOTNULL(alphas);
  const int Nlegs = gates.size();
  const int MaxPtsPerLeg = std::max(1, kMaxTotalpts + 1 - Nlegs);
  tacks->resize(Nlegs, {});
  alphas->resize(Nlegs, 0.5);
  // Lowest cost thus far
  // For initial endpt, use halfway along gate:
  std::vector<Vector2d> nominal_endpts;
  for (int ii = 0; ii < Nlegs; ++ii) {
    const auto &gate = gates[ii];
    nominal_endpts.push_back(0.5 * (gate.first + gate.second));
    tacks->at(ii).push_back(nominal_endpts.back());
  }
  // Vector of hypotheses:
  // Mth point along Nth hypothesis for Pth leg is
  // hypothesis[N][P][M]
  std::vector<std::vector<std::vector<Vector2d>>> hypotheses;
  std::vector<int> hypothesis_lengths;


  for (int ii = 0; ii < Nlegs; ++ii) {
    Vector2d legstart = ii == 0 ? startpt : nominal_endpts[ii - 1];
    std::vector<std::vector<Point>> hypos;
    for (int jj = 0; jj < MaxPtsPerLeg; ++jj) {
      GenerateHypotheses(legstart, nominal_endpts[ii], winddir, jj, &hypos,
                         /*clear_paths=*/false);
    }
    VLOG(1) << "Nleg: " << ii << " nypos " << hypos.size();
    int Nprevhyp = hypotheses.size();
    std::vector<int> old_hypothesis_lengths = hypothesis_lengths;
    for (int newhyp = 0; newhyp < hypos.size(); ++newhyp) {
      int hypothesis_pts = hypos[newhyp].size();
      for (int prevhyp = 0; prevhyp < Nprevhyp; ++prevhyp) {
        if (old_hypothesis_lengths[prevhyp] + hypothesis_pts >
            MaxPtsPerLeg + ii) {
          // If adding the new hypothesis onto the end would increase
          // the total length above that which is allowed, don't do it (accounting
          // for that we need 1 point for every leg).
          continue;
        }
        int idx = prevhyp;
        if (newhyp > 0) {
          // Only need to copy data past the first hypothesis we go through
          // Subtract one from end because we will have added on extra
          // hypotheses at this point.
          hypotheses.emplace_back(hypotheses[prevhyp].begin(),
                                  hypotheses[prevhyp].end() - 1);
          hypothesis_lengths.push_back(old_hypothesis_lengths[prevhyp]);
          idx = hypotheses.size() - 1;
        }
        hypotheses[idx].push_back(hypos[newhyp]);
        hypothesis_lengths[idx] += hypothesis_pts;
      }
      if (Nprevhyp == 0) {
        // We are on the first leg and so need to create things from scratch
        hypotheses.push_back({hypos[newhyp]});
        hypothesis_lengths.push_back(hypothesis_pts);
      }
    }
  }

  *tacks = hypotheses[0];

  double lowest_cost = std::numeric_limits<double>::infinity();
  int cnt = 0;
  for (auto &hypothesis : hypotheses) {
    VLOG(1) << "===WORKING IN " << cnt++ << "th HYPOTHESIS===";
    std::vector<double> iter_alphas;
    iter_alphas.resize(Nlegs, 0.5);
    double itercost;
    bool viable;
    OptimizeMultipleTacks(gates, nextpt, winddir, obstacles, cur_yaw,
                          &hypothesis, &iter_alphas, &itercost, &viable);
    //      viable = true; // XXX TODO(james): Fix
    // Use this path if it is both the cheapest and isn't disqualified
    // for some reason (e.g., upwind legs).
    VLOG(1) << "cost: " << itercost << " viable: " << viable;
    if (viable && itercost < lowest_cost) {
      lowest_cost = itercost;
      *tacks = hypothesis;
      *alphas = iter_alphas;
      VLOG(1) << "Replacing lowest cost";
    }
  }
}

void
LinePlan::OptimizeTacks(const std::pair<Eigen::Vector2d, Eigen::Vector2d> &gate,
                        const Eigen::Vector2d &nextpt, double winddir,
                        const std::vector<Polygon> &obstacles, double cur_yaw,
                        std::vector<Eigen::Vector2d> *tackpts, double *alpha,
                        double *finalcost, bool *viable) {
  CHECK_NOTNULL(tackpts);
  CHECK_NOTNULL(alpha);
  std::vector<std::vector<Point>> tacks = {*tackpts};
  std::vector<double> alphas = {*alpha};
  OptimizeMultipleTacks({gate}, nextpt, winddir, obstacles, cur_yaw, &tacks,
                        &alphas, finalcost, viable);
  *tackpts = tacks[0];
  *alpha = alphas[0];
}

// Optimize for multiple consecutive paths at once:
void LinePlan::OptimizeMultipleTacks(
    const std::vector<std::pair<Vector2d, Vector2d>> &gates,
    const Vector2d &nextpt, double winddir,
    const std::vector<Polygon> &obstacles, double cur_yaw,
    std::vector<std::vector<Vector2d>> *tacks, std::vector<double> *alphas,
    double *finalcost, bool *viable) {
  CHECK_NOTNULL(tacks);
  CHECK_NOTNULL(alphas);
  const int Nlegs = tacks->size();
  CHECK_EQ(Nlegs, alphas->size())
      << "We should have the same number of points as alphas";
  CHECK_EQ(Nlegs, gates.size())
      << "We should have the same number of points as gates";
  for (int ii = 0; ii < Nlegs; ++ii) {
    CHECK(!tacks->at(ii).empty())
        << "You should have at least one element in each path segment";
  }
  double step = 0.01;

  Eigen::Vector2d npt;
  for (int ii = 0; ii < 50 - 0; ++ii) {
    if (ii > 20) {
      step = 3.0e-3;
    }
    if (finalcost != nullptr) *finalcost = 0;
    if (viable != nullptr) *viable = true;
    for (int jj = 0; jj < Nlegs; ++jj) {
      VLOG(1) << "===BackPass ON " << jj << "th LEG in " << ii
              << "th ITERATION===";
      const std::pair<Vector2d, Vector2d> &gate = gates[jj];
      npt = jj == Nlegs - 1 ? nextpt : tacks->at(jj + 1)[0];
      double cyaw = cur_yaw;
      if (jj > 0) {
        tacks->at(jj)[0] =
            gate.first + alphas->at(jj - 1) * (gate.second - gate.first);
        cyaw = util::atan2(tacks->at(jj)[0] - tacks->at(jj - 1).back());
      }
      double segmentcost;
      bool segmentviable;
      BackPass(gate, npt, winddir, obstacles, cyaw, step, /*prescale=*/jj == 0,
               &tacks->at(jj), &alphas->at(jj), &segmentcost, &segmentviable);
      VLOG(1) << "BackPass cost result: " << segmentcost;
      if (finalcost != nullptr) *finalcost += segmentcost;
      // TODO(james): Consider only enforcing viability on first tack
      if (jj == -1 && viable != nullptr)
        *viable = *viable && segmentviable;
    }
  }
}

void LinePlan::GenerateHypotheses(const Vector2d &startpt,
                                  const Vector2d &endpt, double winddir,
                                  int Npts,
                                  std::vector<std::vector<Vector2d>> *paths,
                                  bool clear_paths) {
  CHECK_NOTNULL(paths);
  CHECK_LE(0, Npts);
  if (clear_paths) {
    paths->clear();
  }
  if (Npts == 0) {
    paths->push_back({startpt});
    return;
  }
  // Minimum relative wind to have to not need to tack, radians
  const double kMinWind = 1.0;
  Vector2d diff = endpt - startpt;

  // 0 if upwind, +pi / 2 if starboard tack (i.e., wind from starboard bow)
  double relwind =
      util::norm_angle(M_PI - winddir + util::atan2(diff));

  paths->push_back({startpt});
  for (int ii = 0; ii < Npts; ++ii) {
    double alpha = (ii + 0.5) / (double)Npts;
    paths->back().push_back(startpt + alpha * diff);
  }
  // In upwind/close reaches, we are also going to tack a bit.
  // We construct these by presuming that each tack (except the first and last)
  // will traverse a constant distance perpendicular to the straight lien
  // path. The first and last legs traverse half the lateral distance.
  if (std::abs(relwind) < 1.0) {
    // Amount we have to turn through a tack.
    double alpha = kMinWind * 2.0;
    // Angle of initial leg relative to straight:
    double theta = kMinWind - relwind;
    // Length along crow-line of a pair of tacks, assuming diff.norm == 1:
    double l = 2.0 / (double)Npts;
    // Length of one of the two legs, normalize to l=1:
    double legdist = l * std::sin(alpha - theta) / std::sin(alpha);
    // Lateral distance traversed by any single leg.
    double latdist = legdist * std::sin(theta);
    double londist1 = legdist * std::cos(theta);
    double londist2 = l - londist1;

    paths->push_back({startpt});
    paths->push_back({startpt});
    int idx1 = paths->size() - 2, idx2 = paths->size() - 1;

    for (int ii = 0; ii < Npts; ++ii) {
      double dominantmult = 0.5 + std::floor(ii / 2.0);
      double recessivemult = std::floor((ii + 1.0) / 2.0);
      double lon1 = londist1 * dominantmult + londist2 * recessivemult;
      double lon2 = londist2 * dominantmult + londist1 * recessivemult;

      double latoffset1 = latdist * ((ii % 2) == 0 ? 1.0 : -1.0) / 2.0;
      double latoffset2 = -latoffset1;

      Vector2d perp(-diff.y(), diff.x());

      paths->at(idx1).push_back(startpt + diff * lon1 + perp * latoffset1);
      paths->at(idx2).push_back(startpt + diff * lon2 + perp * latoffset2);
    }
  }
}

/**
 * Perform a single backwards pass where we incrementally
 * improve, using gradient descent, each parameter, starting
 * with alpha and stepping back along tackpts.
 * step adjusts the magnitude of the gradient descent.
 * tackpts and alpha should already be valid; tackpts
 * must be at least length 1; the first element will not
 * be altered, as it is assumed to be our current position.
 *
 * The circumstance is that we are sailing from startpt,
 * via tackpts, to the point that is proportion alpha
 * along gate (e.g., alpha=0.5 corresponds with splitting
 * the gate). nextpt is the point we will be heading to
 * afterwards.
 */
void LinePlan::BackPass(const std::pair<Eigen::Vector2d, Eigen::Vector2d> &gate,
                        const Eigen::Vector2d &nextpt, double winddir,
                        const std::vector<Polygon> &obstacles, double cur_yaw,
                        double step, bool prescale,
                        std::vector<Eigen::Vector2d> *tackpts, double *alpha,
                        double *finalcost, bool *viable) {
  CHECK_NOTNULL(alpha);
  CHECK_NOTNULL(tackpts);
  CHECK_LE(1, tackpts->size()) << "tackpts should have at least one element";
  if (finalcost != nullptr) *finalcost = 0.0;
  if (viable != nullptr) *viable = true;

  VLOG(1) << "===STARTING BackPass===";

  double cost;

  size_t Npts = tackpts->size();

  double penultimate_heading =
      Npts > 1 ? util::atan2(tackpts->at(Npts - 1) - tackpts->at(Npts - 2))
               : cur_yaw;

  double dcostdalpha;
  bool lineviable;
  VLOG(1) << "===IMPROVING alpha===";
  SingleLineCost(gate.first, gate.second, tackpts->at(Npts - 1), *alpha,
                 penultimate_heading, nextpt, winddir, obstacles, &cost,
                 &dcostdalpha, &lineviable);
  VLOG(1) << "alpha cost: " << cost;
  if (viable != nullptr) *viable = *viable && lineviable;
  *alpha -= step * dcostdalpha;
  // TODO(james): Clip alpha to 1 programmatically based on are_gates_.
  *alpha = util::Clip(*alpha, 0.05, 1.0);
  if (finalcost != nullptr) {
    // Strictly speaking calculates cost of previous iteration, but this avoids
    // an instability if dcostdalpha is very high.
    *finalcost += cost;
  }

  // The point following the one we are currently optimizing.
  Vector2d postpt = gate.first + *alpha * (gate.second - gate.first);
  double postheading = util::atan2(postpt - tackpts->at(Npts - 1));
  for (int ii = Npts - 1; ii >= 1; --ii) {
    VLOG(1) << "===IMPROVING " << ii << "th PONT===";
    // The point before the one that we are currently optimizing.
    const Vector2d &prept = tackpts->at(ii - 1);
    // The point that we are optimizing.
    const Vector2d &curpt = tackpts->at(ii);
    // Heading of the boat coming into prept.
    double preheading =
        (ii == 1) ? cur_yaw : util::atan2(prept - tackpts->at(ii - 2));
    double scale_pre = (ii == 1 && prescale) ? kPreTurnScale : 1.0;

    Vector2d dcostdpt;
    LinePairCost(prept, postpt, curpt, preheading, postheading, winddir,
                 scale_pre, obstacles, &cost, &dcostdpt, &lineviable);
    if (viable != nullptr) *viable = *viable && lineviable;
    tackpts->at(ii) -= step * dcostdpt;
    //LOG(INFO) << "pt: " << ii << " dcostdpt: " << dcostdpt.transpose();
    if (finalcost != nullptr) {
      // Strictly speaking calculates cost of previous iteration, but this
      // avoids
      // an instability if dcostdpt is very high.
      *finalcost += cost;
    }
    VLOG(1) << "line pair cost: " << cost;

    postheading = util::atan2(postpt - curpt);
    postpt = curpt;
  }
  VLOG(1) << "===FINISHED WITH BackPass===";
}

/**
 * Computes the cost of a single line path, where the the path starts at startpt,
 * and terminates alpha proportion along the line from startline to endline, e.g.:
 *                         endline *   * nextpt
 *                                 |  /
 *                   ^^^ winddir   | /<--Future boat path
 *                                 |/
 * startpt *-----------------------*--
 *              ^                  | |_startline + alpha * (endline - startline)
 *              |                  | |
 *          boat path    startline *--
 *
 * Does not account for any obstacles.
 *
 * Arguments:
 *   startline: The start of the line along which the path terminates
 *   endline: The end of the line along which the path terminates
 *   startpt: The start of the boat path we are considering (possibly
 *       the current boat position)
 *   alpha: scalar, 0-1, representing how far along the terminating
 *       line the boat path terminates
 *   preheading: The heading coming into startpt, possibly the current boat
 *       yaw.
 *   nextpt: The next point which the boat will be travelling to,
 *       so that we can account for the cost of the turn that will
 *       occur after we cross the line
 *   winddir: The direction of the prevailing wind. Represents
 *       the direction that the wind is blowing, so will be 0
 *       when the wind is blowing in the +x direction, pi/2 when
 *       blowing in the +y direction, etc.
 *   cost: The cost of this configuration, accounting for the length/tack
 *       of the line, the turn required to turn to the heading
 *       nextpt, the length/tack of the line to nextpt, the
 *       point at which we cross the line. The cost corresponding
 *       to where we cross the line is determined by CrossFinishCost.
 *   dcostdalpha: Derivative of the cost w.r.t. alpha.
 */
void LinePlan::SingleLineCost(const Eigen::Vector2d &startline,
                              const Eigen::Vector2d &endline,
                              const Eigen::Vector2d &startpt, double alpha,
                              double preheading, const Eigen::Vector2d &nextpt,
                              double winddir, double *cost, double *dcostdalpha,
                              bool *viable) {
  // endpt is the point along the finish line that we are
  // crossing.
  Vector2d endpt = startline + alpha * (endline - startline);
  Vector2d dendda = endline - startline;
  // firstleg is a vector of the leg from startpt to the finish line.
  Vector2d firstleg = endpt - startpt;
  Vector2d dfirstlegda = dendda;
  // nextleg is the leg from endpt to nextpt.
  Vector2d nextleg = nextpt - endpt;
  Vector2d dnextlegda = -dendda;
  VLOG(1) << "alpha: " << alpha;

  auto datan = [](const Vector2d &x, const Vector2d &dxda) -> double {
    double sqNorm = x.squaredNorm();
    Vector2d d(-x.y(), x.x());
    d /= sqNorm;
    return d.transpose() * dxda;
  };

  double startheading = std::atan2(firstleg.y(), firstleg.x());
  double nextheading = std::atan2(nextleg.y(), nextleg.x());
  double dstarthda = datan(firstleg, dfirstlegda);
  double dnexthda = datan(nextleg, dnextlegda);
  if (nextleg.squaredNorm() < 1e-8) {
    // Make it so that in the special case where nextleg is 0 we will ignore it
    // as much as possible.
    nextheading = startheading;
    dnexthda = dstarthda;
  }

  auto dnorm = [](Vector2d x, const Vector2d &dxda) {
    x.normalize();
    return x.transpose() * dxda;
  };

  double firstlen = firstleg.norm();
  double dfirstlenda = dnorm(firstleg, dfirstlegda);
  double nextlen = nextleg.norm();
  double dnextlenda = dnorm(nextleg, dnextlegda);

  double firstlencost, dflencostdlen, dflencostdheading;
  StraightLineCost(firstlen, startheading, winddir, /*is_real=*/true,
                   &firstlencost, &dflencostdlen, &dflencostdheading, viable);
  // We don't care about the strict viability of the future upwind leg,
  // as we will be breaking it down later and so can live with it
  // pointing straight upwind.
  double nextlencost = 0, dnlencostdlen = 0, dnlencostdheading = 0;
  StraightLineCost(nextlen, nextheading, winddir, /*is_real=*/false,
                   &nextlencost, &dnlencostdlen, &dnlencostdheading, nullptr);
  VLOG(1) << "dfirstcostdlen: " << dflencostdlen
          << " dfirstlenda: " << dfirstlenda
          << " dfirstcostdhead: " << dflencostdheading
          << " dstarthda: " << dstarthda;
  VLOG(1) << "dnextcostdlen: " << dnlencostdlen << " dnextlenda: " << dnextlenda
          << " dnextcostdhead: " << dnlencostdheading
          << " dnexthda: " << dnexthda;

  double lencost = firstlencost + nextlencost;
  double dlencostda = dflencostdlen * dfirstlenda +
                      dflencostdheading * dstarthda +
                      dnlencostdlen * dnextlenda + dnlencostdheading * dnexthda;

  double preturncost, dpreturndstart, dpreturndend;
  TurnCost(preheading, startheading, winddir, &preturncost, &dpreturndstart,
           &dpreturndend);
  double dpreturncostda = dpreturndend * dstarthda;

  double turncost, dturndstart, dturndend;
  TurnCost(startheading, nextheading, winddir, &turncost, &dturndstart,
           &dturndend);
  double dturncostda = dturndstart * dstarthda + dturndend * dnexthda;

  VLOG(1) << "preturncost: " << preturncost << " dpreturnda: " << dpreturncostda
          << " midturncost: " << turncost << " dturncostda: " << dturncostda;

  turncost += preturncost;
  dturncostda += dpreturncostda;

  double crosscost, dcrossda;
  CrossFinishCost(alpha, &crosscost, &dcrossda);

  VLOG(1) << "lencost: " << lencost << " turncost: " << turncost
          << " crosscost: " << crosscost;
  VLOG(1) << "dlencost: " << dlencostda << " dturncost: " << dturncostda
          << " dcrosscost: " << dcrossda;
  *CHECK_NOTNULL(cost) = lencost + turncost + crosscost;
  *CHECK_NOTNULL(dcostdalpha) = dlencostda + dturncostda + dcrossda;

  if (std::isnan(*dcostdalpha)) {
    *dcostdalpha = 0.0;
  }
}

/**
 * Same as above, but with obstacle costs (obstacle costs are computed
 * numerically).
 */
void LinePlan::SingleLineCost(const Eigen::Vector2d &startline,
                              const Eigen::Vector2d &endline,
                              const Eigen::Vector2d &startpt, double alpha,
                              double preheading, const Eigen::Vector2d &nextpt,
                              double winddir,
                              const std::vector<Polygon> &obstacles,
                              double *cost, double *dcostdalpha, bool *viable) {
  *CHECK_NOTNULL(cost) = 0.0;
  *CHECK_NOTNULL(dcostdalpha) = 0.0;

  double basecost = 0.0, basedalpha = 0.0;
  SingleLineCost(startline, endline, startpt, alpha, preheading, nextpt,
                 winddir, &basecost, &basedalpha, viable);

  Vector2d crosspt = startline + alpha * (endline - startline);
  Vector2d dcrossptdalpha = endline - startline;
  double obstaclecost = 0.0;
  Vector2d obstacledend;
  ObstacleCost(crosspt, startpt, obstacles, &obstaclecost, &obstacledend);
  VLOG(1) << "dobstacledend: " << obstacledend.transpose()
          << " dcrossptdalpha: " << dcrossptdalpha.transpose();
  double obstacledalpha = obstacledend.dot(dcrossptdalpha);

  *cost = basecost + obstaclecost;
  *dcostdalpha = basedalpha + obstacledalpha;
}

/**
 * Computes the cost of a pair of lines connecting startpt to endpt
 * via turnpt. Assumes that the heading coming into startpt is preheading and
 * the heading coming out of endpt is postheading, for computation of the
 * turning costs.
 * The returned cost will include components for the two line segments
 * themselves, and the turning costs at each of the three points. Also returns
 * the partial derivatives w.r.t. midpt.
 */
void LinePlan::LinePairCost(const Eigen::Vector2d &startpt,
                            const Eigen::Vector2d &endpt,
                            const Eigen::Vector2d &turnpt, double preheading,
                            double postheading, double winddir,
                            double scale_pre_cost, double *cost,
                            Eigen::Vector2d *dcostdturnpt, bool *viable) {
  *CHECK_NOTNULL(cost) = 0.0;
  *CHECK_NOTNULL(dcostdturnpt) *= 0.0;
  if (viable != nullptr) *viable = true;
  Vector2d sega = turnpt - startpt;
  Vector2d segb = endpt - turnpt;
  VLOG(1) << "LinePairCost";

  double lena = sega.norm();
  Vector2d dlenadturnpt = sega / sega.norm();

  double lenb = segb.norm();
  // Negative because dsegb / dturn = -1
  Vector2d dlenbdturnpt = -segb / segb.norm();

  double headinga = std::atan2(sega.y(), sega.x());
  Vector2d dhadturnpt =
      Vector2d(-sega.y(), sega.x()) / sega.squaredNorm();
  double headingb = std::atan2(segb.y(), segb.x());
  Vector2d dhbdturnpt =
      -Vector2d(-segb.y(), segb.x()) / segb.squaredNorm();

  // Calculate costs for all 3 turns:
  double turncost, dcostdstart, dcostdend;
  TurnCost(preheading, headinga, winddir, &turncost, &dcostdstart, &dcostdend);
  VLOG(1) << "Preturn: cost (before pre-scale) " << turncost << " dcostdend "
          << dcostdend << " dhadpt " << dhadturnpt;
  *cost += turncost * scale_pre_cost;
  *dcostdturnpt += dcostdend * dhadturnpt * scale_pre_cost;

  TurnCost(headinga, headingb, winddir, &turncost, &dcostdstart, &dcostdend);
  VLOG(1) << "Middle Turn: cost " << turncost << " dcostdstart " << dcostdstart
          << " dhadpt " << dhadturnpt << " dcostdend " << dcostdend
          << " dhbdpt " << dhbdturnpt;
  *cost += turncost;
  *dcostdturnpt += dcostdstart * dhadturnpt;
  *dcostdturnpt += dcostdend * dhbdturnpt;

  TurnCost(headingb, postheading, winddir, &turncost, &dcostdstart, &dcostdend);
  VLOG(1) << "Postturn: cost " << turncost << " dcostdstart " << dcostdstart
          << " dhbdpt " << dhbdturnpt;
  *cost += turncost;
  *dcostdturnpt += dcostdstart * dhbdturnpt;

  // Calculate costs for the two legs:
  double linecost, dcostdlen, dcostdheading;
  bool lineviable;
  StraightLineCost(lena, headinga, winddir, /*is_real=*/true, &linecost,
                   &dcostdlen, &dcostdheading, &lineviable);
  if (viable != nullptr) *viable = lineviable && *viable;
  VLOG(1) << "Preline diff: dcostdlen: " << dcostdlen
          << " dlenadpt: " << dlenadturnpt
          << " dcostdheading: " << dcostdheading
          << " dheadingdpt: " << dhadturnpt;
  *cost += linecost;
  *dcostdturnpt += dcostdlen * dlenadturnpt + dcostdheading * dhadturnpt;

  StraightLineCost(lenb, headingb, winddir, /*is_real=*/true, &linecost,
                   &dcostdlen, &dcostdheading, &lineviable);
  if (viable != nullptr) *viable = lineviable && *viable;
  VLOG(1) << "Postline diff: dcostdlen: " << dcostdlen
          << " dlenbdpt: " << dlenbdturnpt
          << " dcostdheading: " << dcostdheading
          << " dheadingdpt: " << dhbdturnpt;
  *cost += linecost;
  *dcostdturnpt += dcostdlen * dlenbdturnpt + dcostdheading * dhbdturnpt;
}

void LinePlan::LinePairCost(const Eigen::Vector2d &startpt,
                            const Eigen::Vector2d &endpt,
                            const Eigen::Vector2d &turnpt, double preheading,
                            double postheading, double winddir,
                            double scale_pre_cost,
                            const std::vector<Polygon> &obstacles, double *cost,
                            Eigen::Vector2d *dcostdturnpt, bool *viable) {
  *CHECK_NOTNULL(cost) = 0.0;
  *CHECK_NOTNULL(dcostdturnpt) *= 0.0;

  double basecost;
  Vector2d basedturnpt;
  LinePairCost(startpt, endpt, turnpt, preheading, postheading, winddir,
               scale_pre_cost, &basecost, &basedturnpt, viable);
  *cost += basecost;
  *dcostdturnpt += basedturnpt;

  double obstaclecost;
  Vector2d obstacledturnpt;

  ObstacleCost(turnpt, startpt, obstacles, &obstaclecost, &obstacledturnpt);
  *cost += obstaclecost;
  *dcostdturnpt += obstacledturnpt;

  ObstacleCost(turnpt, endpt, obstacles, &obstaclecost, &obstacledturnpt);
  *cost += obstaclecost;
  *dcostdturnpt += obstacledturnpt;
}

/**
 * The cost of crossing a line. Generally alpha=0 would be at a buoy,
 * and alpha=1 would be the farthest we want to cross from the buoy.
 *
 * For now, uses quadratic function of alpha, with the minimum at
 * alpha = 0.5.
 *
 * Returns costs on a scale from zero to one.
 */
void LinePlan::CrossFinishCost(double alpha,
                               double *cost, double *dcostdalpha) {
//  const double alpham5 = alpha - 0.5;
//  *CHECK_NOTNULL(cost) = kAlphaCrossCost * alpham5 * alpham5;
//  *CHECK_NOTNULL(dcostdalpha) = 2.0 * kAlphaCrossCost * alpham5;
  // Protect against negative and small numbers)
  alpha = std::max(0.1, alpha);
  const double base_cost = kAlphaCrossCost * std::exp(alpha) / std::sqrt(alpha);
  *CHECK_NOTNULL(cost) = base_cost;
  *CHECK_NOTNULL(dcostdalpha) = base_cost * (1.0 - 0.5 / alpha);
}

/**
 * Compute the cost of turning from startheading to endheading,
 * with the current wind direction being winddir (0 = wind blowing to
 * +x axis).
 *
 * Functions by assuming that the boat will travel the shortest path from
 * startheading to endheading and that the cost of traversing X radians
 * has a cost of X^2 on most tacks while traversing X radians of the upwind
 * no-go zones will be (kTackCost * X)^2.
 * For now, hard-code no-go zone as being 45 deg to 45 deg.
 */
void LinePlan::TurnCost(double startheading, double endheading, double winddir,
                        double *cost, double *dcostdstart, double *dcostdend) {
  CHECK_NOTNULL(cost);
  CHECK_NOTNULL(dcostdstart);
  CHECK_NOTNULL(dcostdend);

  // Normalize so that startheading is zero, and so we express
  // the no-go zones in those terms.
  double nogosize = M_PI_4;
  double plusnogoabs = util::norm_angle(M_PI + winddir + nogosize);
  double negnogoabs = util::norm_angle(M_PI + winddir - nogosize);

  double endnorm = util::norm_angle(endheading - startheading);
  double plusnogonorm = util::norm_angle(plusnogoabs - startheading);
  double negnogonorm = util::norm_angle(negnogoabs - startheading);

  double sign = 1.0;

  if (endnorm < 0) {
    endnorm *= -1.0;
    plusnogonorm *= -1.0;
    negnogonorm *= -1.0;
    sign = -1.0;
    std::swap(plusnogonorm, negnogonorm);
  }

  if ((plusnogonorm < 0 && negnogonorm < 0) || negnogonorm > endnorm) {
    // We are not traversing the no-go zone, so calculation is simple
    *cost = kTurnCost * endnorm * endnorm;
    *dcostdstart = -2.0 * kTurnCost * sign * endnorm;
    *dcostdend = 2.0 * kTurnCost * sign * endnorm;
  } else {
    bool clipstart = negnogonorm < 0.0;
    double startnogo = clipstart ? 0.0 : negnogonorm;
    bool clipend = plusnogonorm > endnorm || plusnogonorm < 0.0;
    double endnogo = clipend ? endnorm : plusnogonorm;
    // Total distance spent in no-go zone; shouldn't need to be normalized.
    double nogodist = endnogo - startnogo;

    // Remember not to double-count the nogodist.
    double costbase = endnorm + (kTackCost - 1) * nogodist;
    *cost = kTurnCost * costbase * costbase;
    *dcostdstart =
        -2.0 * kTurnCost * sign * (clipstart ? kTackCost : 1.0) * costbase;
    *dcostdend =
        2.0 * kTurnCost * sign * (clipend ? kTackCost : 1.0) * costbase;
  }
}

/**
 * The cost of travelling a straight line.
 * This computes both the linear cost associated with the length of the line
 * as well as scaling that cost by a factor according to how far upwind we
 * are pointed.
 * is_real: Whether this is an actual path that we are following, or
 *   just a rough heading. If it is a rough heading, then we don't want
 *   to penalize sailing in irons too much, because it is assumed that
 *   we will be handling tacking at a later date.
 * viable: whether or not the current heading is even sailable.
 */
void LinePlan::StraightLineCost(double len, double heading, double winddir,
                                bool is_real, double *cost, double *dcostdlen,
                                double *dcostdheading, bool *viable) {
  // We try to supply a really large cost on sending us off on highly
  // upwind courses, although we don't want to deal with numerical issues.

  // If winddir == heading, we are downwind
  double headingnorm = util::norm_angle(winddir + M_PI - heading);
  // 0 if straight upwind, M_PI if straight downwind
  double upwindness = std::abs(headingnorm);
  double kUp = is_real ? kUpwindCost : kUpwindApproxCost;
  if (viable != nullptr) *viable = upwindness > kSailableReach;
  const double kMaxUpwindness = 0.9;
  upwindness = util::Clip(upwindness, 0.00, kMaxUpwindness);
  //double upwindscalar = 1.0 + kUp * (1.0 - upwindness * upwindness);
  double upwindexp = std::exp(10.0 * (upwindness - M_PI / 6.0));
  double dupwindexpdheading = -10.0 * upwindexp * util::Sign(headingnorm);
  double upwindscalar = kUp / (1.0 + upwindexp);
  // dupwindscalar / dheading =
  //    -kUp / (1 + upwindexp)^2 * dupwindexpdheading
  double dupwindscalardheading =
      -upwindscalar / (1 + upwindexp) * dupwindexpdheading;

  *CHECK_NOTNULL(cost) = upwindscalar * kLengthCost * len;
  VLOG(1) << "line cost: " << *cost << " heading: " << heading
          << " winddir: " << winddir << " headingnorm: " << headingnorm
          << " len: " << len;
  *CHECK_NOTNULL(dcostdlen) = upwindscalar * kLengthCost;
  if (upwindness == kMaxUpwindness || upwindness == 0.0) {
    *CHECK_NOTNULL(dcostdheading) = 0.0;
  } else {
    *CHECK_NOTNULL(dcostdheading) =
      kLengthCost * len * dupwindscalardheading;
//        util::Sign(headingnorm) * 2.0 * kUp * upwindness * kLengthCost * len;
  }
}

/**
 * The obstacle-derived cost along a given line segment.
 * Does not compute partial of cost if dcostdstart is nullptr.
 * Computes derivative numerically, by calling ObstacleCost
 * at slight variations of start.
 * If you want dcostdend, reverse the order of start/end.
 * Computes cost by choosing discrete points along the line segment,
 * attempting to approximate the integral along the line of the
 * cost at any given point.
 * The cost at any given point shall be kObstacleCost * (max(10 - x, 0) / 10)^2
 * where x is the distance from the point to the polygon.
 * The idea of the maximum and the 10 - x is to ignore
 * the obstacle once we get more than 10m away from it,
 * as it will no longer really be relevant.
 * when you get too far into an obstacle
 * WARNING: If you think that your line might cross the obstacle,
 *   then beware of each half of the line having low cost and
 *   not having a clear gradient to follow.
 */
void LinePlan::ObstacleCost(const Eigen::Vector2d &start,
                            const Eigen::Vector2d &end,
                            const std::vector<Polygon> &obstacles, double *cost,
                            Eigen::Vector2d *dcostdstart) {
  // Number of points to use for integration.
  // We shall divide the line into Npts intervals and
  // evaluate at the midpoint of each interval.
  constexpr int Npts = 10;
  double dx = (start - end).norm() / (double)Npts;
  // If cost is nullptr, there isn't really any point to calling this function.
  *CHECK_NOTNULL(cost) = 0.0;
  for (int ii = 0; ii < Npts; ++ii) {
    double alpha = ((double)ii + 0.5) / (double)Npts;
    Point evalpt = start + alpha * (end - start);
    for (size_t jj = 0; jj < obstacles.size(); ++jj) {
      double dist = obstacles[jj].DistToPoint(evalpt);
      //*cost += kObstacleCost * std::exp(-dist) * dx;
      double diff = std::max(0.0, 10.0 - dist) / 10.0;
      *cost += kObstacleCost * diff * diff * dx;
    }
  }

  if (dcostdstart != nullptr) {
    VLOG(1) << "ObstacleCost: " << *cost;
    *dcostdstart *= 0;
    double dcost;
    double eps = 1e-5;
    Point dstart = start;

    dstart.x() += eps;
    ObstacleCost(dstart, end, obstacles, &dcost, nullptr);
    dcostdstart->x() = (dcost - *cost) / eps;
    dstart = start;

    dstart.y() += eps;
    ObstacleCost(dstart, end, obstacles, &dcost, nullptr);
    dcostdstart->y() = (dcost - *cost) / eps;
    VLOG(1) << "dObstacleCostdstart: " << dcostdstart->transpose();
  }
}

void LinePlan::ReceiveObstacles(const msg::Obstacles &msg) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    *obstacles_out_msg_ = msg;
    obstacles_lonlat_.clear();
    for (const msg::WaypointList& obstacle : msg.polygons()) {
      std::vector<Point> obstaclepts;
      for (const msg::Waypoint& pt : obstacle.points()) {
        obstaclepts.emplace_back(pt.x(), pt.y());
      }
      if (Polygon::ValidatePoints(obstaclepts)) {
        obstacles_lonlat_.emplace_back(obstaclepts);
      } else {
        LOG(ERROR) << "Invalid obstacle; see protobuf definition";
        if (FLAGS_v) {
          LOG(FATAL) << "Invalid obstacle; see protobuf definition";
        }
      }
    }
    UpdateObstacles();
}

void LinePlan::ReceiveWaypoints(const msg::WaypointList &msg) {
    if (msg.points_size() == 0) {
      LOG(WARNING) << "Can't send me no waypoints";
      return;
    }
    std::unique_lock<std::mutex> lck(data_mutex_);
    *waypoints_out_msg_ = msg;
    // TODO(james): Test properly

    repeat_waypoints_ = msg.repeat();

    waypoints_lonlat_.clear();

    if (msg.restart() || next_waypoint_ == 0) {
      next_waypoint_ = 1;
    }
    next_waypoint_ = std::min(next_waypoint_.load(), msg.points_size() - 1);

    if (!msg.defined_start()) {
      waypoints_lonlat_.push_back(boat_pos_lonlat_);
      // There's some ambiguity about what to do with the defined_start flag
      // if the user does not request a restart, but for now only worry about
      // if they do request a restart.
    }

    for (const msg::Waypoint &pt : msg.points()) {
      waypoints_lonlat_.emplace_back(pt.x(), pt.y());
    }

    if (lonlat_scale_.x() == 0) {
      // We do not have a lat/lon reference yet, so use first waypoint
      ResetRef(waypoints_lonlat_[0]);
    }
    UpdateWaypoints();
}

void LinePlan::ReadWaypointsAndObstacles() {
  msg::WaypointList way_list;
  if (util::ReadProtoFromFile(FLAGS_initial_waypoints.c_str(), &way_list)) {
    ReceiveWaypoints(way_list);
    ProtoQueue<msg::WaypointList> q("waypoints", true);
    q.send(&way_list);
  } else {
    LOG(WARNING) << "Unable to retrieve waypoints from file";
  }

  msg::Obstacles obs_list;
  if (util::ReadProtoFromFile(FLAGS_initial_obstacles.c_str(), &obs_list)) {
    ReceiveObstacles(obs_list);
    ProtoQueue<msg::Obstacles> q("planner_obstacles", true);
    q.send(&obs_list);
  } else {
    LOG(WARNING) << "Unable to retrieve obstacles from file";
  }
}

}  // control
}  // sailbot
