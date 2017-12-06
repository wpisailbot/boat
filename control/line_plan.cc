#include "line_plan.h"
#include "control/util.h"
#include <algorithm>

namespace sailbot {
namespace control {

constexpr float LinePlan::dt;
constexpr float LinePlan::kTackCost;
constexpr float LinePlan::kGateWidth;
constexpr float LinePlan::kObstacleCost;

LinePlan::LinePlan()
    : Node(dt), lonlat_ref_(0.0, 0.0), lonlat_scale_(0.0, 0.0) {
  RegisterHandler<msg::WaypointList>("waypoints",
                                     [this](const msg::WaypointList &msg) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    // TODO(james): Reproduce logic from line_tacking to handle sundry flags in
    // the WaypointsList message.
    // Right now behaves as if restart=true, defined_start=true, repeat=false
    waypoints_lonlat_.clear();
    for (const msg::Waypoint &pt : msg.points()) {
      waypoints_lonlat_.emplace_back(pt.x(), pt.y());
    }
    UpdateWaypoints();
  });

  RegisterHandler<msg::BoatState>(
      "boat_state",
      [this](const msg::BoatState &msg) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    boat_pos_ << msg.pos().x(), msg.pos().y();
    yaw_ = msg.euler().yaw();
    // If our position has changed from the last reference by more than 100m,
    // reset.
    if (util::GPSDistance(msg.pos().y(), msg.pos().x(), lonlat_ref_.y(),
                          lonlat_ref_.x()) > 100.0) {
      ResetRef(boat_pos_);
    }
  });

  RegisterHandler<msg::Vector3f>("true_wind", [this](const msg::Vector3f &msg) {
    wind_dir_ = std::atan2(msg.y(), msg.x());
  });

}

void LinePlan::Iterate() {
  // 1) Establish cost function for final position space
  // 2) Iterate over different numbers of line segments, using some reasonable bounds:
  //  a) Start with simple zig-zag
  //  b) Iterate from back-to-front on each segment:
  //   i) Adjust the segment so that we are essentially optimizing a single tack
  //      at a time.
}

void LinePlan::ResetRef(const Point &newlonlatref) {
  std::unique_lock<std::mutex> lck(data_mutex_);

  lonlat_ref_ = newlonlatref;
  double eps = 1e-7;
  lonlat_scale_.x() =
      util::GPSDistance(lonlat_ref_.y(), lonlat_ref_.x(), lonlat_ref_.y(),
                        lonlat_ref_.x() + eps) /
      eps;
  lonlat_scale_.y() =
      util::GPSDistance(lonlat_ref_.y(), lonlat_ref_.x(), lonlat_ref_.y() + eps,
                        lonlat_ref_.x()) /
      eps;

  UpdateObstacles();
  UpdateWaypoints();
}

void LinePlan::UpdateObstacles() {
  obstacles_.clear();
  obstacles_.reserve(obstacles_lonlat_.size());
  // Iterate through every obstacle and point and transform:
  for (size_t ii = 0; ii < obstacles_lonlat_.size(); ++ii) {
    obstacles_[ii] = obstacles_lonlat_[ii];
    for (size_t jj; jj < obstacles_[ii].pts().size(); ++jj) {
      obstacles_[ii].mutable_pts()->at(jj) =
          LonLatToFrame(obstacles_[ii].pt(jj));
    }
  }
}

void LinePlan::UpdateWaypoints() {
  // Iterate through every waypoint and convert appropriately.
  // TODO(james): Figure out better way to receive waypoints/parameterize
  size_t Nway = waypoints_lonlat_.size();
  waypoints_.resize(Nway - 1);

  Point nextpt = LonLatToFrame(waypoints_lonlat_[0]);
  // The trip leg leading up to the current waypoint.
  Point preleg = (nextpt - boat_pos_).normalized();

  for (size_t ii = 1; ii < Nway-1; ++ii) {
    Point curpt = nextpt;
    nextpt = LonLatToFrame(waypoints_lonlat_[ii]);
    Point nextleg = (nextpt - curpt).normalized();
    Point lineseg = preleg - nextleg;
    double norm = lineseg.norm();
    if (norm < 1e-3) {
      // If normalizing lineseg would be unstable, just go with
      // perpendicular to nextleg.
      lineseg << -nextleg.y(), nextleg.x();
      lineseg.normalize();
    } else {
      lineseg /= norm;
    }
    lineseg *= kGateWidth;
    // For the line we are attempting to cross, we shall
    // attempt to cross a constant length line that is halfway
    // between the legs before/after the waypoint and is on the
    // outside of the two (essentially, assuming we are going around
    // a convex polygon).
    waypoints_[ii] = std::make_pair(curpt, curpt + lineseg);
  }
  if (Nway > 1) {
    Point newpt = LonLatToFrame(waypoints_lonlat_[Nway-1]);
    waypoints_[Nway-2] = std::make_pair(newpt, newpt);
  }
}

void
LinePlan::OptimizeTacks(const std::pair<Eigen::Vector2d, Eigen::Vector2d> &gate,
                        const Eigen::Vector2d &nextpt, double winddir,
                        const std::vector<Polygon> &obstacles, double cur_yaw,
                        std::vector<Eigen::Vector2d> *tackpts, double *alpha) {
  CHECK_NOTNULL(tackpts);
  CHECK_NOTNULL(alpha);
  double step = 1.0;
  for (int ii = 0; ii < 10; ++ii) {
    BackPass(gate, nextpt, winddir, obstacles, cur_yaw, step, tackpts, alpha);
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
                        double step, std::vector<Eigen::Vector2d> *tackpts,
                        double *alpha) {
  CHECK_NOTNULL(alpha);
  CHECK_NOTNULL(tackpts);
  CHECK_LE(1, tackpts->size()) << "tackpts should have at least one element";

  double cost;

  size_t Npts = tackpts->size();

  double penultimate_heading =
      Npts > 1 ? util::atan2(tackpts->at(Npts - 1) - tackpts->at(Npts - 2))
               : cur_yaw;

  double dcostdalpha;
  SingleLineCost(gate.first, gate.second, tackpts->at(Npts - 1), *alpha,
                 penultimate_heading, nextpt, winddir, obstacles, &cost,
                 &dcostdalpha);
  *alpha -= step * dcostdalpha;
  *alpha = util::Clip(*alpha, 0.0, 1.0);

  // The point following the one we are currently optimizing.
  Vector2d postpt = gate.first + *alpha * (gate.second - gate.first);
  double postheading = util::atan2(postpt - tackpts->at(Npts - 1));
  for (int ii = Npts - 1; ii >= 1; --ii) {
    // The point before the one that we are currently optimizing.
    const Vector2d &prept = tackpts->at(ii - 1);
    // The point that we are optimizing.
    const Vector2d &curpt = tackpts->at(ii);
    // Heading of the boat coming into prept.
    double preheading =
        (ii == 1) ? cur_yaw : util::atan2(prept - tackpts->at(ii - 2));

    Vector2d dcostdpt;
    LinePairCost(prept, postpt, curpt, preheading, postheading, winddir,
                 obstacles, &cost, &dcostdpt);
    tackpts->at(ii) -= step * dcostdpt;

    postheading = util::atan2(postpt - curpt);
    postpt = curpt;
  }
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
                              double winddir, double *cost,
                              double *dcostdalpha) {
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
  LOG(INFO) << "alpha: " << alpha;

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
    // Because I'm too lazy to avoid these errors properly, assume that if
    // nextleg is sufficiently small, we should just treat this as being a
    // situation where we are in a local minimum.
    nextheading = startheading;
    dnexthda = -dstarthda;
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
  StraightLineCost(firstlen, startheading, winddir, &firstlencost,
                   &dflencostdlen, &dflencostdheading);
  double nextlencost, dnlencostdlen, dnlencostdheading;
  StraightLineCost(nextlen, nextheading, winddir, &nextlencost,
                   &dnlencostdlen, &dnlencostdheading);
  LOG(INFO) << "dfirstcostdlen: " << dflencostdlen
            << " dfirstlenda: " << dfirstlenda
            << " dfirstcostdhead: " << dflencostdheading
            << " dstarthda: " << dstarthda;
  LOG(INFO) << "dnextcostdlen: " << dnlencostdlen
            << " dnextlenda: " << dnextlenda
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

  LOG(INFO) << "preturncost: " << preturncost
            << " dpreturnda: " << dpreturncostda << " midturncost: " << turncost
            << " dturncostda: " << dturncostda;

  turncost += preturncost;
  dturncostda += dpreturncostda;

  double crosscost, dcrossda;
  CrossFinishCost(alpha, &crosscost, &dcrossda);

  LOG(INFO) << "lencost: " << lencost << " turncost: " << turncost
            << " crosscost: " << crosscost;
  LOG(INFO) << "dlencost: " << dlencostda << " dturncost: " << dturncostda
            << " dcrosscost: " << dcrossda;
  *CHECK_NOTNULL(cost) = lencost + turncost + crosscost;
  *CHECK_NOTNULL(dcostdalpha) = dlencostda + dturncostda + dcrossda;
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
                              double *cost, double *dcostdalpha) {
  *CHECK_NOTNULL(cost) = 0.0;
  *CHECK_NOTNULL(dcostdalpha) = 0.0;

  double basecost = 0.0, basedalpha = 0.0;
  SingleLineCost(startline, endline, startpt, alpha, preheading, nextpt,
                 winddir, &basecost, &basedalpha);

  Vector2d crosspt = startline + alpha * (endline - startline);
  Vector2d dcrossptdalpha = endline - startline;
  double obstaclecost = 0.0;
  Vector2d obstacledend;
  ObstacleCost(crosspt, startpt, obstacles, &obstaclecost, &obstacledend);
  LOG(INFO) << "dobstacledend: " << obstacledend.transpose()
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
                            double postheading, double winddir, double *cost,
                            Eigen::Vector2d *dcostdturnpt) {
  *CHECK_NOTNULL(cost) = 0.0;
  *CHECK_NOTNULL(dcostdturnpt) *= 0.0;
  Vector2d sega = turnpt - startpt;
  Vector2d segb = endpt - turnpt;

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
  *cost += turncost;
  *dcostdturnpt += dcostdend * dhadturnpt;

  TurnCost(headinga, headingb, winddir, &turncost, &dcostdstart, &dcostdend);
  *cost += turncost;
  *dcostdturnpt += dcostdstart * dhadturnpt;
  *dcostdturnpt += dcostdend * dhbdturnpt;

  TurnCost(headingb, postheading, winddir, &turncost, &dcostdstart, &dcostdend);
  *cost += turncost;
  *dcostdturnpt += dcostdstart * dhbdturnpt;

  // Calculate costs for the two legs:
  double linecost, dcostdlen, dcostdheading;
  StraightLineCost(lena, headinga, winddir, &linecost, &dcostdlen,
                   &dcostdheading);
  *cost += linecost;
  *dcostdturnpt += dcostdlen * dlenadturnpt + dcostdheading * dhadturnpt;

  StraightLineCost(lenb, headingb, winddir, &linecost, &dcostdlen,
                   &dcostdheading);
  *cost += linecost;
  *dcostdturnpt += dcostdlen * dlenbdturnpt + dcostdheading * dhbdturnpt;
}

void LinePlan::LinePairCost(const Eigen::Vector2d &startpt,
                            const Eigen::Vector2d &endpt,
                            const Eigen::Vector2d &turnpt, double preheading,
                            double postheading, double winddir,
                            const std::vector<Polygon> &obstacles, double *cost,
                            Eigen::Vector2d *dcostdturnpt) {
  *CHECK_NOTNULL(cost) = 0.0;
  *CHECK_NOTNULL(dcostdturnpt) *= 0.0;

  double basecost;
  Vector2d basedturnpt;
  LinePairCost(startpt, endpt, turnpt, preheading, postheading, winddir,
               &basecost, &basedturnpt);
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
  const double alpham5 = alpha - 0.5;
  *CHECK_NOTNULL(cost) = 4.0 * alpham5 * alpham5;
  *CHECK_NOTNULL(dcostdalpha) = 8.0 * alpham5;
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
    *cost = endnorm * endnorm;
    *dcostdstart = -2.0 * sign * endnorm;
    *dcostdend = 2.0 * sign * endnorm;
  } else {
    bool clipstart = negnogonorm < 0.0;
    double startnogo = clipstart ? 0.0 : negnogonorm;
    bool clipend = plusnogonorm > endnorm || plusnogonorm < 0.0;
    double endnogo = clipend ? endnorm : plusnogonorm;
    // Total distance spent in no-go zone; shouldn't need to be normalized.
    double nogodist = endnogo - startnogo;

    // Remember not to double-count the nogodist.
    double costbase = endnorm + (kTackCost - 1) * nogodist;
    *cost = costbase * costbase;
    *dcostdstart = -2.0 * sign * (clipstart ? kTackCost : 1.0) * costbase;
    *dcostdend = 2.0 * sign * (clipend ? kTackCost : 1.0) * costbase;
  }
}

/**
 * The cost of travelling a straight line.
 * This computes both the linear cost associated with the length of the line
 * as well as scaling that cost by a factor according to how far upwind we
 * are pointed.
 */
void LinePlan::StraightLineCost(double len, double heading, double winddir,
    double *cost, double *dcostdlen, double *dcostdheading) {

  // We try to supply a really large cost on sending us off on highly
  // upwind courses, although we don't want to deal with numerical issues.
  double headingnorm = util::norm_angle(winddir + M_PI - heading);
  double upwindness = std::abs(headingnorm);
  upwindness = std::min(std::max(0.01, upwindness), 1.0);
  double upwindscalar = 1.0 / upwindness;

  *CHECK_NOTNULL(cost) = upwindscalar * kLengthCost * len;
  *CHECK_NOTNULL(dcostdlen) = upwindscalar * kLengthCost;
  if (upwindness == 1.0 || upwindness == 0.01) {
    *CHECK_NOTNULL(dcostdheading) = 0.0;
  } else {
    *CHECK_NOTNULL(dcostdheading) = util::Sign(headingnorm) *
                                    (upwindscalar * upwindscalar) *
                                    kLengthCost * len;
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
 * The cost at any given point shall be kObstacleCost * e^{-x}
 * where x is the distance from the point to the polygon.
 */
void LinePlan::ObstacleCost(const Eigen::Vector2d &start,
                            const Eigen::Vector2d &end,
                            const std::vector<Polygon> &obstacles, double *cost,
                            Eigen::Vector2d *dcostdstart) {
  // Number of points to use for integration.
  // We shall divide the line into Npts intervals and
  // evaluate at the midpoint of each interval.
  constexpr int Npts = 10;
  double dx = (start - end).norm() / Npts;
  // If cost is nullptr, there isn't really any point to calling this function.
  *CHECK_NOTNULL(cost) = 0.0;
  double normcost = 0;
  for (int ii = 0; ii < Npts; ++ii) {
    double alpha = ((double)ii + 0.5) / (double)Npts;
    Point evalpt = start + alpha * (end - start);
    for (size_t jj = 0; jj < obstacles.size(); ++jj) {
      double dist = obstacles[jj].DistToPoint(evalpt);
      *cost += kObstacleCost * std::exp(-dist) * dx;
      normcost += kObstacleCost * std::exp(-dist);
    }
  }

  LOG(INFO) << "obstaclestart: " << start.transpose() << " cost: " << *cost
            << " normcost: " << normcost;

  if (dcostdstart != nullptr) {
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
  }
}

}  // control
}  // sailbot
