#include "line_plan.h"
#include "control/util.h"
#include <algorithm>

namespace sailbot {
namespace control {

constexpr float LinePlan::dt;
constexpr float LinePlan::kTackCost;

LinePlan::LinePlan() : Node(dt) {

}

void LinePlan::Iterate() {
  // 1) Establish cost function for final position space
  // 2) Iterate over different numbers of line segments, using some reasonable bounds:
  //  a) Start with simple zig-zag
  //  b) Iterate from back-to-front on each segment:
  //   i) Adjust the segment so that we are essentially optimizing a single tack
  //      at a time.
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
                              const Eigen::Vector2d &nextpt, double winddir,
                              double *cost, double *dcostdalpha) {
  // endpt is the point along the finish line that we are
  // crossing.
  Eigen::Vector2d endpt = startline + alpha * (endline - startline);
  Eigen::Vector2d dendda = endline - startline;
  // firstleg is a vector of the leg from startpt to the finish line.
  Eigen::Vector2d firstleg = endpt - startpt;
  Eigen::Vector2d dfirstlegda = dendda;
  // nextleg is the leg from endpt to nextpt.
  Eigen::Vector2d nextleg = nextpt - endpt;
  Eigen::Vector2d dnextlegda = -dendda;

  auto datan = [](const Eigen::Vector2d &x, const Eigen::Vector2d &dxda) {
    double sqNorm = x.squaredNorm();
    Eigen::Vector2d d(-x.y(), x.x());
    d /= sqNorm;
    return d.transpose() * dxda;
  };

  double startheading = std::atan2(firstleg.y(), firstleg.x());
  double nextheading = std::atan2(nextleg.y(), nextleg.x());
  double dstarthda = datan(firstleg, dfirstlegda);
  double dnexthda = datan(nextleg, dnextlegda);

  auto dnorm = [](Eigen::Vector2d x, const Eigen::Vector2d &dxda) {
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

  double lencost = firstlencost + nextlencost;
  double dlencostda = dflencostdlen * dfirstlenda +
                      dflencostdheading * dstarthda +
                      dnlencostdlen * dnextlenda + dnlencostdheading * dnexthda;

  double turncost, dturndstart, dturndend;
  TurnCost(startheading, nextheading, winddir, &turncost, &dturndstart,
           &dturndend);
  double dturncostda = dturndstart * dstarthda + dturndend * dnexthda;

  double crosscost, dcrossda;
  CrossFinishCost(alpha, &crosscost, &dcrossda);

  *CHECK_NOTNULL(cost) = lencost + turncost + crosscost;
  *CHECK_NOTNULL(dcostdalpha) = dlencostda + dturncostda + dcrossda;
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
  Eigen::Vector2d sega = turnpt - startpt;
  Eigen::Vector2d segb = endpt - turnpt;

  double lena = sega.norm();
  Eigen::Vector2d dlenadturnpt = sega / sega.norm();

  double lenb = segb.norm();
  // Negative because dsegb / dturn = -1
  Eigen::Vector2d dlenbdturnpt = -segb / segb.norm();

  double headinga = std::atan2(sega.y(), sega.x());
  Eigen::Vector2d dhadturnpt =
      Eigen::Vector2d(-sega.y(), sega.x()) / sega.squaredNorm();
  double headingb = std::atan2(segb.y(), segb.x());
  Eigen::Vector2d dhbdturnpt =
      -Eigen::Vector2d(-segb.y(), segb.x()) / segb.squaredNorm();

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
 * startheading to endheading and that the cost of traversing 1 radian
 * has a cost of 1 on most tacks while traversing 1 radian of the upwind no-go
 * zones will be kTackCost.
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
    *cost = endnorm;
    *dcostdstart = -sign;
    *dcostdend = sign;
  } else {
    bool clipstart = negnogonorm < 0.0;
    double startnogo = clipstart ? 0.0 : negnogonorm;
    bool clipend = plusnogonorm > endnorm || plusnogonorm < 0.0;
    double endnogo = clipend ? endnorm : plusnogonorm;
    // Total distance spent in no-go zone; shouldn't need to be normalized.
    double nogodist = endnogo - startnogo;

    // Remember not to double-count the nogodist.
    *cost = endnorm + (kTackCost - 1) * nogodist;
    *dcostdstart = -sign * (clipstart ? kTackCost : 1.0);
    *dcostdend = sign * (clipend ? kTackCost : 1.0);
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
    *CHECK_NOTNULL(dcostdheading) =
        util::Sign(headingnorm) * (upwindscalar * upwindscalar) * kLengthCost;
  }
}

}  // control
}  // sailbot
