#include "polygon.h"

#include "glog/logging.h"

namespace sailbot {

double DistToLine(Point l0, Point l1, Point pt) {
  // Method:
  // 1) Calculate the vector perpendicular to (l0, l1)
  // 2) Project pt onto that vector
  // 3) Ta-Da!

  Point perp(l0.y() - l1.y(), l1.x() - l0.x());
  pt = pt - l0;
  return perp.dot(pt);
}

Polygon::Polygon(std::vector<Point> pts) : pts_(pts) {
  size_t N = pts_.size();
  CHECK_LT(2, N) << "Need at least 3 points for a polygon";
  for (size_t ii = 0; ii < N; ++ii) {
    CHECK_GT(DistToLine(pts[ii], pts[(ii + 1) % N], pts[(ii + 2) % N]), 0.0)
        << "Either not convex or not counter-clockwise";
  }
}

double Polygon::DistToPoint(Point pt) {
  // To calculate, iterate through points, and choose the two nearest to pt.
  // Use dot product to figure out if pt projects onto segment or not, and
  // use DisttoLine.
  size_t N = pts_.size();

  // Minimum distance and corresponding node thus far.
  double mindist = std::numeric_limits<double>::infinity();
  size_t minvertex = 0;
  for (size_t ii = 0; ii < N; ++ii) {
    double dist = (pt - pts_[ii]).norm();
    if (dist < mindist) {
      mindist = dist;
      minvertex = ii;
    }
  }

  double d0 = -DistToLine(pts_[(minvertex-1) % N], pts_[minvertex], pt);
  double d1 = -DistToLine(pts_[minvertex], pts_[(minvertex+1) % N], pt);

  if (d0 < 0.0 && d1 < 0.0) {
    // Point is inside of polygon
    return 0.0;
  } else if (d0 < 0.0) {
    // Point projects onto (minvertex, minvertex+1) edge
    return d1;
  } else if (d1 < 0.0) {
    // Point projects onto (minvertex-1, minvertex) edge
    return d0;
  } else {
    // Point projects to minvertex.
    return mindist;
  }

}

}  // namespace sailbot
