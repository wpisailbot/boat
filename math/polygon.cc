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

bool ProjectsToLine(Point l0, Point l1, Point pt) {
  Point line = l1 - l0;
  double dot = line.dot(pt - l0);
  return dot > 0.0 && dot < line.norm();
}

Polygon::Polygon(std::vector<Point> pts) : pts_(pts) {
  size_t N = pts_.size();
  CHECK_LT(1, N) << "Need at least 2 points for a polygon";
  for (size_t ii = 0; ii < N; ++ii) {
    CHECK_GE(DistToLine(pts[ii], pts[(ii + 1) % N], pts[(ii + 2) % N]), 0.0)
        << "Either not convex or not counter-clockwise";
  }
}

double Polygon::DistToPoint(Point pt) const {
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

  const Point &prept = pts_[(minvertex - 1) % N];
  const Point &postpt = pts_[(minvertex + 1) % N];
  const Point &minpt = pts_[minvertex];
  double d0 = -DistToLine(prept, minpt, pt);
  double d1 = -DistToLine(minpt, postpt, pt);
  bool d0proj = ProjectsToLine(prept, minpt, pt);
  bool d1proj = ProjectsToLine(minpt, postpt, pt);

  if (d0 < 0.0 && d1 < 0.0) {
    // Point is inside of polygon
    return 0.0;
  } else if (d1proj && d1 >= 0.0) {
    // Point projects onto (minvertex, minvertex+1) edge
    return d1;
  } else if (d0proj && d0 >= 0.0) {
    // Point projects onto (minvertex-1, minvertex) edge
    return d0;
  } else {
    // Point projects to minvertex.
    return mindist;
  }

}

}  // namespace sailbot
