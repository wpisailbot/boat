#include "polygon.h"

#include "glog/logging.h"

namespace sailbot {

double DistToLine(Point l0, Point l1, Point pt) {
  // Method:
  // 1) Calculate the unit vector perpendicular to (l0, l1)
  // 2) Project pt onto that vector
  // 3) Ta-Da!

  Point perp(l0.y() - l1.y(), l1.x() - l0.x());
  perp.normalize();
  pt = pt - l0;
  return perp.dot(pt);
}

bool ProjectsToLine(Point l0, Point l1, Point pt) {
  Point line = l1 - l0;
  double dot = line.dot(pt - l0);
  return dot > 0.0 && dot < line.squaredNorm();
}

Polygon::Polygon(std::vector<Point> pts) : pts_(pts) {
  CHECK(ValidatePoints(pts)) << "Points not correctly arranged";
  ProcessPts();
}

bool Polygon::ValidatePoints(const std::vector<Point> pts) {
  size_t N = pts.size();
  if (N < 2) {
    LOG(ERROR) << "Need at least 2 points for a polygon";
    return false;
  }
  for (size_t ii = 0; ii < N; ++ii) {
    if (DistToLine(pts[ii], pts[(ii + 1) % N], pts[(ii + 2) % N]) < 0.0) {
      LOG(ERROR) << "Either not convex or not counter-clockwise";
      return false;
    }
  }
  return true;
}

void Polygon::ProcessPts() {
  return;
  // To calculate centroid, we use the formulas from
  // wikipedia:
  double area = 0;
  const int N = pts_.size();
  for (int ii = 0; ii < N; ++ii) {
    int nextidx = (ii + 1) % N;
    area += 0.5 *
            (pts_[ii].x() * pts_[nextidx].y() - pts_[nextidx].x() * pts_[ii].y());
  }

  centroid_.x() = 0;
  centroid_.y() = 0;
  for (int ii = 0; ii < N; ++ii) {
    int nextidx = (ii + 1) % N;
    double common =
        pts_[ii].x() * pts_[nextidx].y() - pts_[nextidx].x() * pts_[ii].y();
    centroid_.x() += (pts_[ii].x() + pts_[nextidx].x()) * common;
    centroid_.y() += (pts_[ii].y() + pts_[nextidx].y()) * common;
  }
  centroid_ /= 6.0 * area;
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
    return std::min(d0, d1);
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
