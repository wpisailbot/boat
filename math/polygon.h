#pragma once
#include <Eigen/Core>
#include <vector>

namespace sailbot {

typedef Eigen::Vector2d Point;

/**
 * TODO(james): Put this in a better spot
 *
 * Computes the distance from pt the the line going through l0 and l1.
 * Signed such that the returned value is positive if pt lies to
 * the left of the line if looking at l1 from l0.
 */
double DistToLine(Point l0, Point l1, Point pt);
/**
 * Determines whether you can draw a line at a right angle to (l0, l1) from some
 * point on (l0, l1) to pt.
 */
bool ProjectsToLine(Point l0, Point l1, Point pt);
/**
 * Determines weather the point pt projects onto the ray that
 * starts at l0 and passes through l1.
 */
bool ProjectsToRay(Point l0, Point l1, Point pt);

/**
 * 2D polygon, all math done in Euclidean plane. i.e., this will not perfectly
 * model the latitude/longitude models.
 */
class Polygon {
 public:
  /**
   * Construct a convex polygon.
   * pts should be the vertices of the polygon, arranged
   * in counter-clockwise order, e.g.:
   *      0---3
   *     /    |
   *    /     |
   *   1------2
   * Validates the input to ensure that we have a convex polygon
   * with points in the correct order.
   * Will also permit two-point polygons (lines), because
   * that can be convenient.
   */
  Polygon(std::vector<Point> pts);

  static bool ValidatePoints(const std::vector<Point> pts);

  // Redoes the processing on the points, e.g. to recalculate
  // the cached centroid.
  void ProcessPts();

  std::vector<Point> pts() const { return pts_; }
  std::vector<Point> *mutable_pts() { return &pts_; }
  Point pt(size_t ii) const { return pts_[ii]; }
  Point centroid() const { return centroid_; }

  /**
   * Return the distance to point pt.
   * If pt is on an edge, returns zero.
   * If pt is inside the polygon, returns a negative number
   * that is maximal at the centroid
   */
  double DistToPoint(Point pt) const;
 private:
  std::vector<Point> pts_;
  Point centroid_;
};  // class Polygon

}  // namespace sailbot
