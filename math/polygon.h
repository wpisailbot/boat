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

  std::vector<Point> pts() const { return pts_; }
  std::vector<Point> *mutable_pts() { return &pts_; }
  Point pt(size_t ii) const { return pts_[ii]; }

  /**
   * Return the distance to point pt, returning
   * zero if the point is inside the polygon.
   */
  double DistToPoint(Point pt) const;
 private:
  std::vector<Point> pts_;
};  // class Polygon

}  // namespace sailbot
