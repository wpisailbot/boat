#include "gtest/gtest.h"

#include "polygon.h"

namespace sailbot {

TEST(PolygonTest, Constructs) {
  std::vector<Point> pts({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
  Polygon poly(pts);
  for (size_t ii = 0; ii < pts.size(); ++ii) {
    EXPECT_EQ(pts[ii], poly.pt(ii)) << "Something is horribly wrong";
  }
}

TEST(PolygonTest, FailsNonConvex) {
  std::vector<Point> pts(
      {{0.0, 0.0}, {0.5, 0.5}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
  EXPECT_DEATH(Polygon pt = Polygon(pts), ".*convex.*")
      << "Should crash when given non-convex polygon";
}

TEST(PolygonTest, FailsOutOfOrder) {
  std::vector<Point> pts(
      {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}});
  EXPECT_DEATH(Polygon pt = Polygon(pts), ".*clockwise.*")
      << "Should crash when given incorrect ordering of points";
}

TEST(PolygonTest, DistToLine) {
  std::vector<Point> pts({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
  Polygon poly(pts);
  EXPECT_EQ(1.0, poly.DistToPoint({2.0, 0.5}))
      << "Failed simple distance to edge";
  EXPECT_EQ(5.0, poly.DistToPoint({-3.0, -4.0}))
      << "Failed when individual vertex is closest";
}

TEST(PolygonTest, DistToInsidePt) {
  std::vector<Point> pts({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
  Polygon poly(pts);
  EXPECT_EQ(0.0, poly.DistToPoint({0.5, 0.5}))
      << "Points inside of polygon should be zero distance away";
}

}  // namespace sailbot
