#include "gtest/gtest.h"

#include "polygon.h"

namespace sailbot {

TEST(PolygonUtilTest, DistToLine) {
  EXPECT_EQ(1.0, DistToLine({2.0, 0.0}, {2.0, 2.0}, {1.0, 1.0}));
}

TEST(PolygonUtilTest, ProjectsToLine) {
  EXPECT_TRUE(ProjectsToLine({0.0, 0.0}, {1.0, 0.0}, {0.5, 0.5}));
  EXPECT_TRUE(ProjectsToLine({0.0, 1.0}, {0.0, -2.0}, {0.5, 0.5}));
  EXPECT_TRUE(ProjectsToLine({0.0, 1.0}, {0.0, -2.0}, {-0.5, 0.5}));
  EXPECT_TRUE(ProjectsToLine({2.0, 2.0}, {2.0, 0.0}, {1.0, 1.0}));
  EXPECT_FALSE(ProjectsToLine({0.0, 1.0}, {0.0, -2.0}, {0.5, 2.5}));
  EXPECT_FALSE(ProjectsToLine({0.0, 1.0}, {0.0, -2.0}, {0.5, -2.5}));
}

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

TEST(PolygonTest, OnEdgePoint) {
  // Test when a point is exactly on a given edge.
  std::vector<Point> pts({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
  Polygon poly(pts);
  EXPECT_EQ(0.0, poly.DistToPoint({0.0, 0.5}))
      << "Points inside of polygon should be zero distance away";
}

TEST(PolygonTest, NonSquarePolygon) {
  // Test that edges not at right angles work properly.
  std::vector<Point> pts({{0.0, 0.0}, {2.0, 0.0}, {1.0, 0.5}});
  Polygon poly(pts);
  EXPECT_EQ(1.0, poly.DistToPoint({0.0, 1.0}))
      << "Point off of obtuse corner";
  EXPECT_EQ(1.0, poly.DistToPoint({3.0, 0.0})) << "Point off of acute corner";
}

TEST(PolygonTest, TwoPointsFunction) {
  std::vector<Point> pts({{0.0, 0.0}, {1.0, 0.0}});
  Polygon poly(pts);
  EXPECT_EQ(0.0, poly.DistToPoint({0.5, 0.0}))
      << "Point on line should be zero distance";
  EXPECT_EQ(0.5, poly.DistToPoint({1.5, 0.0}));
  EXPECT_EQ(0.5, poly.DistToPoint({0.0, 0.5}));
  EXPECT_EQ(0.5, poly.DistToPoint({0.5, 0.5}));
  EXPECT_EQ(0.25, poly.DistToPoint({0.75, -0.25}));
}

}  // namespace sailbot
