#include "gtest/gtest.h"

#include "util.h"

namespace sailbot {
namespace util {
namespace testing {

TEST(UtilTest, EigenToProto) {
  Vector3d vec(1, 2, 3);
  msg::Vector3f msg;
  EigenToProto(vec, &msg);
  EXPECT_EQ(vec(0), msg.x());
  EXPECT_EQ(vec(1), msg.y());
  EXPECT_EQ(vec(2), msg.z());
}

TEST(UtilTest, NormalizeSimpleAngle) {
  float a = 0.18;
  EXPECT_FLOAT_EQ(norm_angle(a), a);
}

TEST(UtilTest, NormalizeNegativeAngle) {
  float a = -0.18;
  EXPECT_FLOAT_EQ(norm_angle(a), a);
}

TEST(UtilTest, NormalizeLargeAngle) {
  float a = 100;
  EXPECT_FLOAT_EQ(norm_angle(a), -0.53096491487);
}

TEST(UtilTest, NormalizeLargeNegativeAngle) {
  float a = -100;
  EXPECT_FLOAT_EQ(norm_angle(a), 0.53096491487);
}

TEST(UtilTest, HaversineWorks) {
  double lat1 = 42.276618, lon1 = -71.804579; // Salisbury Pond, Institute Park
  double lat2 = lat1 + 1e-4, lon2 = lon1; // Point to the north
  EXPECT_NEAR(11.11949, GPSDistance(lat1, lon1, lat2, lon2), 1e-5);
  EXPECT_FLOAT_EQ(M_PI / 2, GPSBearing(lat1, lon1, lat2, lon2));
  // Now, point to the east
  lat2 = lat1;
  lon2 = lon1 + 1e-5;
  EXPECT_NEAR(0.822737, GPSDistance(lat1, lon1, lat2, lon2), 1e-5);
  EXPECT_NEAR(0, GPSBearing(lat1, lon1, lat2, lon2), 1e-6);
  // Now, point to the southwest
  lat2 = lat1 - 1e-3;
  lon2 = lon1 - 1e-3;
  EXPECT_NEAR(138.323499, GPSDistance(lat1, lon1, lat2, lon2), 1e-5);
  EXPECT_FLOAT_EQ(-2.207815, GPSBearing(lat1, lon1, lat2, lon2));
}

TEST(UtilTest, CheckRPYToQuatAndBack) {
  Vector3d start(-1, 0.1, 2);
  Eigen::Quaterniond q = RollPitchYawToQuat(start);
  Vector3d end = GetRollPitchYawFromQuat(q);
  EXPECT_NEAR(start.x(), end.x(), 1e-5);
  EXPECT_NEAR(start.y(), end.y(), 1e-5);
  EXPECT_NEAR(start.z(), end.z(), 1e-5);

  start << -2, -0.5, -2;
  q = RollPitchYawToQuat(start);
  end = GetRollPitchYawFromQuat(q);
  EXPECT_NEAR(start.x(), end.x(), 1e-5);
  EXPECT_NEAR(start.y(), end.y(), 1e-5);
  EXPECT_NEAR(start.z(), end.z(), 1e-5);
}

TEST(UtilTest, GPSScale) {
  // Expected values calculated using
  // https://msi.nga.mil/MSISiteContent/StaticFiles/Calculators/degree.html
  double lat = 42.276617;
  double latscale, lonscale;
  GPSLatLonScale(lat, &latscale, &lonscale);
  EXPECT_NEAR(111079.0, latscale, 50.0);
  EXPECT_NEAR(82491.0, lonscale, 50.0);

  lat = 20.0;
  GPSLatLonScale(lat, &latscale, &lonscale);
  EXPECT_NEAR(110704.0, latscale, 100.0);
  EXPECT_NEAR(104646.0, lonscale, 100.0);
}

}  // namespace testing
}  // namespace util
}  // namespace sailbot
