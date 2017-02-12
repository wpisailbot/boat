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
  float lat1 = 42.276618, lon1 = -71.804579; // Salisbury Pond, Institute Park
  float lat2 = lat1 + 1e-4, lon2 = lon1; // Point to the north
  float lat1rad = ToRad(lat1), lon1rad = ToRad(lon1);
  float lat2rad = ToRad(lat2), lon2rad = ToRad(lon2);
  EXPECT_NEAR(11.01249, GPSDistance(lat1rad, lon1rad, lat2rad, lon2rad), 1e-5);
  EXPECT_FLOAT_EQ(M_PI / 2, GPSBearing(lat1rad, lon1rad, lat2rad, lon2rad));
  // Now, point to the east
  lat2 = lat1;
  lon2 = lon1 + 1e-5;
  lat1rad = ToRad(lat1);
  lon1rad = ToRad(lon1);
  lat2rad = ToRad(lat2);
  lon2rad = ToRad(lon2);
  EXPECT_NEAR(1.12389, GPSDistance(lat1rad, lon1rad, lat2rad, lon2rad), 1e-5);
  EXPECT_FLOAT_EQ(0, GPSBearing(lat1rad, lon1rad, lat2rad, lon2rad));
  // Now, point to the southwest
  lat2 = lat1 - 1e-3;
  lon2 = lon1 - 1e-3;
  lat1rad = ToRad(lat1);
  lon1rad = ToRad(lon1);
  lat2rad = ToRad(lat2);
  lon2rad = ToRad(lon2);
  EXPECT_NEAR(138.2427, GPSDistance(lat1rad, lon1rad, lat2rad, lon2rad), 1e-5);
  EXPECT_FLOAT_EQ(-2.2061794, GPSBearing(lat1rad, lon1rad, lat2rad, lon2rad));
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

}  // namespace testing
}  // namespace util
}  // namespace sailbot
