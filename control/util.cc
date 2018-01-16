#include "util.h"
#include <random>

namespace sailbot {
namespace util {

// Transforms a vector x using rotation matrix R and origin.
Eigen::Vector3d Trans(const Vector3d& x, const Matrix3d& R, const Vector3d& o) {
  return R * x + o;
}

Matrix3d Skew(const Vector3d& x) {
  Matrix3d Sx;
  Sx << 0, -x(2), x(1),
        x(2), 0, -x(0),
        -x(1), x(0), 0;
  return Sx;
}

Matrix3d Orthogonalize(const Matrix3d &R) {
  double err = R.row(0).dot(R.row(1));
  Matrix3d Rn;
  Rn.row(2) = R.row(2) - err/2 * R.row(1);
  Rn.row(1) = R.row(1) - err/2 * R.row(2);
  Rn.row(0) = Rn.row(2).cross(Rn.row(1));
  Rn.row(0) *= .5 * (3 - Rn.row(0).squaredNorm());
  Rn.row(1) *= .5 * (3 - Rn.row(1).squaredNorm());
  Rn.row(2) *= .5 * (3 - Rn.row(2).squaredNorm());
  return Rn;
}

void EigenToProto(const Eigen::Vector3d &ve,
                         sailbot::msg::Vector3f *msg) {
  msg->set_x(ve(0, 0));
  msg->set_y(ve(1, 0));
  msg->set_z(ve(2, 0));
}

void EigenToProtod(const Eigen::Vector3d &ve,
                         sailbot::msg::Vector3d *msg) {
  msg->set_x(ve(0, 0));
  msg->set_y(ve(1, 0));
  msg->set_z(ve(2, 0));
}

Vector3d GetRollPitchYaw(Matrix3d R) {
  // We don't want to just use Eigen's toEulerAngle, because they
  // don't always turn things out in useful ranges (eg, a pitch of PI + roll of
  // PI = a yaw of PI).
  // First, get the yaw and pull it out of R.
  double yaw = std::atan2(R(1, 0), R(0, 0));
  R = Eigen::AngleAxisd(-yaw, Vector3d::UnitZ()) * R;
  double pitch = std::atan2(-R(2, 0), R(0, 0));
  R = Eigen::AngleAxisd(-pitch, Vector3d::UnitY()) * R;
  double roll = std::atan2(R(2, 1), R(1, 1));
  return Vector3d(roll, pitch, yaw);
}

Vector3d GetRollPitchYawFromQuat(Eigen::Quaterniond q) {
  return GetRollPitchYaw(q.toRotationMatrix());
}

Matrix3d RollPitchYawToMatrix(const Vector3d &angles) {
  Matrix3d R = Eigen::AngleAxisd(angles[0], Vector3d::UnitX()) * Matrix3d::Identity(); // Roll
  R = Eigen::AngleAxisd(angles[1], Vector3d::UnitY()) * R; // Pitch
  R = Eigen::AngleAxisd(angles[2], Vector3d::UnitZ()) * R; // Yaw
  return R;
}

Eigen::Quaterniond RollPitchYawToQuat(const Vector3d &angles) {
  return Eigen::Quaterniond(RollPitchYawToMatrix(angles));
}

double norm_angle(double a) {
  double tau = 2. * M_PI;
  if (a > M_PI) {
    a -= int(a / tau + .5) * tau;
  } else if (a < -M_PI) {
    a -= int(a / tau - .5) * tau;
  }
  return a;
}

namespace {
double GPSDistanceRad(double lat1, double lon1, double lat2, double lon2) {
  // Haversine function
  double a = std::pow(std::sin((lat2 - lat1) / 2.), 2) +
             std::cos(lat1) * std::cos(lat2) *
                 std::pow(std::sin((lon2 - lon1) / 2.), 2);
  double c = 2. * std::atan2(std::sqrt(a), std::sqrt(1 - a));
  constexpr double R = 6.371e6; // Earth's Radius
  double d = R * c;
  return d;
}

// Aims to return a bearing with 0deg = West, 90deg = North
double GPSBearingRad(double lat1, double lon1, double lat2, double lon2) {
  const double y = std::sin(lon2 - lon1) * std::cos(lat2);
  const double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) * std::cos(lon2 - lon1);
  return std::atan2(x, y);
}

} // namespace

double GPSDistance(double lat1, double lon1, double lat2, double lon2) {
  return GPSDistanceRad(ToRad(lat1), ToRad(lon1), ToRad(lat2), ToRad(lon2));
}

// Aims to return a bearing with 0deg = West, 90deg = North

double GPSBearing(double lat1, double lon1, double lat2, double lon2) {
  return GPSBearingRad(ToRad(lat1), ToRad(lon1), ToRad(lat2), ToRad(lon2));
}

void GPSLatLonScale(double lat, double *latscale,
                      double *lonscale) {
  // Calculates meters per radian of lat/lon for the WGS84 ellipsoid at a given
  // latitude (in degrees).
  // See
  // https://knowledge.safe.com/articles/725/calculating-accurate-length-in-meters-for-lat-long.html
  double clat = std::cos(ToRad(lat));
  double clat2 = clat * clat;
  double denom = std::sqrt(1.0 + 0.0067394967565868823004 * clat2);
  *latscale = 111693.97955992134774 / (denom * denom * denom);
  *lonscale = 111693.97955992134774 * clat / denom;
}

namespace {
  std::default_random_engine generator;
}

float Normal(float mean, float std) {
  std::normal_distribution<float> dist(mean, std);
  return dist(generator);
}

double atan2(const Eigen::Vector2d &diff) {
  return std::atan2(diff.y(), diff.x());
}

}  // namespace util
}  // namespace sailbot
