
#ifndef utilfunctions_h__
#define utilfunctions_h__

#include <cmath>
namespace robo2d {
namespace math {
static inline float sqr(float val) {
  return val*val;
}

static inline int sign(int x) {
  return x > 0 ? 1 : -1;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

}

namespace convert {

// 对角度进行标准化     [-M_PI, M_PI]
static void NormAngle(float& angle) {        
    if(angle > M_PI)
        angle -= 2.0 * M_PI;
    if(angle < -M_PI)
        angle += 2.0 * M_PI;
}
  
template<typename T>
static T toDeg(const T radVal) {
  return radVal * static_cast<T>(180.0 / M_PI);
}

template<typename T>
static T toRad(const T degVal) {
  return degVal * static_cast<T>(M_PI / 180.0);
}

static Eigen::Isometry2f getTransformForState(const Eigen::Vector3f &transVector) {
    return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
}    

static Eigen::Isometry3f get3DTransformForState(const Eigen::Matrix<float, 6, 1>&transVector) {
  Eigen::Isometry3f T;
  Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(transVector(3, 0), Eigen::Vector3f::UnitX()));
  Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(transVector(4, 0), Eigen::Vector3f::UnitY()));
  Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(transVector(5, 0), Eigen::Vector3f::UnitZ()));
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix=yawAngle * pitchAngle * rollAngle;
  T.linear() = rotation_matrix;
  Eigen::Vector3f t = {transVector(0, 0), transVector(1, 0), transVector(2, 0)};
  T.translation() = t;
  return T;
}    

static Eigen::Isometry3d get3DTransformForState(const Eigen::Matrix<double, 6, 1>&transVector) {
  Eigen::Isometry3d T;
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(transVector(3, 0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(transVector(4, 0), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(transVector(5, 0), Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix=yawAngle * pitchAngle * rollAngle;
  T.linear() = rotation_matrix;
  Eigen::Vector3d t = {transVector(0, 0), transVector(1, 0), transVector(2, 0)};
  T.translation() = t;
  return T;
}    
}

namespace module {

static bool poseDifferenceLargerThan(const Eigen::Vector3f& pose1, 
                                                                                const Eigen::Vector3f& pose2, 
                                                                                float distanceDiffThresh, 
                                                                                float angleDiffThresh) {
  //check distance
  if ( ( (pose1.head<2>() - pose2.head<2>()).norm() ) > distanceDiffThresh){
    return true;
  }

  float angleDiff = (pose1.z() - pose2.z());

  if (angleDiff > M_PI) {
    angleDiff -= M_PI * 2.0f;
  } else if (angleDiff < -M_PI) {
    angleDiff += M_PI * 2.0f;
  }

  if (abs(angleDiff) > angleDiffThresh){
    return true;
  }
  return false;
}
}
}
#endif
