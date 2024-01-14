
#pragma once 
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "UtilFunctions.hpp"
namespace msa2d {
class Pose2d {
public:
    Pose2d() {
        vec_.setZero(); 
    }
    
    Pose2d(float x, float y, float theta) : vec_(x, y, theta) {
        convert::NormAngle(vec_[2]);
    }

    Pose2d(const Eigen::Vector3f& vec) : vec_(vec) {
        convert::NormAngle(vec_[2]);
    }

    void SetIdentity() {
        vec_.setZero(); 
    }

    void SetX(const float& x ) {
        vec_[0] = x;
    }

    void SetY(const float& y) {
        vec_[1] = y;
    }

    void SetTransform(float const& x, float const& y) {
        vec_[0] = x;
        vec_[1] = y;
    }

    void SetRotation(float const& yaw) {
        vec_[2] = yaw;
        convert::NormAngle(vec_[2]); 
    }

    void SetVec(const Eigen::Vector3f& vec) {
        vec_ = vec; 
    }

    const float& x() const {
        return vec_[0];
    }

    const float& y() const {
        return vec_[1];
    }

    const float& yaw() const {
        return vec_[2];
    }

    const Eigen::Vector3f& vec() const {
        return vec_; 
    }

    bool operator == (const Pose2d& rhs) {
        if (vec_[0] == rhs.x() && vec_[1] == rhs.y() && vec_[2] == rhs.yaw())
            return true;
        return false;  
    }

    bool operator != (const Pose2d& rhs) {
        return !(*this == rhs);  
    }

    // 重载pose的乘法   Pose2d (x,y,theta)  p1*p2   T1*T2  
    const Pose2d operator*(const Pose2d& p2) {  
        Pose2d p;
        Eigen::Matrix2f R;
        // 构造旋转矩阵   R1<-2
        // R << cos(vec_[2]), -sin(vec_[2]),
        //         sin(vec_[2]), cos(vec_[2]);
        R(0, 0) = cos(vec_[2]); R(0, 1) = -sin(vec_[2]);
        R(1, 0) = sin(vec_[2]); R(1, 1) = cos(vec_[2]);
        Eigen::Vector2f pt2(p2.vec_[0], p2.vec_[1]);          
        Eigen::Vector2f pt = R * pt2 + Eigen::Vector2f(vec_[0], vec_[1]);     // t = Rt + t 
        
        p.vec_[0] = pt(0);
        p.vec_[1] = pt(1);
        p.vec_[2] = vec_[2] + p2.vec_[2];         // 这里注意  由于是2D平面  所以旋转可以转换为一个旋转向量 z*theta,  
                                                                    // z为z轴, 即绕着z轴旋转   因此两次旋转 R1*R2 的结果 可以看成绕着z轴连续旋转2次 , 因此角度直接相加 
        convert::NormAngle(p.vec_[2]);      // 规范化 
        return p;
    }  

    // 重载  T*P
    // Rgl * Pl = Pg  , 局部转到全局 
    const Eigen::Vector2f operator*(const Eigen::Vector2f& p) const {  
        Eigen::Matrix2f R;
        // R << cos(vec_[2]), -sin(vec_[2]),
        // sin(vec_[2]), cos(vec_[2]);
        R(0, 0) = cos(vec_[2]); R(0, 1) = -sin(vec_[2]);
        R(1, 0) = sin(vec_[2]); R(1, 1) = cos(vec_[2]);
        Eigen::Vector2f t(vec_[0], vec_[1]);
        return R * p + t;
    }  

    Pose2d inv() const {
        float x = - ( cos(vec_[2]) * vec_[0] + sin(vec_[2]) * vec_[1]);
        float y = - ( -sin(vec_[2]) * vec_[0] + cos(vec_[2]) * vec_[1]);
        float theta = - vec_[2];
        return Pose2d(x, y, theta);
    }

private:
    Eigen::Vector3f vec_;   // x, y, yaw
}; //class Pose2d

struct TimedPose2d {
    Pose2d pose_;
    double time_stamp_ = -1;  
};
}