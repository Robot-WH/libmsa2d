
#pragma once 
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace msa2d {

class Pose2d {
    public:
        Pose2d() {
            vec_.setZero(); 
            orientation_.setIdentity(); 
        }
        
        Pose2d(float x, float y, float theta) : vec_(x, y, theta) {
            NormAngle(vec_[2]);
            // yaw 转 四元数
            Eigen::AngleAxisf rollAngle(0,Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitchAngle(0,Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yawAngle(vec_[2],Eigen::Vector3f::UnitZ());
            orientation_ = rollAngle * pitchAngle * yawAngle;
        }

        Pose2d(float x, float y, Eigen::Quaternionf const& orientation) {
            vec_[0] = x;
            vec_[1] = y;
            orientation_ = orientation.normalized();  
            Eigen::Vector3f eulerAngle=orientation_.matrix().eulerAngles(0,1,2);    // 分解为 0 <- 1 <- 2 顺序的欧拉角
            vec_[2] = eulerAngle[2];  
        }

        void SetIdentity() {
            orientation_.setIdentity(); 
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

        void SetRotation(Eigen::Quaternionf const& orientation) {
            orientation_ = orientation.normalized();  
            Eigen::Vector3f eulerAngle = orientation_.matrix().eulerAngles(0,1,2);    // 分解为 0 <- 1 <- 2 顺序的欧拉角
            vec_[2] = eulerAngle[2];  
            NormAngle(vec_[2]); 
        }

        void SetRotation(const float& x, const float& y, const float& z, const float& w) {
            orientation_.x() = x;  
            orientation_.y() = y;  
            orientation_.z() = z;  
            orientation_.w() = w;
            orientation_.normalize(); 
            Eigen::Vector3f eulerAngle = orientation_.matrix().eulerAngles(0,1,2);    // 分解为 0 <- 1 <- 2 顺序的欧拉角
            vec_[2] = eulerAngle[2];  
            NormAngle(vec_[2]); 
        }

        void SetRotation(float const& yaw) {
            vec_[2] = yaw;
            NormAngle(vec_[2]); 
            Eigen::AngleAxisf rollAngle(0,Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitchAngle(0,Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yawAngle(vec_[2],Eigen::Vector3f::UnitZ());
            orientation_ = rollAngle * pitchAngle * yawAngle;   // z->y->x
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

        const Eigen::Quaternionf& orientation() const {
            return orientation_; 
        }  

        const Eigen::Vector3f& vec() const {
            return vec_; 
        }

        // 重载pose的乘法   Pose2d (x,y,theta)  p1*p2   T1*T2  
        const Pose2d operator*(const Pose2d& p2) {  
            Pose2d p;
            Eigen::Matrix2f R;
            // 构造旋转矩阵
            R << cos(vec_[2]), -sin(vec_[2]),
            sin(vec_[2]), cos(vec_[2]);
            Eigen::Vector2f pt2(p2.vec_[0], p2.vec_[1]);          
            Eigen::Vector2f pt = R * pt2 + Eigen::Vector2f(vec_[0], vec_[1]);     // t = Rt + t 
            
            p.vec_[0] = pt(0);
            p.vec_[1] = pt(1);
            p.vec_[2] = vec_[2] + p2.vec_[2];         // 这里注意  由于是2D平面  所以旋转可以转换为一个旋转向量 z*theta,  
                                                                        // z为z轴, 即绕着z轴旋转   因此两次旋转 R1*R2 的结果 可以看成绕着z轴连续旋转2次 , 因此角度直接相加 
            NormAngle(p.vec_[2]);      // 规范化 
            p.orientation_ = orientation_ * p2.orientation_;   // 四元数更新
            p.orientation_.normalize(); 
            return p;
        }  

        // 重载  T*P
        const Eigen::Vector2f operator*(const Eigen::Vector2f& p) const {  
            Eigen::Matrix2f R;
            R << cos(vec_[2]), -sin(vec_[2]),
            sin(vec_[2]), cos(vec_[2]);
            Eigen::Vector2f t(vec_[0], vec_[1]);
            return R * p + t;
        }  

        Pose2d inv() {
            float x = - ( cos(vec_[2]) * vec_[0] + sin(vec_[2]) * vec_[1]);
            float y = - ( -sin(vec_[2]) * vec_[0] + cos(vec_[2]) * vec_[1]);
            float theta = - vec_[2];
            return Pose2d(x, y, theta);
        }

        // 对角度进行标准化     [-M_PI, M_PI]
        static void NormAngle(float& angle) {        
            if(angle >= M_PI)
                angle -= 2.0*M_PI;
            if(angle < -M_PI)
                angle += 2.0*M_PI;
        }
    private:
        Eigen::Vector3f vec_;   // x, y, yaw
        Eigen::Quaternionf orientation_;  
}; //class Pose2d

struct TimedPose2d {
    Pose2d pose_;
    double time_stamp_ = -1;  
};
}