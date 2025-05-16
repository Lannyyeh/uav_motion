#pragma once

#include <Eigen/Dense>
#include <algorithm>  // for std::clamp

class DisturbEstimator {
public:
    DisturbEstimator()
    {
        Mb = 0.0;
        K_f.setZero();
        f_est.setZero();
        f_int.setZero();
        f_control.setZero();
    }

    void initialize(const double mass, const Eigen::Vector3d& K_force)
    {
        Mb = mass;
        K_f = K_force;
        f_est.setZero();
        f_int.setZero();
        f_control.setZero();
    }

    void setBodyMass(const double mass)
    {
        Mb = mass;
    }

    void setParams(const Eigen::Vector3d& K_force)
    {
        K_f = K_force;
    }
    
    // 设置控制输入（在 update_force 前调用）
    void setControlForce(const Eigen::Vector3d& f_ctrl) {
        f_control = f_ctrl;
    }

    // 更新估计（输入速度和当前时间戳）
    void update_force(const Eigen::Vector3d& velocity, double time_now) {
        double dt = constrain(time_now - f_time, 0.005, 0.02);  // 秒
        f_time = time_now;

        const Eigen::Vector3d world_z(0.0, 0.0, 1.0);

        // 积分更新
        f_int.noalias() += (f_control - Mb * gravity_constant * world_z + f_est) * dt;

        // 力估计更新
        f_est.noalias() = K_f.asDiagonal() * (Mb * velocity - f_int);
    }

    // 获取当前扰动力估计值
    const Eigen::Vector3d& getForceEstimate() const {
        return f_est;
    }

private:
    double Mb;  // 质量
    double f_time; // 上一次更新时间
    Eigen::Vector3d K_f; // 增益

    Eigen::Vector3d f_est;      // 当前扰动力估计
    Eigen::Vector3d f_int;      // 积分状态
    Eigen::Vector3d f_control;  // 当前控制力输入

    static constexpr double gravity_constant = 9.81;

    // 手动约束函数（可替代 std::clamp）
    inline double constrain(double val, double min_val, double max_val) {
        return std::max(min_val, std::min(val, max_val));
    }
};
