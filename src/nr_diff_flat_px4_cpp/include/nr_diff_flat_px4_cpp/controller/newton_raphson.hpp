#pragma once

#include <Eigen/Dense>
#include <string>
#include <utility>

namespace nr_diff_flat_px4_cpp::controller {

struct NRResult {
    Eigen::Vector4d u;
    Eigen::Matrix<double, 12, 1> x_df;
    Eigen::Vector4d v;
};

struct NRProfileConfig {
    std::string name = "baseline";
    double lookahead_horizon_s = 0.8;
    Eigen::Vector4d alpha = (Eigen::Vector4d() << 20.0, 30.0, 30.0, 30.0).finished();
    Eigen::Vector4d integral_gain = Eigen::Vector4d::Zero();
    Eigen::Vector4d integral_limit = Eigen::Vector4d::Zero();
    int num_iterations = 1;
    double iteration_damping = 1.0;
    bool use_thrust_cbf = true;
};

NRProfileConfig build_nr_profile(const std::string& profile_name);

NRResult nr_tracker_flat(
    const Eigen::Vector<double, 9>& state,
    const Eigen::Vector4d& last_input,
    const Eigen::Matrix<double, 12, 1>& x_df,
    const Eigen::Vector4d& reference,
    const Eigen::Vector4d& error_integral,
    double lookahead_horizon_s,
    double integration_dt,
    double mass,
    const Eigen::Matrix3d& rot_matrix,
    double yaw_dot,
    const NRProfileConfig& profile);

}  // namespace nr_diff_flat_px4_cpp::controller
