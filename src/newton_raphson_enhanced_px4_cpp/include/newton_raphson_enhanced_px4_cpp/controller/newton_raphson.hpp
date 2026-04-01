#pragma once

#include <Eigen/Dense>
#include <string>
#include <utility>

namespace newton_raphson_enhanced_px4_cpp::controller {

struct NRResult {
    Eigen::Vector4d u;
    Eigen::Vector4d v;
};

struct NRProfileConfig {
    std::string name = "baseline";
    double lookahead_horizon_s = 1.2;
    Eigen::Vector4d alpha = (Eigen::Vector4d() << 30.0, 40.0, 40.0, 40.0).finished();
    Eigen::Vector4d integral_gain = Eigen::Vector4d::Zero();
    Eigen::Vector4d integral_limit = Eigen::Vector4d::Zero();
    int num_iterations = 1;
    double iteration_damping = 1.0;
    bool use_foh = false;
};

NRProfileConfig build_nr_profile(const std::string& profile_name);

NRResult newton_raphson_enhanced(
    const Eigen::Vector<double, 9>& state,
    const Eigen::Vector4d& last_input,
    const Eigen::Vector4d& reference,
    const Eigen::Vector4d& reference_rate,
    const Eigen::Vector4d& error_integral,
    double lookahead_horizon_s,
    double lookahead_stage_dt,
    double integration_dt,
    double mass,
    const NRProfileConfig& profile);

}  // namespace newton_raphson_enhanced_px4_cpp::controller
