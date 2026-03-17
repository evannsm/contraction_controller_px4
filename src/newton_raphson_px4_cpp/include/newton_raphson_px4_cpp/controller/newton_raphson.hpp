#pragma once

#include <Eigen/Dense>
#include <utility>

namespace newton_raphson_px4_cpp::controller {

struct NRResult {
    Eigen::Vector4d u;
    Eigen::Vector4d v;
};

NRResult newton_raphson_standard(
    const Eigen::Vector<double, 9>& state,
    const Eigen::Vector4d& last_input,
    const Eigen::Vector4d& reference,
    double lookahead_horizon_s,
    double lookahead_stage_dt,
    double integration_dt,
    double mass);

}  // namespace newton_raphson_px4_cpp::controller
