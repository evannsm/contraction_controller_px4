#include "newton_raphson_px4_cpp/controller/newton_raphson.hpp"
#include "newton_raphson_px4_cpp/controller/nr_utils.hpp"

namespace newton_raphson_px4_cpp::controller {

static const Eigen::Vector4d ALPHA = (Eigen::Vector4d() << 20.0, 30.0, 30.0, 30.0).finished();
static constexpr bool USE_CBF = true;

NRResult newton_raphson_standard(
    const Eigen::Vector<double, 9>& state,
    const Eigen::Vector4d& last_input,
    const Eigen::Vector4d& reference,
    double lookahead_horizon_s,
    double lookahead_stage_dt,
    double integration_dt,
    double mass)
{
    Eigen::Vector4d y_pred = predict_output<double>(state, last_input,
                                                     lookahead_horizon_s, lookahead_stage_dt, mass);
    Eigen::Vector4d error = get_tracking_error(reference, y_pred);


    // Match the Python implementation: use the Jacobian pseudo-inverse rather
    // than an exact solve so the NR step stays well-behaved near singular or
    // poorly conditioned operating points introduced by feedforward.
    Eigen::Matrix4d dgdu_inv = get_inv_jac_pred_u(
        state, last_input, lookahead_horizon_s, lookahead_stage_dt, mass);
    Eigen::Vector4d NR = dgdu_inv * error;

    Eigen::Vector4d v = USE_CBF ? get_integral_cbf(last_input, NR) : Eigen::Vector4d::Zero();
    Eigen::Vector4d udot = NR + v;
    Eigen::Vector4d change_u = udot * integration_dt;

    Eigen::Vector4d u = last_input + ALPHA.cwiseProduct(change_u);

    return {u, v};
}

}  // namespace newton_raphson_px4_cpp::controller
