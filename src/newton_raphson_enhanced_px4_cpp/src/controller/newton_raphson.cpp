#include "newton_raphson_enhanced_px4_cpp/controller/newton_raphson.hpp"
#include "newton_raphson_enhanced_px4_cpp/controller/nr_utils.hpp"

#include <stdexcept>

namespace newton_raphson_enhanced_px4_cpp::controller {

static constexpr bool USE_CBF = true;

NRProfileConfig build_nr_profile(const std::string& profile_name)
{
    if (profile_name == "baseline") {
        NRProfileConfig config;
        config.name = "baseline";
        config.lookahead_horizon_s = 1.2;
        config.alpha = (Eigen::Vector4d() << 30.0, 40.0, 40.0, 40.0).finished();
        config.integral_gain = Eigen::Vector4d::Zero();
        config.integral_limit = Eigen::Vector4d::Zero();
        config.num_iterations = 1;
        config.iteration_damping = 1.0;
        config.use_foh = false;
        return config;
    }
    if (profile_name == "workshop") {
        NRProfileConfig config;
        config.name = "workshop";
        config.lookahead_horizon_s = 0.8;
        config.alpha = (Eigen::Vector4d() << 30.0, 40.0, 40.0, 40.0).finished();
        config.integral_gain = (Eigen::Vector4d() << 0.35, 0.35, 0.50, 0.12).finished();
        config.integral_limit = (Eigen::Vector4d() << 0.75, 0.75, 0.50, 0.30).finished();
        config.num_iterations = 2;
        config.iteration_damping = 0.65;
        config.use_foh = true;
        return config;
    }
    throw std::runtime_error("Unknown Newton-Raphson enhanced profile: " + profile_name);
}

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
    const NRProfileConfig& profile)
{
    Eigen::Vector4d clipped_integral = error_integral.cwiseMin(profile.integral_limit)
        .cwiseMax(-profile.integral_limit);

    Eigen::Vector4d candidate_input = last_input;
    Eigen::Vector4d cbf_term = Eigen::Vector4d::Zero();

    for (int iteration = 0; iteration < profile.num_iterations; ++iteration) {
        Eigen::Vector4d y_pred = predict_output<double>(
            state,
            last_input,
            candidate_input,
            lookahead_horizon_s,
            lookahead_stage_dt,
            mass,
            profile.use_foh);

        Eigen::Vector4d regular_error = get_tracking_error(reference, y_pred)
            + profile.integral_gain.cwiseProduct(clipped_integral);

        JacPredXUInv jac = get_jac_pred_x_uinv(
            state,
            last_input,
            candidate_input,
            lookahead_horizon_s,
            lookahead_stage_dt,
            mass,
            profile.use_foh);

        Eigen::Vector4d regular_term = profile.alpha.cwiseProduct(regular_error);
        Eigen::Vector4d enhanced_error_term = get_enhanced_error(
            jac.jac_x,
            reference_rate,
            state,
            candidate_input,
            mass);

        Eigen::Vector4d nr_step = jac.dgdu_inv * (regular_term + enhanced_error_term);

        cbf_term = USE_CBF ? get_integral_cbf(candidate_input, nr_step) : Eigen::Vector4d::Zero();
        candidate_input += profile.iteration_damping * (nr_step + cbf_term) * integration_dt;
    }

    return {candidate_input, cbf_term};
}

}  // namespace newton_raphson_enhanced_px4_cpp::controller
