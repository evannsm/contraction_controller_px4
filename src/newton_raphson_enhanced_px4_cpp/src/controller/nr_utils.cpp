#include "newton_raphson_enhanced_px4_cpp/controller/nr_utils.hpp"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cmath>
#include <algorithm>

namespace newton_raphson_enhanced_px4_cpp::controller {

Eigen::Matrix4d get_jac_pred_u(const Eigen::Vector<double, 9>& state,
                                const Eigen::Vector4d& last_input,
                                const Eigen::Vector4d& candidate_input,
                                double T_lookahead,
                                double lookahead_step,
                                double mass,
                                bool use_foh) {
    using autodiff::real;
    using autodiff::jacobian;
    using autodiff::wrt;
    using autodiff::at;

    Eigen::Vector<real, 4> last_input_ad = last_input.cast<real>();
    Eigen::Vector<real, 4> candidate_input_ad = candidate_input.cast<real>();
    Eigen::Vector<real, 9> state_ad = state.cast<real>();

    Eigen::Vector<real, 4> F;
    Eigen::Matrix4d J = jacobian(
        [&](Eigen::Vector<real, 4>& u) -> Eigen::Vector<real, 4> {
            return predict_output<real>(
                state_ad,
                last_input_ad,
                u,
                T_lookahead,
                lookahead_step,
                mass,
                use_foh);
        },
        wrt(candidate_input_ad), at(candidate_input_ad), F);

    return J;
}

Eigen::Matrix4d get_inv_jac_pred_u(const Eigen::Vector<double, 9>& state,
                                    const Eigen::Vector4d& last_input,
                                    const Eigen::Vector4d& candidate_input,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass,
                                    bool use_foh) {
    Eigen::Matrix4d J = get_jac_pred_u(
        state, last_input, candidate_input, T_lookahead, lookahead_step, mass, use_foh);
    return J.completeOrthogonalDecomposition().pseudoInverse();
}

JacPredXUInv get_jac_pred_x_uinv(const Eigen::Vector<double, 9>& state,
                                  const Eigen::Vector4d& last_input,
                                  const Eigen::Vector4d& candidate_input,
                                  double T_lookahead,
                                  double lookahead_step,
                                  double mass,
                                  bool use_foh) {
    using autodiff::real;
    using autodiff::jacobian;
    using autodiff::wrt;
    using autodiff::at;

    Eigen::Vector<real, 4> last_input_ad = last_input.cast<real>();
    Eigen::Vector<real, 4> candidate_input_ad = candidate_input.cast<real>();
    Eigen::Vector<real, 9> state_ad = state.cast<real>();

    Eigen::Vector<real, 4> F;
    // Differentiate w.r.t state
    Eigen::Matrix<double, 4, 9> Jx = jacobian(
        [&](const Eigen::Vector<real, 9>& s) -> Eigen::Vector<real, 4> {
            return predict_output<real>(
                s,
                last_input_ad,
                candidate_input_ad,
                T_lookahead,
                lookahead_step,
                mass,
                use_foh);
        },
        wrt(state_ad), at(state_ad), F);

    // Differentiate w.r.t candidate_input
    Eigen::Matrix4d Ju = jacobian(
        [&](const Eigen::Vector<real, 4>& u) -> Eigen::Vector<real, 4> {
            return predict_output<real>(
                state_ad,
                last_input_ad,
                u,
                T_lookahead,
                lookahead_step,
                mass,
                use_foh);
        },
        wrt(candidate_input_ad), at(candidate_input_ad), F);

    return {Jx, Ju.completeOrthogonalDecomposition().pseudoInverse()};
}

Eigen::Vector4d get_enhanced_error(const Eigen::Matrix<double, 4, 9>& jac_x,
                                    const Eigen::Vector4d& rdot,
                                    const Eigen::Vector<double, 9>& state,
                                    const Eigen::Vector4d& candidate_input,
                                    double mass) {
    return rdot - jac_x * f_quad<double>(state, candidate_input, mass);
}

double execute_cbf(double current, double phi,
                   double max_value, double min_value,
                   double gamma, double switch_value) {
    double zeta_max = gamma * (max_value - current) - phi;
    double zeta_min = gamma * (min_value - current) - phi;

    if (current >= switch_value) {
        return std::min(0.0, zeta_max);
    } else {
        return std::max(0.0, zeta_min);
    }
}

Eigen::Vector4d get_integral_cbf(const Eigen::Vector4d& last_input,
                                  const Eigen::Vector4d& phi) {
    double curr_thrust = last_input[0];
    double curr_roll_rate = last_input[1];
    double curr_pitch_rate = last_input[2];
    double curr_yaw_rate = last_input[3];

    double phi_thrust = phi[0];
    double phi_roll_rate = phi[1];
    double phi_pitch_rate = phi[2];
    double phi_yaw_rate = phi[3];

    // Thrust CBF
    double thrust_gamma = 1.0;
    double thrust_max = 27.0;
    double thrust_min = 15.0;
    double switch_value = (thrust_max + thrust_min) / 2.0;
    double v_thrust = execute_cbf(curr_thrust, phi_thrust,
                                   thrust_max, thrust_min, thrust_gamma, switch_value);

    // Rates CBF
    double rates_max = 0.8;
    double rates_min = -0.8;
    double gamma_rates = 1.0;

    double v_roll  = execute_cbf(curr_roll_rate, phi_roll_rate,
                                  rates_max, rates_min, gamma_rates);
    double v_pitch = execute_cbf(curr_pitch_rate, phi_pitch_rate,
                                  rates_max, rates_min, gamma_rates);
    double v_yaw   = execute_cbf(curr_yaw_rate, phi_yaw_rate,
                                  rates_max, rates_min, gamma_rates);

    Eigen::Vector4d v;
    v << v_thrust, v_roll, v_pitch, v_yaw;
    return v;
}

// Quaternion helpers
static Eigen::Vector4d quaternion_from_yaw(double yaw) {
    double half = yaw / 2.0;
    return Eigen::Vector4d(std::cos(half), 0.0, 0.0, std::sin(half));
}

static Eigen::Vector4d quaternion_conjugate(const Eigen::Vector4d& q) {
    return Eigen::Vector4d(q[0], -q[1], -q[2], -q[3]);
}

static Eigen::Vector4d quaternion_multiply(const Eigen::Vector4d& q1,
                                            const Eigen::Vector4d& q2) {
    double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    return Eigen::Vector4d(
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    );
}

static double yaw_error_from_quaternion(const Eigen::Vector4d& q) {
    return 2.0 * std::atan2(q[3], q[0]);
}

static Eigen::Vector4d quaternion_normalize(const Eigen::Vector4d& q) {
    return q / q.norm();
}

double shortest_path_yaw_quaternion(double current_yaw, double desired_yaw) {
    Eigen::Vector4d q_current = quaternion_normalize(quaternion_from_yaw(current_yaw));
    Eigen::Vector4d q_desired = quaternion_normalize(quaternion_from_yaw(desired_yaw));
    Eigen::Vector4d q_error = quaternion_multiply(q_desired, quaternion_conjugate(q_current));
    Eigen::Vector4d q_error_normalized = quaternion_normalize(q_error);
    return yaw_error_from_quaternion(q_error_normalized);
}

Eigen::Vector4d get_tracking_error(const Eigen::Vector4d& ref,
                                    const Eigen::Vector4d& pred) {
    Eigen::Vector4d err = ref - pred;
    err[3] = shortest_path_yaw_quaternion(pred[3], ref[3]);
    return err;
}

}  // namespace newton_raphson_enhanced_px4_cpp::controller
