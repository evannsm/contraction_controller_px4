#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace newton_raphson_enhanced_px4_cpp::controller {

constexpr double GRAVITY = 9.8;  // Match Gazebo world

// Output selection matrix C: extracts [x, y, z, yaw] from 9D state
inline Eigen::Matrix<double, 4, 9> get_C_matrix() {
    Eigen::Matrix<double, 4, 9> C = Eigen::Matrix<double, 4, 9>::Zero();
    C(0, 0) = 1.0;  // x
    C(1, 1) = 1.0;  // y
    C(2, 2) = 1.0;  // z
    C(3, 8) = 1.0;  // yaw
    return C;
}

// Templated dynamics for autodiff propagation
template<typename T>
Eigen::Vector<T, 3> body2world_angular_rates(T roll, T pitch,
                                              const Eigen::Vector<T, 3>& body_rates) {
    using std::sin; using std::cos; using std::tan;
    T sr = sin(roll), cr = cos(roll);
    T sp = sin(pitch), cp = cos(pitch);
    T tp = tan(pitch);

    Eigen::Matrix<T, 3, 3> Tmat;
    Tmat << T(1.0), sr * tp,       cr * tp,
            T(0.0), cr,           -sr,
            T(0.0), sr / cp,       cr / cp;

    return Tmat * body_rates;
}

template<typename T>
Eigen::Vector<T, 9> f_quad(const Eigen::Vector<T, 9>& state,
                            const Eigen::Vector<T, 4>& input,
                            double mass) {
    using std::sin; using std::cos;

    T x = state[0], y = state[1], z = state[2];
    T vx = state[3], vy = state[4], vz = state[5];
    T roll = state[6], pitch = state[7], yaw = state[8];

    T curr_thrust = input[0];
    Eigen::Vector<T, 3> body_rates = input.template segment<3>(1);

    Eigen::Vector<T, 3> world_rates = body2world_angular_rates(roll, pitch, body_rates);
    T curr_rolldot = world_rates[0];
    T curr_pitchdot = world_rates[1];
    T curr_yawdot = world_rates[2];

    T sr = sin(roll), cr = cos(roll);
    T sp = sin(pitch), cp = cos(pitch);
    T sy = sin(yaw), cy = cos(yaw);

    T m_inv = T(1.0 / mass);
    T vxdot = -(curr_thrust * m_inv) * (sr * sy + cr * cy * sp);
    T vydot = -(curr_thrust * m_inv) * (cr * sy * sp - cy * sr);
    T vzdot = T(GRAVITY) - (curr_thrust * m_inv) * (cr * cp);

    Eigen::Vector<T, 9> xdot;
    xdot << vx, vy, vz, vxdot, vydot, vzdot, curr_rolldot, curr_pitchdot, curr_yawdot;
    return xdot;
}

template<typename T>
Eigen::Vector<T, 4> interpolate_input(const Eigen::Vector<T, 4>& u_prev,
                                      const Eigen::Vector<T, 4>& u_next,
                                      double progress,
                                      bool use_foh) {
    progress = std::clamp(progress, 0.0, 1.0);
    if (!use_foh) {
        return u_next;
    }
    return u_prev + (u_next - u_prev) * T(progress);
}

template<typename T>
Eigen::Vector<T, 9> rk4_pred(const Eigen::Vector<T, 9>& state,
                               const Eigen::Vector<T, 4>& u_prev,
                               const Eigen::Vector<T, 4>& u_next,
                               double lookahead_step,
                               int integrations_int,
                               double mass,
                               bool use_foh) {
    Eigen::Vector<T, 9> current = state;
    T dt = T(lookahead_step);
    const double total_steps = static_cast<double>(std::max(1, integrations_int));

    auto input_at = [&](int i, double stage_fraction) {
        double progress = (static_cast<double>(i) + stage_fraction) / total_steps;
        return interpolate_input(u_prev, u_next, progress, use_foh);
    };

    for (int i = 0; i < integrations_int; ++i) {
        Eigen::Vector<T, 9> k1 = f_quad(current, input_at(i, 0.0), mass);
        Eigen::Vector<T, 9> s2 = (current + k1 * dt / T(2.0)).eval();
        Eigen::Vector<T, 9> k2 = f_quad(s2, input_at(i, 0.5), mass);
        Eigen::Vector<T, 9> s3 = (current + k2 * dt / T(2.0)).eval();
        Eigen::Vector<T, 9> k3 = f_quad(s3, input_at(i, 0.5), mass);
        Eigen::Vector<T, 9> s4 = (current + k3 * dt).eval();
        Eigen::Vector<T, 9> k4 = f_quad(s4, input_at(i, 1.0), mass);
        current = current + (k1 + T(2.0) * k2 + T(2.0) * k3 + k4) * dt / T(6.0);
    }
    return current;
}

template<typename T>
Eigen::Vector<T, 9> predict_state(const Eigen::Vector<T, 9>& state,
                                    const Eigen::Vector<T, 4>& u_prev,
                                    const Eigen::Vector<T, 4>& u_next,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass,
                                    bool use_foh) {
    int integrations_int = static_cast<int>(T_lookahead / lookahead_step);
    return rk4_pred(state, u_prev, u_next, lookahead_step, integrations_int, mass, use_foh);
}

template<typename T>
Eigen::Vector<T, 4> predict_output(const Eigen::Vector<T, 9>& state,
                                    const Eigen::Vector<T, 4>& u_prev,
                                    const Eigen::Vector<T, 4>& u_next,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass,
                                    bool use_foh) {
    Eigen::Vector<T, 9> pred = predict_state(
        state, u_prev, u_next, T_lookahead, lookahead_step, mass, use_foh);
    // Extract [x, y, z, yaw]
    Eigen::Vector<T, 4> output;
    output << pred[0], pred[1], pred[2], pred[8];
    return output;
}

// Non-template functions (Jacobian, CBF, quaternion yaw error)
Eigen::Matrix4d get_jac_pred_u(const Eigen::Vector<double, 9>& state,
                                const Eigen::Vector4d& last_input,
                                const Eigen::Vector4d& candidate_input,
                                double T_lookahead,
                                double lookahead_step,
                                double mass,
                                bool use_foh);

Eigen::Matrix4d get_inv_jac_pred_u(const Eigen::Vector<double, 9>& state,
                                    const Eigen::Vector4d& last_input,
                                    const Eigen::Vector4d& candidate_input,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass,
                                    bool use_foh);

struct JacPredXUInv {
    Eigen::Matrix<double, 4, 9> jac_x;
    Eigen::Matrix4d dgdu_inv;
};

JacPredXUInv get_jac_pred_x_uinv(const Eigen::Vector<double, 9>& state,
                                  const Eigen::Vector4d& last_input,
                                  const Eigen::Vector4d& candidate_input,
                                  double T_lookahead,
                                  double lookahead_step,
                                  double mass,
                                  bool use_foh);

Eigen::Vector4d get_enhanced_error(const Eigen::Matrix<double, 4, 9>& jac_x,
                                    const Eigen::Vector4d& rdot,
                                    const Eigen::Vector<double, 9>& state,
                                    const Eigen::Vector4d& candidate_input,
                                    double mass);

double execute_cbf(double current, double phi,
                   double max_value, double min_value,
                   double gamma, double switch_value = 0.0);

Eigen::Vector4d get_integral_cbf(const Eigen::Vector4d& last_input,
                                  const Eigen::Vector4d& phi);

double shortest_path_yaw_quaternion(double current_yaw, double desired_yaw);

Eigen::Vector4d get_tracking_error(const Eigen::Vector4d& ref,
                                    const Eigen::Vector4d& pred);

}  // namespace newton_raphson_enhanced_px4_cpp::controller
