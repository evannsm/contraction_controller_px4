#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace newton_raphson_px4_cpp::controller {

constexpr double GRAVITY = 9.806;

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
Eigen::Vector<T, 9> rk4_pred(const Eigen::Vector<T, 9>& state,
                               const Eigen::Vector<T, 4>& input,
                               double lookahead_step,
                               int integrations_int,
                               double mass) {
    Eigen::Vector<T, 9> current = state;
    T dt = T(lookahead_step);
    for (int i = 0; i < integrations_int; ++i) {
        Eigen::Vector<T, 9> k1 = f_quad(current, input, mass);
        Eigen::Vector<T, 9> s2 = (current + k1 * dt / T(2.0)).eval();
        Eigen::Vector<T, 9> k2 = f_quad(s2, input, mass);
        Eigen::Vector<T, 9> s3 = (current + k2 * dt / T(2.0)).eval();
        Eigen::Vector<T, 9> k3 = f_quad(s3, input, mass);
        Eigen::Vector<T, 9> s4 = (current + k3 * dt).eval();
        Eigen::Vector<T, 9> k4 = f_quad(s4, input, mass);
        current = current + (k1 + T(2.0) * k2 + T(2.0) * k3 + k4) * dt / T(6.0);
    }
    return current;
}

template<typename T>
Eigen::Vector<T, 9> predict_state(const Eigen::Vector<T, 9>& state,
                                    const Eigen::Vector<T, 4>& u,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass) {
    int integrations_int = static_cast<int>(T_lookahead / lookahead_step);
    return rk4_pred(state, u, lookahead_step, integrations_int, mass);
}

template<typename T>
Eigen::Vector<T, 4> predict_output(const Eigen::Vector<T, 9>& state,
                                    const Eigen::Vector<T, 4>& u,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass) {
    Eigen::Vector<T, 9> pred = predict_state(state, u, T_lookahead, lookahead_step, mass);
    // Extract [x, y, z, yaw]
    Eigen::Vector<T, 4> output;
    output << pred[0], pred[1], pred[2], pred[8];
    return output;
}

// Non-template functions (Jacobian, CBF, quaternion yaw error)
Eigen::Matrix4d get_jac_pred_u(const Eigen::Vector<double, 9>& state,
                                const Eigen::Vector4d& last_input,
                                double T_lookahead,
                                double lookahead_step,
                                double mass);

Eigen::Matrix4d get_inv_jac_pred_u(const Eigen::Vector<double, 9>& state,
                                    const Eigen::Vector4d& last_input,
                                    double T_lookahead,
                                    double lookahead_step,
                                    double mass);

double execute_cbf(double current, double phi,
                   double max_value, double min_value,
                   double gamma, double switch_value = 0.0);

Eigen::Vector4d get_integral_cbf(const Eigen::Vector4d& last_input,
                                  const Eigen::Vector4d& phi);

double shortest_path_yaw_quaternion(double current_yaw, double desired_yaw);

Eigen::Vector4d get_tracking_error(const Eigen::Vector4d& ref,
                                    const Eigen::Vector4d& pred);

}  // namespace newton_raphson_px4_cpp::controller
