#include "nr_diff_flat_px4_cpp/controller/newton_raphson.hpp"
#include "nr_diff_flat_px4_cpp/controller/nr_utils.hpp"

#include <stdexcept>
#include <iostream>

namespace nr_diff_flat_px4_cpp::controller {

static constexpr double CBF_GAMMA = 10.0;
static constexpr double CBF_THRUST_MIN = 3.0;
static constexpr double CBF_THRUST_MAX = 27.0;
static constexpr double CLIP_THRUST_MIN = 0.5;
static constexpr double CLIP_THRUST_MAX = 27.0;

NRProfileConfig build_nr_profile(const std::string& profile_name)
{
    if (profile_name == "baseline") {
        NRProfileConfig config;
        config.name = "baseline";
        config.lookahead_horizon_s = 0.8;
        config.alpha = (Eigen::Vector4d() << 20.0, 30.0, 30.0, 30.0).finished();
        config.integral_gain = Eigen::Vector4d::Zero();
        config.integral_limit = Eigen::Vector4d::Zero();
        config.num_iterations = 1;
        config.iteration_damping = 1.0;
        config.use_thrust_cbf = true;
        return config;
    }
    if (profile_name == "workshop") {
        NRProfileConfig config;
        config.name = "workshop";
        config.lookahead_horizon_s = 0.5;
        config.alpha = (Eigen::Vector4d() << 24.0, 34.0, 34.0, 24.0).finished();
        config.integral_gain = (Eigen::Vector4d() << 0.30, 0.30, 0.45, 0.10).finished();
        config.integral_limit = (Eigen::Vector4d() << 0.75, 0.75, 0.50, 0.30).finished();
        config.num_iterations = 2;
        config.iteration_damping = 0.65;
        config.use_thrust_cbf = true;
        return config;
    }
    throw std::runtime_error("Unknown NR diff-flat profile: " + profile_name);
}

static Eigen::Vector4d thrust_cbf(double curr_thrust,
                                  const Eigen::Vector3d& u_df_xyz,
                                  const Eigen::Vector3d& sigmad2_xyz,
                                  double mass)
{
    Eigen::Vector3d a3(0.0, 0.0, -1.0);
    Eigen::Vector3d grav_vector = GRAVITY * a3;

    double tau = curr_thrust;
    double h = -(5.0 / 72.0) * (tau - CBF_THRUST_MIN) * (tau - CBF_THRUST_MAX);
    double dhdtau = -(5.0 / 72.0) * (2.0 * tau - (CBF_THRUST_MIN + CBF_THRUST_MAX));

    Eigen::Vector3d accels = sigmad2_xyz + grav_vector;
    double accels_norm = std::max(accels.norm(), 1e-8);
    Eigen::Vector3d xT_over_normx = accels / accels_norm;
    Eigen::Vector3d phi = dhdtau * mass * xT_over_normx;

    double d = (-phi.transpose() * u_df_xyz).value() - CBF_GAMMA * h;
    double phi_norm_sq = std::max(phi.squaredNorm(), 1e-12);

    Eigen::Vector3d v_xyz = (d / phi_norm_sq) * phi;
    if (d <= 0.0) {
        v_xyz.setZero();
    }

    Eigen::Vector4d v;
    v << v_xyz, 0.0;
    return v;
}

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
    const NRProfileConfig& profile)
{
    double curr_x = state[0], curr_y = state[1], curr_z = state[2], curr_yaw = state[8];
    double curr_vx = state[3], curr_vy = state[4], curr_vz = state[5];
    double curr_yawdot = last_input[3];
    double curr_thrust = last_input[0];

    Eigen::Vector4d z1(curr_x, curr_y, curr_z, curr_yaw);
    Eigen::Vector4d z2(curr_vx, curr_vy, curr_vz, curr_yawdot);
    Eigen::Vector4d z3_initial = x_df.segment<4>(8);

    Eigen::Matrix4d dgdu_inv = (2.0 / (lookahead_horizon_s * lookahead_horizon_s)) * Eigen::Matrix4d::Identity();
    Eigen::Vector4d clipped_integral = error_integral.cwiseMin(profile.integral_limit)
        .cwiseMax(-profile.integral_limit);

    Eigen::Vector4d candidate_z3 = z3_initial;
    Eigen::Vector4d corrected_u_df = Eigen::Vector4d::Zero();
    Eigen::Vector4d cbf_term = Eigen::Vector4d::Zero();

    for (int iteration = 0; iteration < profile.num_iterations; ++iteration) {
        Eigen::Vector4d pred = z1 + z2 * lookahead_horizon_s + 0.5 * candidate_z3 * lookahead_horizon_s * lookahead_horizon_s;
        Eigen::Vector4d error = get_tracking_error(reference, pred)
            + profile.integral_gain.cwiseProduct(clipped_integral);

        Eigen::Vector4d nr_step = dgdu_inv * error;
        Eigen::Vector4d raw_u_df = profile.alpha.cwiseProduct(nr_step);

        if (profile.use_thrust_cbf) {
            cbf_term = thrust_cbf(curr_thrust, raw_u_df.head<3>(), candidate_z3.head<3>(), mass);
        } else {
            cbf_term.setZero();
        }

        corrected_u_df = raw_u_df + cbf_term;
        candidate_z3 += profile.iteration_damping * corrected_u_df * integration_dt;
    }

    // Update x_df
    Eigen::Matrix<double, 12, 12> A_df = Eigen::Matrix<double, 12, 12>::Zero();
    A_df.block<4, 4>(0, 4) = Eigen::Matrix4d::Identity();
    A_df.block<4, 4>(4, 8) = Eigen::Matrix4d::Identity();

    Eigen::Matrix<double, 12, 4> B_df = Eigen::Matrix<double, 12, 4>::Zero();
    B_df.block<4, 4>(8, 0) = Eigen::Matrix4d::Identity();

    Eigen::Matrix<double, 12, 1> x_dot_df = A_df * x_df + B_df * corrected_u_df;
    Eigen::Matrix<double, 12, 1> next_x_df = x_df + x_dot_df * integration_dt;

    Eigen::Vector4d sigmad2 = next_x_df.segment<4>(8);
    Eigen::Vector4d sigmad3 = corrected_u_df;

    Eigen::Vector3d a3(0.0, 0.0, -1.0);
    Eigen::Vector3d accels = sigmad2.head<3>() + GRAVITY * a3;
    double thrust = mass * std::max(accels.norm(), 1e-8);
    double clipped_thrust = profile.use_thrust_cbf ? thrust : std::clamp(thrust, CLIP_THRUST_MIN, CLIP_THRUST_MAX);

    Eigen::Vector3d b3 = accels / std::max(accels.norm(), 1e-8);
    Eigen::Vector3d a1(1.0, 0.0, 0.0);
    Eigen::Vector3d a2(0.0, 1.0, 0.0);
    Eigen::Vector3d e1 = std::cos(curr_yaw) * a1 + std::sin(curr_yaw) * a2;
    Eigen::Vector3d val2 = b3.cross(e1);
    Eigen::Vector3d b2 = val2 / std::max(val2.norm(), 1e-8);
    Eigen::Vector3d b1 = b2.cross(b3);

    Eigen::Vector3d jerk = sigmad3.head<3>();
    Eigen::Vector3d val3 = jerk - (jerk.transpose() * b3).value() * b3;
    double p = (b2.transpose() * ((mass / -std::max(clipped_thrust, 1e-8)) * val3)).value();
    double q = (b1.transpose() * ((mass / -std::max(clipped_thrust, 1e-8)) * val3)).value();

    // Solving for r
    Eigen::Matrix3d Amat;
    Amat.col(0) = e1;
    Amat.col(1) = b2;
    Amat.col(2) = a3;

    Eigen::Matrix3d Bmat = rot_matrix;
    Eigen::Vector2d y_known(p, q);
    Eigen::Vector3d rhs = (Bmat.leftCols<2>() * y_known) - (Amat.rightCols<1>() * yaw_dot);

    Eigen::Matrix<double, 3, 3> Mmat;
    Mmat.leftCols<2>() = Amat.leftCols<2>();
    Mmat.rightCols<1>() = -Bmat.rightCols<1>();

    Eigen::Vector3d unknowns = Mmat.colPivHouseholderQr().solve(rhs);
    double r = -unknowns[2];

    double max_rate = 0.8;
    p = std::clamp(p, -max_rate, max_rate);
    q = std::clamp(q, -max_rate, max_rate);
    r = std::clamp(r, -max_rate, max_rate);

    Eigen::Vector4d u;
    u << clipped_thrust, p, q, r;

    return {u, next_x_df, cbf_term};
}

}  // namespace nr_diff_flat_px4_cpp::controller
