#include "nr_diff_flat_px4_cpp/offboard_control_node.hpp"
#include "nr_diff_flat_px4_cpp/px4_utils/core_funcs.hpp"
#include "nr_diff_flat_px4_cpp/controller/newton_raphson.hpp"
#include "nr_diff_flat_px4_cpp/controller/nr_utils.hpp"
#include "quad_trajectories_cpp/registry.hpp"
#include "quad_trajectories_cpp/utils.hpp"

#include <cmath>
#include <thread>
#include <iostream>

namespace nr_diff_flat_px4_cpp {

using namespace std::chrono_literals;
namespace qp = quad_platforms_cpp;
namespace qt = quad_trajectories_cpp;
namespace ctrl = nr_diff_flat_px4_cpp::controller;

static const std::string BANNER = "\n============================================================\n";

// --- Feedforward helper ---------------------------------------------------
struct FfState {
    Eigen::Vector<double, 10> x;
    Eigen::Vector4d u;
};

static FfState flat_to_x_u_f8(double t, bool sim) {
    constexpr double g = 9.8;
    const double height = sim ? 3.0 : 0.85;
    const double R      = sim ? 2.0 : 0.5;
    const double Tp     = 10.0;

    const double w1 = 2.0 * M_PI / Tp;
    const double w2 = 4.0 * M_PI / Tp;

    double px = R * std::sin(w1 * t);
    double py = (R / 2.0) * std::sin(w2 * t);
    double pz = -height;
    double psi = 0.0;

    double vx   = R * w1 * std::cos(w1 * t);
    double vy   = (R / 2.0) * w2 * std::cos(w2 * t);
    double vz   = 0.0;

    double ax = -R * w1 * w1 * std::sin(w1 * t);
    double ay = -(R / 2.0) * w2 * w2 * std::sin(w2 * t);
    double az = 0.0;

    double jx = -R * w1 * w1 * w1 * std::cos(w1 * t);
    double jy = -(R / 2.0) * w2 * w2 * w2 * std::cos(w2 * t);
    double jz = 0.0;

    double f  = std::sqrt(ax*ax + ay*ay + (az - g)*(az - g));
    double th  = std::asin(-ax / f);
    double phi = std::atan2(ay, g - az);

    double df  = (ax*jx + ay*jy + (az - g)*jz) / f;
    double dth = (1.0 / std::cos(th)) * (-jx + ax * df / f) / f;
    double dphi = ((g - az)*jy + ay*jz) / ((g - az)*(g - az) + ay*ay);

    FfState ff;
    ff.x << px, py, pz, vx, vy, vz, f, phi, th, psi;
    ff.u << df, dphi, dth, 0.0;
    return ff;
}

static void quat_to_euler(double w, double x, double y, double z,
                            double& roll, double& pitch, double& yaw) {
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

OffboardControlNode::OffboardControlNode(
    qp::PlatformType platform_type,
    qt::TrajectoryType trajectory,
    std::optional<int> hover_mode,
    bool double_speed,
    bool short_variant,
    bool spin,
    bool logging_enabled,
    std::string log_file,
    std::optional<double> flight_period_override,
    bool feedforward,
    std::string nr_profile_name)
    : Node("offboard_control_node"),
      sim_(platform_type == qp::PlatformType::SIM),
      platform_type_(platform_type),
      trajectory_type_(trajectory),
      hover_mode_(hover_mode),
      double_speed_(double_speed),
      short_variant_(short_variant),
      spin_(spin),
      logging_enabled_(logging_enabled),
      log_file_(std::move(log_file)),
      feedforward_(feedforward),
      nr_profile_name_(std::move(nr_profile_name)),
      nr_profile_(ctrl::build_nr_profile(nr_profile_name_)),
      offboard_mode_rc_switch_on_(sim_)
{
    RCLCPP_INFO(get_logger(), "%sInitializing ROS 2 node: 'OffboardControlNode'%s",
                BANNER.c_str(), BANNER.c_str());

    platform_ = qp::create_platform(platform_type_);

    std::cout << "\n[Trajectory] Main trajectory type: "
              << qt::trajectory_type_name(trajectory_type_) << std::endl;
    T_LOOKAHEAD_ = nr_profile_.lookahead_horizon_s;
    std::cout << "[NR Diff-Flat Profile] " << nr_profile_.name
              << ": lookahead=" << T_LOOKAHEAD_ << "s, "
              << "iterations=" << nr_profile_.num_iterations << std::endl;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()
        .transient_local();

    offboard_control_mode_publisher = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    trajectory_setpoint_publisher = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos);
    rates_setpoint_publisher = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", qos);
    vehicle_command_publisher = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos);

    vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        qos,
        std::bind(&OffboardControlNode::vehicle_odometry_callback, this, std::placeholders::_1));

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        qos,
        std::bind(&OffboardControlNode::vehicle_status_callback, this, std::placeholders::_1));

    rc_channels_sub_ = create_subscription<px4_msgs::msg::RcChannels>(
        "/fmu/out/rc_channels",
        qos,
        std::bind(&OffboardControlNode::rc_channel_callback, this, std::placeholders::_1));

    flight_period_ = flight_period_override.value_or(sim_ ? 30.0 : 60.0);
    land_time_ = flight_period_ + 2.0 * cushion_period_;
    T0_ = std::chrono::steady_clock::now();
    flight_phase_ = get_phase();

    offboard_timer_ = create_wall_timer(100ms,
        std::bind(&OffboardControlNode::offboard_mode_timer_callback, this));
    publish_control_timer_ = create_wall_timer(10ms,
        std::bind(&OffboardControlNode::publish_control_timer_callback, this));
    compute_control_timer_ = create_wall_timer(10ms,
        std::bind(&OffboardControlNode::compute_control_timer_callback, this));

    if (logging_enabled_) {
        data_log_timer_ = create_wall_timer(100ms,
            std::bind(&OffboardControlNode::data_log_timer_callback, this));
    }

    HOVER_HEIGHT_ = sim_ ? 3.0 : 0.7;
    LAND_HEIGHT_  = sim_ ? 0.6 : 0.45;

    first_thrust_ = platform_->mass() * ctrl::GRAVITY;
    last_input_ << first_thrust_, 0.0, 0.0, 0.0;
    new_input_ = last_input_;
    normalized_input_ = {platform_->get_throttle_from_force(first_thrust_), 0.0, 0.0, 0.0};
    ref_ = Eigen::Vector4d::Zero();

    time_controller();

    std::cout << "[Offboard Control Node] Node initialized successfully!\n" << std::endl;
    std::this_thread::sleep_for(3s);

    T0_ = std::chrono::steady_clock::now();

    if (logging_enabled_) {
        csv_logger_ = std::make_unique<ros2_logger_cpp::CsvLogger>();
        const char* col_names[] = {
            "platform", "controller", "trajectory",
            "traj_double", "traj_short", "traj_spin", "lookahead_time",
            "time", "traj_time", "ref_time", "comp_time",
            "x", "y", "z", "yaw",
            "vx", "vy", "vz",
            "x_ref", "y_ref", "z_ref", "yaw_ref",
            "vx_ref", "vy_ref", "vz_ref",
            "p", "q", "r",
            "throttle_input", "p_input", "q_input", "r_input",
            "cbf_v_throttle", "cbf_v_p", "cbf_v_q", "cbf_v_r"
        };
        for (const auto& name : col_names) {
            csv_logger_->add_header(name);
        }

        csv_logger_->set_const_string(COL_PLATFORM,    sim_ ? "SIM" : "HW");
        csv_logger_->set_const_string(COL_CONTROLLER,  "nr_diff_flat_cpp");
        csv_logger_->set_const_string(COL_TRAJECTORY,  qt::trajectory_type_name(trajectory_type_));
        csv_logger_->set_const_string(COL_TRAJ_DOUBLE, double_speed_ ? "DblSpd" : "NormSpd");
        csv_logger_->set_const_string(COL_TRAJ_SHORT,  short_variant_ ? "Short" : "Not Short");
        csv_logger_->set_const_string(COL_TRAJ_SPIN,   spin_ ? "Spin" : "NoSpin");
    }
}

OffboardControlNode::~OffboardControlNode() {
    if (csv_logger_ && !log_file_.empty()) {
        csv_logger_->save(log_file_);
    }
}

void OffboardControlNode::vehicle_odometry_callback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    x_ = msg->position[0];
    y_ = msg->position[1];
    z_ = msg->position[2];
    vx_ = msg->velocity[0];
    vy_ = msg->velocity[1];
    vz_ = msg->velocity[2];
    p_ = msg->angular_velocity[0];
    q_ = msg->angular_velocity[1];
    r_ = msg->angular_velocity[2];

    double qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
    quat_to_euler(qw, qx, qy, qz, roll_, pitch_, raw_yaw_);
    yaw_ = yaw_tracker_.adjust_yaw(raw_yaw_);
    mocap_initialized_ = yaw_tracker_.initialized;

    nr_state_ << x_, y_, z_, vx_, vy_, vz_, roll_, pitch_, yaw_;

    Eigen::Quaterniond q_eigen(qw, qx, qy, qz);
    rot_matrix_ = q_eigen.toRotationMatrix();
}

void OffboardControlNode::vehicle_status_callback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    in_offboard_mode_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    in_land_mode_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
}

void OffboardControlNode::rc_channel_callback(
    const px4_msgs::msg::RcChannels::SharedPtr msg) {
    float flight_mode = msg->channels[mode_channel_ - 1];
    offboard_mode_rc_switch_on_ = (flight_mode >= 0.75f);
}

px4_utils::FlightPhase OffboardControlNode::get_phase() const {
    if (program_time_ < cushion_period_)
        return px4_utils::FlightPhase::HOVER;
    else if (program_time_ < cushion_period_ + flight_period_)
        return px4_utils::FlightPhase::CUSTOM;
    else if (program_time_ < land_time_)
        return px4_utils::FlightPhase::RETURN;
    else
        return px4_utils::FlightPhase::LAND;
}

double OffboardControlNode::time_before_next_phase(px4_utils::FlightPhase phase) const {
    switch (phase) {
        case px4_utils::FlightPhase::HOVER:  return cushion_period_ - program_time_;
        case px4_utils::FlightPhase::CUSTOM: return (cushion_period_ + flight_period_) - program_time_;
        case px4_utils::FlightPhase::RETURN: return land_time_ - program_time_;
        case px4_utils::FlightPhase::LAND:   return 0.0;
    }
    return 0.0;
}

bool OffboardControlNode::killswitch_and_flight_phase() {
    if (!offboard_mode_rc_switch_on_) {
        offboard_setpoint_counter_ = 0;
        return false;
    }
    auto now = std::chrono::steady_clock::now();
    program_time_ = std::chrono::duration<double>(now - T0_).count();
    flight_phase_ = get_phase();
    return true;
}

bool OffboardControlNode::get_offboard_health() const {
    return in_offboard_mode_ && armed_ && mocap_initialized_;
}

void OffboardControlNode::offboard_mode_timer_callback() {
    if (!killswitch_and_flight_phase()) return;
    if (offboard_setpoint_counter_ == 10) {
        px4_utils::engage_offboard_mode(*this);
        px4_utils::arm(*this);
    }
    if (offboard_setpoint_counter_ < 11) offboard_setpoint_counter_++;

    switch (flight_phase_) {
        case px4_utils::FlightPhase::HOVER:
        case px4_utils::FlightPhase::RETURN:
        case px4_utils::FlightPhase::LAND:
            px4_utils::publish_offboard_heartbeat_position(*this);
            break;
        case px4_utils::FlightPhase::CUSTOM:
            px4_utils::publish_offboard_heartbeat_bodyrate(*this);
            break;
    }
}

void OffboardControlNode::publish_position_setpoint(double x, double y, double z, double yaw) {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    msg.yaw = static_cast<float>(yaw);
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
}

void OffboardControlNode::publish_rates_setpoint(double thrust, double roll, double pitch, double yaw) {
    auto msg = px4_msgs::msg::VehicleRatesSetpoint();
    msg.roll = static_cast<float>(roll);
    msg.pitch = static_cast<float>(pitch);
    msg.yaw = static_cast<float>(yaw);
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = static_cast<float>(-thrust);
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    rates_setpoint_publisher->publish(msg);
}

void OffboardControlNode::publish_control_timer_callback() {
    if (in_land_mode_) {
        double threshold = sim_ ? 0.71 : 0.64;
        if (std::abs(z_) < threshold) {
            px4_utils::disarm(*this);
            rclcpp::shutdown();
            return;
        }
    }
    if (!killswitch_and_flight_phase()) return;
    if (!get_offboard_health()) return;

    switch (flight_phase_) {
        case px4_utils::FlightPhase::HOVER:  publish_position_setpoint(0.0, 0.0, -HOVER_HEIGHT_, 0.0); break;
        case px4_utils::FlightPhase::CUSTOM: publish_rates_setpoint(normalized_input_[0], normalized_input_[1],
                                                                    normalized_input_[2], normalized_input_[3]); break;
        case px4_utils::FlightPhase::RETURN: publish_position_setpoint(0.0, 0.0, -HOVER_HEIGHT_, 0.0); break;
        case px4_utils::FlightPhase::LAND:
            publish_position_setpoint(0.0, 0.0, -LAND_HEIGHT_, 0.0);
            if (std::abs(z_) < 0.64) px4_utils::land(*this);
            break;
    }
}

void OffboardControlNode::compute_control_timer_callback() {
    if (!killswitch_and_flight_phase()) return;
    if (!get_offboard_health()) return;
    if (get_phase() != px4_utils::FlightPhase::CUSTOM) return;

    if (!trajectory_started_) {
        trajectory_T0_ = std::chrono::steady_clock::now();
        trajectory_time_ = 0.0;
        trajectory_started_ = true;
        nr_error_integral_.setZero();
        x_df_.setZero();
        x_df_.head<3>() << x_, y_, z_;
        x_df_[3] = yaw_;
        x_df_.segment<3>(4) << vx_, vy_, vz_;
        x_df_[7] = r_; // approximate yawdot
    }

    auto now = std::chrono::steady_clock::now();
    trajectory_time_ = std::chrono::duration<double>(now - trajectory_T0_).count();
    reference_time_ = trajectory_time_ + T_LOOKAHEAD_;

    ref_ = generate_ref_position(trajectory_type_, std::nullopt, reference_time_);
    Eigen::Vector4d ref_now = generate_ref_position(trajectory_type_, std::nullopt, trajectory_time_);
    update_nr_error_integral(ref_now);
    {
        qt::TrajContext ctx;
        ctx.sim = sim_;
        ctx.hover_mode = hover_mode_;
        ctx.spin = spin_;
        ctx.double_speed = (trajectory_type_ == qt::TrajectoryType::FIG8_CONTRACTION) ? false : double_speed_;
        ctx.short_variant = short_variant_;
        ref_vel_ = qt::get_velocity(trajectory_type_, reference_time_, ctx);
    }

    auto t0 = std::chrono::steady_clock::now();
    controller();
    auto t1 = std::chrono::steady_clock::now();
    compute_time_ = std::chrono::duration<double>(t1 - t0).count();

    last_input_ = new_input_;
    normalized_input_ = {platform_->get_throttle_from_force(new_input_[0]),
                         new_input_[1], new_input_[2], new_input_[3]};
}

void OffboardControlNode::controller() {
    auto result = ctrl::nr_tracker_flat(
        nr_state_, last_input_, x_df_, ref_, nr_error_integral_,
        T_LOOKAHEAD_, compute_dt_, platform_->mass(),
        rot_matrix_, r_, nr_profile_);
    new_input_ = result.u;
    x_df_ = result.x_df;
    cbf_term_ = result.v;
}

void OffboardControlNode::update_nr_error_integral(const Eigen::Vector4d& ref_now) {
    Eigen::Vector4d actual_output;
    actual_output << x_, y_, z_, yaw_;
    Eigen::Vector4d current_error = ctrl::get_tracking_error(ref_now, actual_output);
    nr_error_integral_ += current_error * compute_dt_;
    nr_error_integral_ = nr_error_integral_.cwiseMin(nr_profile_.integral_limit)
        .cwiseMax(-nr_profile_.integral_limit);
}

Eigen::Vector4d OffboardControlNode::generate_ref_position(
    qt::TrajectoryType type,
    std::optional<int> hover_mode_override,
    std::optional<double> t_start_override) {
    qt::TrajContext ctx;
    ctx.sim = sim_;
    ctx.hover_mode = hover_mode_override ? hover_mode_override : hover_mode_;
    ctx.spin = spin_;
    ctx.double_speed = (type == qt::TrajectoryType::FIG8_CONTRACTION) ? false : double_speed_;
    ctx.short_variant = short_variant_;
    auto fn = qt::get_trajectory_fn(type);
    return fn(t_start_override.value_or(reference_time_), ctx);
}

void OffboardControlNode::data_log_timer_callback() {
    if (flight_phase_ != px4_utils::FlightPhase::CUSTOM) return;
    if (!csv_logger_) return;
    csv_logger_->new_row();
    csv_logger_->set(COL_TIME, program_time_);
    csv_logger_->set(COL_TRAJ_TIME, trajectory_time_);
    csv_logger_->set(COL_REF_TIME, reference_time_);
    csv_logger_->set(COL_COMP_TIME, compute_time_);
    csv_logger_->set(COL_LOOKAHEAD_TIME, T_LOOKAHEAD_);
    csv_logger_->set(COL_X, x_); csv_logger_->set(COL_Y, y_); csv_logger_->set(COL_Z, z_); csv_logger_->set(COL_YAW, yaw_);
    csv_logger_->set(COL_VX, vx_); csv_logger_->set(COL_VY, vy_); csv_logger_->set(COL_VZ, vz_);
    csv_logger_->set(COL_XREF, ref_[0]); csv_logger_->set(COL_YREF, ref_[1]); csv_logger_->set(COL_ZREF, ref_[2]); csv_logger_->set(COL_YAWREF, ref_[3]);
    csv_logger_->set(COL_VXREF, ref_vel_[0]); csv_logger_->set(COL_VYREF, ref_vel_[1]); csv_logger_->set(COL_VZREF, ref_vel_[2]);
    csv_logger_->set(COL_P, p_); csv_logger_->set(COL_Q, q_); csv_logger_->set(COL_R, r_);
    csv_logger_->set(COL_THROTTLE, normalized_input_[0]); csv_logger_->set(COL_P_INPUT, normalized_input_[1]); csv_logger_->set(COL_Q_INPUT, normalized_input_[2]); csv_logger_->set(COL_R_INPUT, normalized_input_[3]);
    csv_logger_->set(COL_CBF_V_THROTTLE, cbf_term_[0]); csv_logger_->set(COL_CBF_V_P, cbf_term_[1]); csv_logger_->set(COL_CBF_V_Q, cbf_term_[2]); csv_logger_->set(COL_CBF_V_R, cbf_term_[3]);
}

void OffboardControlNode::time_controller() {
    Eigen::Vector<double, 9> state0; state0.setZero();
    Eigen::Vector4d input0; input0 << first_thrust_, 0.0, 0.0, 0.0;
    Eigen::Vector4d ref0; ref0 << 0.0, 0.0, -HOVER_HEIGHT_, 0.0;
    Eigen::Matrix<double, 12, 1> xdf0; x_df_.setZero();
    auto result = ctrl::nr_tracker_flat(state0, input0, xdf0, ref0, Eigen::Vector4d::Zero(),
                                       T_LOOKAHEAD_, compute_dt_, platform_->mass(),
                                       Eigen::Matrix3d::Identity(), 0.0, nr_profile_);
    (void)result;
}

}  // namespace nr_diff_flat_px4_cpp
