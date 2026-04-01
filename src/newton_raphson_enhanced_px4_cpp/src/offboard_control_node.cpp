#include "newton_raphson_enhanced_px4_cpp/offboard_control_node.hpp"
#include "newton_raphson_enhanced_px4_cpp/px4_utils/core_funcs.hpp"
#include "newton_raphson_enhanced_px4_cpp/controller/newton_raphson.hpp"
#include "newton_raphson_enhanced_px4_cpp/controller/nr_utils.hpp"
#include "quad_trajectories_cpp/registry.hpp"
#include "quad_trajectories_cpp/utils.hpp"

#include <cmath>
#include <thread>
#include <iostream>

namespace newton_raphson_enhanced_px4_cpp {

using namespace std::chrono_literals;
namespace qp = quad_platforms_cpp;
namespace qt = quad_trajectories_cpp;
namespace ctrl = newton_raphson_enhanced_px4_cpp::controller;

static const std::string BANNER = "\n============================================================\n";

// --- Feedforward helper ---------------------------------------------------
// Analytically computes flat-output feedforward (x_ff, u_ff) for fig8_contraction.
//
// x_ff = [px, py, pz, vx, vy, vz, f_specific, phi, th, psi]
//   f_specific = specific thrust [m/s²] = thrust / mass
//   phi = roll, th = pitch, psi = yaw
//
// u_ff = [df, dphi, dth, dpsi]  (time-derivatives of f_specific, roll, pitch, yaw)
struct FfState {
    Eigen::Vector<double, 10> x;
    Eigen::Vector4d u;
};

static FfState flat_to_x_u_f8(double t, bool sim) {
    constexpr double g = 9.8;  // Match Gazebo world
    const double height = sim ? 3.0 : 0.85;
    const double R      = sim ? 2.0 : 0.5;
    const double Tp     = 10.0;

    const double w1 = 2.0 * M_PI / Tp;
    const double w2 = 4.0 * M_PI / Tp;

    // Position
    double px = R * std::sin(w1 * t);
    double py = (R / 2.0) * std::sin(w2 * t);
    double pz = -height;
    double psi = 0.0;

    // Velocity (1st derivative)
    double vx   = R * w1 * std::cos(w1 * t);
    double vy   = (R / 2.0) * w2 * std::cos(w2 * t);
    double vz   = 0.0;
    double dpsi = 0.0;

    // Acceleration (2nd derivative)
    double ax = -R * w1 * w1 * std::sin(w1 * t);
    double ay = -(R / 2.0) * w2 * w2 * std::sin(w2 * t);
    double az = 0.0;

    // Jerk (3rd derivative)
    double jx = -R * w1 * w1 * w1 * std::cos(w1 * t);
    double jy = -(R / 2.0) * w2 * w2 * w2 * std::cos(w2 * t);
    double jz = 0.0;

    // Flat-output inversion
    double f  = std::sqrt(ax*ax + ay*ay + (az - g)*(az - g));
    double th  = std::asin(-ax / f);
    double phi = std::atan2(ay, g - az);

    // Time derivatives of f, th, phi
    double df  = (ax*jx + ay*jy + (az - g)*jz) / f;
    // th = arcsin(-ax/f)  =>  dth = 1/cos(th) * (-jx*f + ax*df) / f²
    double dth = (1.0 / std::cos(th)) * (-jx + ax * df / f) / f;
    // phi = atan2(ay, g-az)  =>  dphi = ((g-az)*jy + ay*(-jz)) / ((g-az)^2 + ay^2)
    double dphi = ((g - az)*jy + ay*jz) / ((g - az)*(g - az) + ay*ay);

    FfState ff;
    ff.x << px, py, pz, vx, vy, vz, f, phi, th, psi;
    ff.u << df, dphi, dth, dpsi;
    return ff;
}
// --------------------------------------------------------------------------

// Quaternion to Euler conversion (scalar-first: [w,x,y,z])
static void quat_to_euler(double w, double x, double y, double z,
                            double& roll, double& pitch, double& yaw) {
    // Roll (x-axis)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis)
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

    // Platform
    platform_ = qp::create_platform(platform_type_);

    std::cout << "\n[Trajectory] Main trajectory type: "
              << qt::trajectory_type_name(trajectory_type_) << std::endl;
    T_LOOKAHEAD_ = nr_profile_.lookahead_horizon_s;
    std::cout << "[Enhanced NR Profile] " << nr_profile_.name
              << ": lookahead=" << T_LOOKAHEAD_ << "s, "
              << "iterations=" << nr_profile_.num_iterations << ", "
              << "predictor=" << (nr_profile_.use_foh ? "FOH" : "ZOH")
              << std::endl;

    // QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()
        .transient_local();

    // Publishers
    offboard_control_mode_publisher = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    trajectory_setpoint_publisher = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos);
    rates_setpoint_publisher = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", qos);
    vehicle_command_publisher = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos);

    // Subscribers
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

    // Flight timing
    flight_period_ = flight_period_override.value_or(sim_ ? 30.0 : 60.0);
    land_time_ = flight_period_ + 2.0 * cushion_period_;
    T0_ = std::chrono::steady_clock::now();
    flight_phase_ = get_phase();
    std::cout << "Flight time: " << land_time_ << "s" << std::endl;

    // Timers
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

    // Control initialization
    HOVER_HEIGHT_ = sim_ ? 3.0 : 0.7;
    LAND_HEIGHT_  = sim_ ? 0.6 : 0.45;

    first_thrust_ = platform_->mass() * ctrl::GRAVITY;
    last_input_ << first_thrust_, 0.0, 0.0, 0.0;
    new_input_ = last_input_;
    normalized_input_ = {platform_->get_throttle_from_force(first_thrust_), 0.0, 0.0, 0.0};
    ref_ = Eigen::Vector4d::Zero();

    // Test controller timing
    time_controller();
    time_trajectories();

    std::cout << "[Offboard Control Node] Node initialized successfully!\n" << std::endl;
    std::this_thread::sleep_for(3s);

    T0_ = std::chrono::steady_clock::now(); // Reset after init

    // Set up logging
    if (logging_enabled_) {
        csv_logger_ = std::make_unique<ros2_logger_cpp::CsvLogger>();
        const char* col_names[] = {
            // Metadata
            "platform", "controller", "trajectory",
            "traj_double", "traj_short", "traj_spin", "lookahead_time",
            // Timing
            "time", "traj_time", "ref_time", "comp_time",
            // State
            "x", "y", "z", "yaw",
            "vx", "vy", "vz",
            // Reference
            "x_ref", "y_ref", "z_ref", "yaw_ref",
            "vx_ref", "vy_ref", "vz_ref",
            // Body rates
            "p", "q", "r",
            // Control inputs
            "throttle_input", "p_input", "q_input", "r_input",
            // CBF
            "cbf_v_throttle", "cbf_v_p", "cbf_v_q", "cbf_v_r"
        };
        for (const auto& name : col_names) {
            csv_logger_->add_header(name);
        }

        // Constant metadata strings
        csv_logger_->set_const_string(COL_PLATFORM,    sim_ ? "SIM" : "HW");
        csv_logger_->set_const_string(COL_CONTROLLER,  "nr_enhanced");
        csv_logger_->set_const_string(COL_TRAJECTORY,  qt::trajectory_type_name(trajectory_type_));
        csv_logger_->set_const_string(COL_TRAJ_DOUBLE, double_speed_ ? "DblSpd" : "NormSpd");
        csv_logger_->set_const_string(COL_TRAJ_SHORT,  short_variant_ ? "Short" : "Not Short");
        csv_logger_->set_const_string(COL_TRAJ_SPIN,   spin_ ? "Spin" : "NoSpin");

        std::cout << "Data logging is ON" << std::endl;
    }
}

OffboardControlNode::~OffboardControlNode() {
    if (csv_logger_ && !log_file_.empty()) {
        std::cout << "Saving log data to " << log_file_ << "..." << std::endl;
        if (csv_logger_->save(log_file_)) {
            std::cout << "Log saved (" << csv_logger_->num_rows() << " rows)." << std::endl;
        } else {
            std::cerr << "Failed to save log to " << log_file_ << std::endl;
        }
    }
}

// Subscriber callbacks
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

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
        "State output: [%.3f, %.3f, %.3f, %.3f]", x_, y_, z_, yaw_);
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

// Flight phase
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
        case px4_utils::FlightPhase::HOVER:
            return cushion_period_ - program_time_;
        case px4_utils::FlightPhase::CUSTOM:
            return (cushion_period_ + flight_period_) - program_time_;
        case px4_utils::FlightPhase::RETURN:
            return land_time_ - program_time_;
        case px4_utils::FlightPhase::LAND:
            return 0.0;
    }
    return 0.0;
}

bool OffboardControlNode::killswitch_and_flight_phase() {
    if (!offboard_mode_rc_switch_on_) {
        RCLCPP_WARN(get_logger(),
            "Offboard Callback: RC Flight Mode Channel %d Switch Not Set to Offboard", mode_channel_);
        offboard_setpoint_counter_ = 0;
        return false;
    }

    auto now = std::chrono::steady_clock::now();
    program_time_ = std::chrono::duration<double>(now - T0_).count();
    flight_phase_ = get_phase();

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[%.2fs] In %s phase for the next %.2fs",
        program_time_, px4_utils::flight_phase_name(flight_phase_).c_str(),
        time_before_next_phase(flight_phase_));

    return true;
}

bool OffboardControlNode::get_offboard_health() const {
    bool healthy = true;
    if (!in_offboard_mode_) {
        RCLCPP_WARN(get_logger(), "Vehicle is NOT in OFFBOARD mode.");
        healthy = false;
    }
    if (!armed_) {
        RCLCPP_WARN(get_logger(), "Vehicle is NOT ARMED.");
        healthy = false;
    }
    if (!mocap_initialized_) {
        RCLCPP_WARN(get_logger(), "Odometry is NOT received.");
        healthy = false;
    }
    return healthy;
}

// Timer callbacks
void OffboardControlNode::offboard_mode_timer_callback() {
    if (!killswitch_and_flight_phase()) return;

    if (offboard_setpoint_counter_ == 10) {
        std::cout << "Trying to arm" << std::endl;
        px4_utils::engage_offboard_mode(*this);
        px4_utils::arm(*this);
    }
    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }

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
        case px4_utils::FlightPhase::HOVER:
            publish_position_setpoint(0.0, 0.0, -HOVER_HEIGHT_, 0.0);
            break;
        case px4_utils::FlightPhase::CUSTOM:
            publish_rates_setpoint(normalized_input_[0], normalized_input_[1],
                                   normalized_input_[2], normalized_input_[3]);
            break;
        case px4_utils::FlightPhase::RETURN:
            publish_position_setpoint(0.0, 0.0, -HOVER_HEIGHT_, 0.0);
            break;
        case px4_utils::FlightPhase::LAND:
            publish_position_setpoint(0.0, 0.0, -LAND_HEIGHT_, 0.0);
            if (std::abs(z_) < 0.64) {
                px4_utils::land(*this);
            }
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
        ref_rate_ = qt::get_velocity(trajectory_type_, reference_time_, ctx);
        ref_vel_ = qt::get_velocity(trajectory_type_, reference_time_, ctx);
    }

    // Feedforward: compute x_ff, u_ff from flat-output inversion
    if (feedforward_) {
        qt::TrajContext ctx;
        ctx.sim = sim_;
        ctx.hover_mode = hover_mode_;
        ctx.spin = spin_;
        ctx.double_speed = (trajectory_type_ == qt::TrajectoryType::FIG8_CONTRACTION) ? false : double_speed_;
        ctx.short_variant = short_variant_;
        auto ff = qt::flat_to_x_u(reference_time_, trajectory_type_, ctx);
        x_ff_ = ff.x_ff;
        u_ff_ = ff.u_ff;
        ff_valid_ = true;
    } else {
        ff_valid_ = false;
        u_dev_initialized_ = false;
    }

    auto t0 = std::chrono::steady_clock::now();
    controller();
    auto t1 = std::chrono::steady_clock::now();
    compute_time_ = std::chrono::duration<double>(t1 - t0).count();

    last_input_ = new_input_;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 300,
        "Control computation time: %.4fs, Good for %.2f Hz control loop",
        compute_time_, 1.0 / compute_time_);

    normalized_input_ = {platform_->get_throttle_from_force(new_input_[0]),
                         new_input_[1], new_input_[2], new_input_[3]};
}

void OffboardControlNode::controller() {
    Eigen::Vector4d effective_last_input = last_input_;
    Eigen::Vector4d u_ff_vec = Eigen::Vector4d::Zero();

    if (ff_valid_) {
        double sr = std::sin(roll_), cr = std::cos(roll_);
        double sp = std::sin(pitch_), cp = std::cos(pitch_);
        double tp = sp / cp;

        Eigen::Matrix3d Tmat;
        Tmat << 1.0, sr * tp,       cr * tp,
                0.0, cr,           -sr,
                0.0, sr / cp,       cr / cp;

        Eigen::Vector3d euler_rates_ff = u_ff_.segment<3>(1);
        Eigen::Vector3d body_rates_ff  = Tmat.colPivHouseholderQr().solve(euler_rates_ff);

        double thrust_ff = platform_->mass() * x_ff_[6];
        u_ff_vec << thrust_ff, body_rates_ff[0], body_rates_ff[1], body_rates_ff[2];

        if (!u_dev_initialized_) {
            u_dev_ = last_input_ - u_ff_vec;
            u_dev_initialized_ = true;
        }
        effective_last_input = u_ff_vec + u_dev_;
    } else {
        u_dev_initialized_ = false;
    }

    auto result = ctrl::newton_raphson_enhanced(
        nr_state_, effective_last_input, ref_, ref_rate_, nr_error_integral_,
        T_LOOKAHEAD_, LOOKAHEAD_STATE_DT_,
        compute_dt_, platform_->mass(), nr_profile_);
    new_input_ = result.u;
    cbf_term_ = result.v;

    if (ff_valid_) {
        u_dev_ = new_input_ - u_ff_vec;
    }
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
    ctx.hover_mode = hover_mode_override.has_value() ? hover_mode_override : hover_mode_;
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

    csv_logger_->set(COL_TIME,        program_time_);
    csv_logger_->set(COL_TRAJ_TIME,   trajectory_time_);
    csv_logger_->set(COL_REF_TIME,    reference_time_);
    csv_logger_->set(COL_COMP_TIME,   compute_time_);
    csv_logger_->set(COL_LOOKAHEAD_TIME, T_LOOKAHEAD_);

    csv_logger_->set(COL_X,   x_);
    csv_logger_->set(COL_Y,   y_);
    csv_logger_->set(COL_Z,   z_);
    csv_logger_->set(COL_YAW, yaw_);
    csv_logger_->set(COL_VX,  vx_);
    csv_logger_->set(COL_VY,  vy_);
    csv_logger_->set(COL_VZ,  vz_);

    csv_logger_->set(COL_XREF,   ref_[0]);
    csv_logger_->set(COL_YREF,   ref_[1]);
    csv_logger_->set(COL_ZREF,   ref_[2]);
    csv_logger_->set(COL_YAWREF, ref_[3]);
    csv_logger_->set(COL_VXREF,  ref_vel_[0]);
    csv_logger_->set(COL_VYREF,  ref_vel_[1]);
    csv_logger_->set(COL_VZREF,  ref_vel_[2]);

    csv_logger_->set(COL_P, p_);
    csv_logger_->set(COL_Q, q_);
    csv_logger_->set(COL_R, r_);

    csv_logger_->set(COL_THROTTLE,  normalized_input_[0]);
    csv_logger_->set(COL_P_INPUT,   normalized_input_[1]);
    csv_logger_->set(COL_Q_INPUT,   normalized_input_[2]);
    csv_logger_->set(COL_R_INPUT,   normalized_input_[3]);

    csv_logger_->set(COL_CBF_V_THROTTLE, cbf_term_[0]);
    csv_logger_->set(COL_CBF_V_P,        cbf_term_[1]);
    csv_logger_->set(COL_CBF_V_Q,        cbf_term_[2]);
    csv_logger_->set(COL_CBF_V_R,        cbf_term_[3]);
}

void OffboardControlNode::time_controller() {
    Eigen::Vector<double, 9> state0;
    state0 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
    Eigen::Vector4d input0;
    input0 << first_thrust_, 0.1, 0.2, 0.3;
    Eigen::Vector4d ref0;
    ref0 << 2.0, 2.0, -6.0, 0.0;
    Eigen::Vector4d rate0;
    rate0 << 0.1, 0.1, 0.0, 0.0;

    std::cout << "\n[Timing] Enhanced controller computation..." << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    auto result1 = ctrl::newton_raphson_enhanced(
        state0,
        input0,
        ref0,
        rate0,
        Eigen::Vector4d::Zero(),
        T_LOOKAHEAD_,
        LOOKAHEAD_STATE_DT_,
        compute_dt_,
        platform_->mass(),
        nr_profile_);
    auto t1 = std::chrono::steady_clock::now();
    double time1 = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "  Result: u=[" << result1.u.transpose() << "]" << std::endl;
    std::cout << "  Time: " << time1 << "s" << std::endl;
}

void OffboardControlNode::time_trajectories() {
    auto pos = generate_ref_position(qt::TrajectoryType::HOVER, hover_mode_);
    auto pos2 = generate_ref_position(trajectory_type_);
    (void)pos; (void)pos2;
}

}  // namespace newton_raphson_enhanced_px4_cpp
