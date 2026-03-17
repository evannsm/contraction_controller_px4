#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/battery_status.hpp>

#include <Eigen/Dense>
#include <memory>
#include <array>
#include <chrono>
#include <string>

#include "quad_platforms_cpp/platform_config.hpp"
#include "quad_trajectories_cpp/types.hpp"
#include "quad_trajectories_cpp/utils.hpp"
#include "newton_raphson_px4_cpp/px4_utils/flight_phases.hpp"
#include "newton_raphson_px4_cpp/transformations/adjust_yaw.hpp"
#include <ros2_logger_cpp/csv_logger.hpp>

namespace newton_raphson_px4_cpp {

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode(
        quad_platforms_cpp::PlatformType platform_type,
        quad_trajectories_cpp::TrajectoryType trajectory,
        std::optional<int> hover_mode,
        bool double_speed,
        bool short_variant,
        bool spin,
        bool logging_enabled,
        std::string log_file,
        std::optional<double> flight_period_override,
        bool feedforward = false);

    ~OffboardControlNode();

    // Public publishers (accessed by core_funcs templates)
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher;

private:
    // Subscriber callbacks
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void rc_channel_callback(const px4_msgs::msg::RcChannels::SharedPtr msg);
    void battery_status_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

    // Timer callbacks
    void offboard_mode_timer_callback();
    void publish_control_timer_callback();
    void compute_control_timer_callback();
    void data_log_timer_callback();

    // Flight phase logic
    px4_utils::FlightPhase get_phase() const;
    double time_before_next_phase(px4_utils::FlightPhase phase) const;
    bool killswitch_and_flight_phase();
    bool get_offboard_health() const;

    // Control
    void controller();
    void publish_position_setpoint(double x, double y, double z, double yaw);
    void publish_rates_setpoint(double thrust, double roll, double pitch, double yaw);

    // Trajectory
    Eigen::Vector4d generate_ref_position(quad_trajectories_cpp::TrajectoryType type,
                                           std::optional<int> hover_mode_override = std::nullopt);

    // Initialization timing
    void time_controller();
    void time_trajectories();

    // Config
    bool sim_;
    quad_platforms_cpp::PlatformType platform_type_;
    quad_trajectories_cpp::TrajectoryType trajectory_type_;
    std::optional<int> hover_mode_;
    bool double_speed_;
    bool short_variant_;
    bool spin_;
    bool logging_enabled_;
    std::string log_file_;
    bool feedforward_;

    std::unique_ptr<quad_platforms_cpp::PlatformConfig> platform_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr publish_control_timer_;
    rclcpp::TimerBase::SharedPtr compute_control_timer_;
    rclcpp::TimerBase::SharedPtr data_log_timer_;

    // State
    transformations::YawTracker yaw_tracker_;
    bool mocap_initialized_ = false;

    double x_ = 0.0, y_ = 0.0, z_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double p_ = 0.0, q_ = 0.0, r_ = 0.0;   // actual body rates
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    double raw_yaw_ = 0.0;
    Eigen::Vector<double, 9> nr_state_ = Eigen::Vector<double, 9>::Zero();

    bool in_offboard_mode_ = false;
    bool armed_ = false;
    bool in_land_mode_ = false;
    bool offboard_mode_rc_switch_on_;
    int mode_channel_ = 5;

    double current_voltage_ = 16.8;

    // Flight phase timing
    std::chrono::steady_clock::time_point T0_;
    double program_time_ = 0.0;
    double cushion_period_ = 10.0;
    double flight_period_;
    double land_time_;
    px4_utils::FlightPhase flight_phase_;

    int offboard_setpoint_counter_ = 0;

    // Control
    double HOVER_HEIGHT_;
    double LAND_HEIGHT_;
    double T_LOOKAHEAD_ = 1.2;
    double LOOKAHEAD_STATE_DT_ = 0.05;
    double compute_dt_ = 1.0 / 100.0;

    double first_thrust_;
    Eigen::Vector4d last_input_;
    Eigen::Vector4d new_input_;
    std::array<double, 4> normalized_input_;
    Eigen::Vector4d ref_;
    Eigen::Vector4d ref_vel_ = Eigen::Vector4d::Zero();  // reference velocity [vx,vy,vz,yaw_rate]
    Eigen::Vector4d cbf_term_ = Eigen::Vector4d::Zero();
    double compute_time_ = 0.0;

    // Trajectory tracking
    bool trajectory_started_ = false;
    std::chrono::steady_clock::time_point trajectory_T0_;
    double trajectory_time_ = 0.0;
    double reference_time_ = 0.0;

    // Feedforward
    bool ff_valid_ = false;
    Eigen::Vector<double, 10> x_ff_;  // [px,py,pz,vx,vy,vz,f_specific,phi,th,psi]
    Eigen::Vector4d u_ff_;             // [df,dphi,dth,dpsi]
    Eigen::Vector4d u_dev_ = Eigen::Vector4d::Zero();
    bool u_dev_initialized_ = false;

    // Logging
    std::unique_ptr<ros2_logger_cpp::CsvLogger> csv_logger_;
    enum LogCol : size_t {
        // Metadata (constant strings)
        COL_PLATFORM = 0, COL_CONTROLLER, COL_TRAJECTORY,
        COL_TRAJ_DOUBLE, COL_TRAJ_SHORT, COL_TRAJ_SPIN, COL_LOOKAHEAD_TIME,
        // Timing
        COL_TIME, COL_TRAJ_TIME, COL_REF_TIME, COL_COMP_TIME,
        // State
        COL_X, COL_Y, COL_Z, COL_YAW,
        COL_VX, COL_VY, COL_VZ,
        // Reference
        COL_XREF, COL_YREF, COL_ZREF, COL_YAWREF,
        COL_VXREF, COL_VYREF, COL_VZREF,
        // Body rates
        COL_P, COL_Q, COL_R,
        // Control inputs
        COL_THROTTLE, COL_P_INPUT, COL_Q_INPUT, COL_R_INPUT,
        // CBF
        COL_CBF_V_THROTTLE, COL_CBF_V_P, COL_CBF_V_Q, COL_CBF_V_R,
        COL_COUNT
    };
};

}  // namespace newton_raphson_px4_cpp
