#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <algorithm>
#include <optional>
#include <filesystem>

#include <ros2_logger_cpp/log_path.hpp>

#include "quad_platforms_cpp/platform_config.hpp"
#include "quad_trajectories_cpp/types.hpp"
#include "nr_diff_flat_px4_cpp/offboard_control_node.hpp"

namespace qp = quad_platforms_cpp;
namespace qt = quad_trajectories_cpp;

struct Args {
    qp::PlatformType platform;
    qt::TrajectoryType trajectory;
    std::optional<int> hover_mode;
    bool log = false;
    std::string log_file;
    bool double_speed = false;
    bool short_variant = false;
    bool spin = false;
    std::optional<double> flight_period;
    bool feedforward = false;
    std::string nr_profile = "baseline";
};

static std::string generate_log_filename(const Args& args) {
    std::string name = qp::platform_type_to_string(args.platform);
    name += "_nr_diff_flat_";
    name += qt::trajectory_type_to_string(args.trajectory);
    if (args.feedforward) name += "_ff";
    if (args.nr_profile != "baseline") name += "_" + args.nr_profile;
    name += args.double_speed ? "_2x" : "_1x";
    if (args.short_variant) name += "_short";
    if (args.spin) name += "_spin";
    name += "_cpp";
    return name;
}

static void print_usage() {
    std::cerr <<
        "\nUsage: ros2 run nr_diff_flat_px4_cpp run_node [OPTIONS]\n"
        "\n"
        "Newton-Raphson Diff-Flat Offboard Control for Quadrotor\n"
        "\n"
        "Required:\n"
        "  --platform {sim,hw}          Platform type\n"
        "  --trajectory {hover,yaw_only,circle_horz,circle_vert,\n"
        "                fig8_horz,fig8_vert,triangle,sawtooth,\n"
        "                helix,fig8_contraction}\n"
        "                               Trajectory to execute\n"
        "\n"
        "Optional:\n"
        "  --hover-mode 1-8             Required when --trajectory=hover (hw: 1-4 only)\n"
        "  --log                        Enable data logging (auto-generates filename)\n"
        "  --log-file NAME              Custom log filename (requires --log)\n"
        "  --double-speed               Use 2x speed for trajectories\n"
        "  --short                      Use short variant for fig8_vert trajectory\n"
        "  --spin                       Enable spin for circle_horz and helix\n"
        "  --flight-period SECONDS      Override default flight duration (sim: 30s, hw: 60s)\n"
        "  --ff                         Enable feedforward (only valid with --trajectory=fig8_contraction)\n"
        "  --nr-profile {baseline,workshop}\n"
        "                               Newton-Raphson profile to run\n"
        "\n"
        "Examples:\n"
        "  # Auto-generated log filename:\n"
        "  ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory helix --double-speed --spin --log\n"
        "  # -> logs to: src/data_analysis/log_files/nr_diff_flat_px4_cpp/sim_nr_diff_flat_helix_2x_spin.csv\n"
        "\n"
        "  # Custom log filename:\n"
        "  ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory helix --log --log-file my_log\n"
        "  # -> logs to: src/data_analysis/log_files/nr_diff_flat_px4_cpp/my_log.csv\n"
        "\n"
        "  # Feedforward (fig8_contraction only):\n"
        "  ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff --log\n"
        "  # -> logs to: src/data_analysis/log_files/nr_diff_flat_px4_cpp/sim_nr_diff_flat_fig8_contraction_ff_1x.csv\n"
        "\n";
}

static Args parse_args(int argc, char* argv[]) {
    Args args;
    bool has_platform = false, has_trajectory = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--platform" && i + 1 < argc) {
            args.platform = qp::platform_type_from_string(argv[++i]);
            has_platform = true;
        } else if (arg == "--trajectory" && i + 1 < argc) {
            args.trajectory = qt::trajectory_type_from_string(argv[++i]);
            has_trajectory = true;
        } else if (arg == "--hover-mode" && i + 1 < argc) {
            args.hover_mode = std::stoi(argv[++i]);
        } else if (arg == "--log") {
            args.log = true;
        } else if (arg == "--log-file" && i + 1 < argc) {
            args.log_file = argv[++i];
        } else if (arg == "--double-speed") {
            args.double_speed = true;
        } else if (arg == "--short") {
            args.short_variant = true;
        } else if (arg == "--spin") {
            args.spin = true;
        } else if (arg == "--flight-period" && i + 1 < argc) {
            args.flight_period = std::stod(argv[++i]);
        } else if (arg == "--ff") {
            args.feedforward = true;
        } else if (arg == "--nr-profile" && i + 1 < argc) {
            args.nr_profile = argv[++i];
        } else {
            print_usage();
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (!has_platform || !has_trajectory) {
        print_usage();
        throw std::runtime_error("--platform and --trajectory are required");
    }

    // Validation
    if (args.trajectory == qt::TrajectoryType::HOVER) {
        if (!args.hover_mode.has_value()) {
            throw std::runtime_error("--hover-mode is required when --trajectory=hover");
        }
        if (args.platform == qp::PlatformType::HARDWARE && args.hover_mode.value() > 4) {
            throw std::runtime_error("--hover-mode must be 1-4 for --platform=hw");
        }
    } else {
        if (args.hover_mode.has_value()) {
            throw std::runtime_error("--hover-mode is only valid when --trajectory=hover");
        }
    }

    if (!args.log_file.empty() && !args.log) {
        throw std::runtime_error("--log-file requires --log to be enabled");
    }

    if (args.feedforward && args.trajectory != qt::TrajectoryType::FIG8_CONTRACTION) {
        throw std::runtime_error("--ff is only valid with --trajectory=fig8_contraction");
    }
    if (args.nr_profile != "baseline" && args.nr_profile != "workshop") {
        throw std::runtime_error("--nr-profile must be baseline or workshop");
    }

    return args;
}

int main(int argc, char* argv[]) {
    Args args;
    try {
        args = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Determine log file path using ros2_logger_cpp path resolution
    std::string log_file;
    if (args.log) {
        std::string fname = args.log_file.empty() ? generate_log_filename(args) : args.log_file;
        std::filesystem::path exe_path = std::filesystem::canonical(argv[0]);
        auto paths = ros2_logger_cpp::resolve_log_path(fname, exe_path);
        log_file = paths.log_file.string();
        ros2_logger_cpp::copy_analysis_helpers(paths.data_analysis);
    }

    // Print configuration
    std::string sep(60, '=');
    std::cout << "\n" << sep << std::endl;
    std::cout << "Newton-Raphson Diff-Flat Offboard Control Configuration (C++)" << std::endl;
    std::cout << sep << std::endl;
    std::cout << "Platform:      " << qp::platform_type_to_string(args.platform) << std::endl;
    std::cout << "Controller:    STANDARD" << std::endl;
    std::cout << "Trajectory:    " << qt::trajectory_type_to_string(args.trajectory) << std::endl;
    std::cout << "Hover Mode:    " << (args.hover_mode ? std::to_string(*args.hover_mode) : "N/A") << std::endl;
    std::cout << "Speed:         " << (args.double_speed ? "Double (2x)" : "Regular (1x)") << std::endl;
    std::cout << "Short:         " << (args.short_variant ? "Enabled" : "Disabled") << std::endl;
    double fp = args.flight_period.value_or(
        args.platform == qp::PlatformType::HARDWARE ? 60.0 : 30.0);
    std::cout << "Flight Period: " << fp << " seconds" << std::endl;
    std::cout << "Spin:          " << (args.spin ? "Enabled" : "Disabled") << std::endl;
    std::cout << "Feedforward:   " << (args.feedforward ? "Enabled (fig8_contraction)" : "Disabled") << std::endl;
    std::cout << "NR Profile:    " << args.nr_profile << std::endl;
    std::cout << "Data Logging:  " << (args.log ? "Enabled" : "Disabled") << std::endl;
    if (args.log) {
        std::cout << "Log File:      " << log_file << std::endl;
    }
    std::cout << sep << "\n" << std::endl;

    rclcpp::init(argc, argv);

    auto node = std::make_shared<nr_diff_flat_px4_cpp::OffboardControlNode>(
        args.platform, args.trajectory, args.hover_mode,
        args.double_speed, args.short_variant, args.spin,
        args.log, log_file, args.flight_period, args.feedforward, args.nr_profile);

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // Log is saved automatically in OffboardControlNode's destructor when node goes out of scope.

    rclcpp::shutdown();
    std::cout << "\nNode shut down." << std::endl;
    return 0;
}
