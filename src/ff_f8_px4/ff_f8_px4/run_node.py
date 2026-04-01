"""Entry point for the flatness-based contraction-trajectory controller."""

import rclpy
import traceback
import argparse
import os
from pathlib import Path

from ros2_logger import Logger  # type: ignore
from .ros2px4_node import DEFAULT_FBL_GAINS, FBLGains, FeedforwardControl

from quad_platforms import PlatformType
from quad_trajectories import TrajectoryType


SUPPORTED_TRAJECTORIES = [
    TrajectoryType.HOVER_CONTRACTION.value,
    TrajectoryType.FIG8_CONTRACTION.value,
    TrajectoryType.FIG8_HEADING_CONTRACTION.value,
    TrajectoryType.SPIRAL_CONTRACTION.value,
    TrajectoryType.TREFOIL_CONTRACTION.value,
]


def create_parser():
    parser = argparse.ArgumentParser(
        description="Flatness-based controller for contraction trajectories",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        """ + "==" * 60 + """
        Example usage:
        # Auto-generated log filename:
        ros2 run ff_f8_px4 run_node --platform sim --trajectory fig8_contraction --p-feedback --log
        # -> logs to: sim_fbl_fig8_contraction_ramp2p0s_1x.csv

        ros2 run ff_f8_px4 run_node --platform sim --trajectory spiral_contraction --p-feedback --log

        ros2 run ff_f8_px4 run_node --platform sim --trajectory fig8_contraction --ramp-seconds 4.0 --log
        # -> logs to: sim_ff_f8_fig8_contraction_ramp4p0s_1x.csv

        # Custom log filename:
        ros2 run ff_f8_px4 run_node --platform sim --trajectory fig8_contraction --p-feedback --log --log-file my_custom_log
        """ + "==" * 60 + """
        """
    )

    parser.add_argument(
        "--platform",
        type=PlatformType,
        choices=list(PlatformType),
        required=True,
        help="Platform type. Options: " + ", ".join(e.value for e in PlatformType) + ".",
    )

    parser.add_argument(
        "--trajectory",
        type=str,
        default=TrajectoryType.FIG8_CONTRACTION.value,
        choices=SUPPORTED_TRAJECTORIES,
        help="Contraction trajectory to execute.",
    )

    parser.add_argument(
        "--double-speed",
        action="store_true",
        help="Mark log filename with _2x suffix (trajectory period is always 10s)",
    )

    parser.add_argument(
        "--p-feedback",
        action="store_true",
        help="Add light proportional position/attitude feedback on top of feedforward",
    )

    parser.add_argument(
        "--ramp-seconds",
        type=float,
        default=2.0,
        help="Blend from hover commands into feedforward over this many seconds (0 disables)",
    )

    parser.add_argument(
        "--log",
        action="store_true",
        help="Enable data logging.",
    )

    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Custom log file name (without extension). Requires --log.",
    )

    parser.add_argument(
        "--flight-period",
        type=float,
        default=None,
        help="Override default flight duration in seconds (sim: 30s, hw: 60s)",
    )

    parser.add_argument("--kp-xy", type=float, default=None, help="Override lateral position feedback gain.")
    parser.add_argument("--kv-xy", type=float, default=None, help="Override lateral velocity feedback gain.")
    parser.add_argument("--kp-z", type=float, default=None, help="Override vertical position feedback gain.")
    parser.add_argument("--kv-z", type=float, default=None, help="Override vertical velocity feedback gain.")
    parser.add_argument("--kp-att", type=float, default=None, help="Override attitude feedback gain.")
    parser.add_argument("--kp-yaw", type=float, default=None, help="Override yaw feedback gain.")
    parser.add_argument(
        "--kd-body-rates",
        type=float,
        default=None,
        help="Override body-rate damping gain.",
    )
    parser.add_argument(
        "--max-tilt-cmd",
        type=float,
        default=None,
        help="Override maximum commanded tilt [rad].",
    )

    return parser


def ensure_csv(filename: str) -> str:
    filename = filename.strip()
    if filename.lower().endswith(".csv"):
        return filename[:-4] + ".csv"
    return filename + ".csv"


def generate_log_filename(args) -> str:
    """Format: {platform}_{controller}_{trajectory}[_rampXs]_{speed}"""
    controller_tag = "fbl" if args.p_feedback else "ff_f8"
    parts = [
        args.platform.value,
        controller_tag,
        args.trajectory,
    ]
    if args.ramp_seconds > 0.0:
        ramp_tag = str(args.ramp_seconds).replace(".", "p")
        parts.append(f"ramp{ramp_tag}s")
    if any(
        value is not None
        for value in (
            args.kp_xy,
            args.kv_xy,
            args.kp_z,
            args.kv_z,
            args.kp_att,
            args.kp_yaw,
            args.kd_body_rates,
            args.max_tilt_cmd,
        )
    ):
        parts.append("customgains")
    parts.append("2x" if args.double_speed else "1x")
    return "_".join(parts)


def validate_args(args, parser):
    if args.log_file is not None and not args.log:
        parser.error("--log-file requires --log to be enabled")
    if args.ramp_seconds < 0.0:
        parser.error("--ramp-seconds must be >= 0")


def build_fbl_gains(args) -> FBLGains:
    """Apply optional CLI overrides on top of the package defaults."""
    return FBLGains(
        kp_xy=DEFAULT_FBL_GAINS.kp_xy if args.kp_xy is None else args.kp_xy,
        kv_xy=DEFAULT_FBL_GAINS.kv_xy if args.kv_xy is None else args.kv_xy,
        kp_z=DEFAULT_FBL_GAINS.kp_z if args.kp_z is None else args.kp_z,
        kv_z=DEFAULT_FBL_GAINS.kv_z if args.kv_z is None else args.kv_z,
        kp_att=DEFAULT_FBL_GAINS.kp_att if args.kp_att is None else args.kp_att,
        kp_yaw=DEFAULT_FBL_GAINS.kp_yaw if args.kp_yaw is None else args.kp_yaw,
        kd_body_rates=(
            DEFAULT_FBL_GAINS.kd_body_rates
            if args.kd_body_rates is None
            else args.kd_body_rates
        ),
        max_tilt_cmd=(
            DEFAULT_FBL_GAINS.max_tilt_cmd
            if args.max_tilt_cmd is None
            else args.max_tilt_cmd
        ),
    )


def _logger_base_path(file_path: str, pkg_name: str) -> str:
    """Return the base_path that Logger's algorithm needs to produce the correct log directory.

    Logger does: os.path.dirname(base_path) → replaces install/build→src → inserts
    data_analysis/log_files.  When installed by ROS 2, __file__ lives inside
    lib/python3.X/site-packages/, which confuses the algorithm.  We find the
    {ws}/{install_or_src}/{pkg_name} node in the path and return
    {ws}/{install_or_src}/{pkg_name}/{pkg_name} so Logger gets the right root.
    """
    path  = os.path.abspath(file_path)
    parts = path.split(os.sep)
    for i, part in enumerate(parts[:-1]):
        if part in ('install', 'src', 'build') and parts[i + 1] == pkg_name:
            return os.sep.join(parts[:i + 2] + [pkg_name])
    return os.path.dirname(path)  # fallback: works when running directly from src/


def _resolved_log_path(log_file: str | None, base_path: str) -> str | None:
    if not log_file:
        return None

    base_dir = os.path.dirname(base_path)
    parts = ["src" if part in ("build", "install") else part for part in base_dir.split(os.sep)]
    if "src" in parts:
        idx = parts.index("src") + 1
        parts[idx:idx] = ["data_analysis", "log_files"]
    return str(Path(os.sep.join(parts)) / log_file)


def main():
    parser = create_parser()
    args   = parser.parse_args()
    validate_args(args, parser)

    platform       = args.platform
    trajectory_type = TrajectoryType(args.trajectory)
    double_speed   = args.double_speed
    p_feedback     = args.p_feedback
    ramp_seconds   = args.ramp_seconds
    gains          = build_fbl_gains(args)
    logging_enabled = args.log
    flight_period  = args.flight_period
    base_path      = _logger_base_path(__file__, 'ff_f8_px4')

    if logging_enabled:
        log_file_stem = args.log_file if args.log_file is not None else generate_log_filename(args)
        log_file = ensure_csv(log_file_stem)
    else:
        log_file = None
    resolved_log_path = _resolved_log_path(log_file, base_path)

    print("\n" + "=" * 60)
    print("Feedforward Control Configuration")
    print("=" * 60)
    controller_name = "FBL (flatness + light feedback)" if p_feedback else "Flatness Feedforward"
    print(f"Controller:    {controller_name}")
    print(f"Platform:      {platform.value.upper()}")
    print(f"Trajectory:    {args.trajectory.upper()}")
    print(f"P Feedback:    {'Enabled' if p_feedback else 'Disabled'}")
    print(f"Ramp:          {ramp_seconds:.2f} s")
    print(f"Speed:         {'Double (2x)' if double_speed else 'Regular (1x)'}")
    print(f"Flight Period: {flight_period if flight_period is not None else 60.0 if platform == PlatformType.HARDWARE else 30.0} seconds")
    print(f"Data Logging:  {'Enabled' if logging_enabled else 'Disabled'}")
    if logging_enabled:
        print(f"Log File:      {log_file}")
        print(f"Log Path:      {resolved_log_path}")
    print(
        "FBL Gains:     "
        f"kp_xy={gains.kp_xy:.3f}, kv_xy={gains.kv_xy:.3f}, "
        f"kp_z={gains.kp_z:.3f}, kv_z={gains.kv_z:.3f}, "
        f"kp_att={gains.kp_att:.3f}, kp_yaw={gains.kp_yaw:.3f}, "
        f"kd_body_rates={gains.kd_body_rates:.3f}, max_tilt_cmd={gains.max_tilt_cmd:.3f}"
    )
    print("=" * 60 + "\n")

    rclpy.init(args=None)
    node = FeedforwardControl(
        platform_type=platform,
        trajectory_type=trajectory_type,
        double_speed=double_speed,
        p_feedback=p_feedback,
        ramp_seconds=ramp_seconds,
        logging_enabled=logging_enabled,
        flight_period_=flight_period,
        gains=gains,
    )

    logger = None

    def shutdown_logging(*_):
        if logging_enabled and resolved_log_path:
            print(f"\nShutting down, saving log to: {resolved_log_path}")
        else:
            print("\nShutting down.")
        if logger and logging_enabled:
            logger.log(node)
        node.destroy_node()
        rclpy.shutdown()

    try:
        print("\nInitializing Feedforward Control Node")
        if logging_enabled:
            logger = Logger(log_file, base_path)
            print(f"[ff_f8_px4] Logger initialized -> {logger.full_path}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt (Ctrl+C)")
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        if logging_enabled:
            print(f"Saving log data to: {resolved_log_path}")
        shutdown_logging()
        print("\nNode shut down.")


if __name__ == "__main__":
    main()
