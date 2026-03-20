"""Entry point for the fig8_contraction feedforward ROS 2 node."""

import rclpy
import traceback
import argparse
import os
from pathlib import Path

from ros2_logger import Logger  # type: ignore
from .ros2px4_node import FeedforwardControl

from quad_platforms import PlatformType


def create_parser():
    parser = argparse.ArgumentParser(
        description="Feedforward-based controller for the fig8_contraction trajectory",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        """ + "==" * 60 + """
        Example usage:
        # Auto-generated log filename:
        ros2 run ff_f8_px4 run_node --platform sim --log
        # -> logs to: sim_ff_f8_ramp2p0s_1x.csv

        ros2 run ff_f8_px4 run_node --platform sim --double-speed --log
        # -> logs to: sim_ff_f8_ramp2p0s_2x.csv

        ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 4.0 --log
        # -> logs to: sim_ff_f8_pfb_ramp4p0s_1x.csv

        # Custom log filename:
        ros2 run ff_f8_px4 run_node --platform sim --log --log-file my_custom_log
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

    return parser


def ensure_csv(filename: str) -> str:
    filename = filename.strip()
    if filename.lower().endswith(".csv"):
        return filename[:-4] + ".csv"
    return filename + ".csv"


def generate_log_filename(args) -> str:
    """Format: {platform}_ff_f8[_pfb][_rampXs]_{speed}"""
    parts = [
        args.platform.value,
        "ff_f8",
    ]
    if args.p_feedback:
        parts.append("pfb")
    if args.ramp_seconds > 0.0:
        ramp_tag = str(args.ramp_seconds).replace(".", "p")
        parts.append(f"ramp{ramp_tag}s")
    parts.append("2x" if args.double_speed else "1x")
    return "_".join(parts)


def validate_args(args, parser):
    if args.log_file is not None and not args.log:
        parser.error("--log-file requires --log to be enabled")
    if args.ramp_seconds < 0.0:
        parser.error("--ramp-seconds must be >= 0")


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
    double_speed   = args.double_speed
    p_feedback     = args.p_feedback
    ramp_seconds   = args.ramp_seconds
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
    print("Controller:    Pure feedforward (fig8_contraction, flat-output inversion)")
    print(f"Platform:      {platform.value.upper()}")
    print(f"Trajectory:    F8_CONTRACTION")
    print(f"P Feedback:    {'Enabled' if p_feedback else 'Disabled'}")
    print(f"Ramp:          {ramp_seconds:.2f} s")
    print(f"Speed:         {'Double (2x)' if double_speed else 'Regular (1x)'}")
    print(f"Flight Period: {flight_period if flight_period is not None else 60.0 if platform == PlatformType.HARDWARE else 30.0} seconds")
    print(f"Data Logging:  {'Enabled' if logging_enabled else 'Disabled'}")
    if logging_enabled:
        print(f"Log File:      {log_file}")
        print(f"Log Path:      {resolved_log_path}")
    print("=" * 60 + "\n")

    rclpy.init(args=None)
    node = FeedforwardControl(
        platform_type=platform,
        double_speed=double_speed,
        p_feedback=p_feedback,
        ramp_seconds=ramp_seconds,
        logging_enabled=logging_enabled,
        flight_period_=flight_period,
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
