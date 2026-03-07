"""Entry point for the contraction controller ROS2 node."""

import argparse
import os
import traceback

import rclpy

from quad_platforms import PlatformType  # type: ignore[import]
from .trajectories import TrajectoryType
from .ros2px4_node import ContractionOffboardControl

from Logger import Logger  # type: ignore[import]
from Logger.shutdown_helpers import install_shutdown_logging  # type: ignore[import]


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Contraction Neural-Network Offboard Controller for Quadrotor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run contraction_controller_px4 run_node --platform sim --trajectory hover --hover-mode 1
  ros2 run contraction_controller_px4 run_node --platform sim --trajectory figure_eight --log
  ros2 run contraction_controller_px4 run_node --platform sim --trajectory trefoil --log --log-file my_run
  ros2 run contraction_controller_px4 run_node --platform hw  --trajectory spiral
        """,
    )

    parser.add_argument(
        "--platform",
        type=PlatformType,
        choices=list(PlatformType),
        required=True,
        help="Platform: " + ", ".join(e.value for e in PlatformType),
    )
    parser.add_argument(
        "--trajectory",
        type=TrajectoryType,
        choices=list(TrajectoryType),
        required=True,
        help="Trajectory: " + ", ".join(e.value for e in TrajectoryType),
    )
    parser.add_argument(
        "--hover-mode",
        type=int,
        choices=range(1, 9),
        help="Hover sub-mode (required when --trajectory=hover).",
    )
    parser.add_argument(
        "--controller-dir",
        type=str,
        default=None,
        help="Path to the Controller directory containing arch.txt and model.eqx. "
             "Defaults to src/controller_params relative to workspace root.",
    )
    parser.add_argument(
        "--flight-period",
        type=float,
        default=None,
        help="Flight duration in seconds (default: 30s sim / 60s hw).",
    )
    parser.add_argument(
        "--log",
        action="store_true",
        help="Enable CSV data logging. Written to src/data_analysis/log_files/ on shutdown.",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Log filename stem (without .csv). Auto-generated from config if omitted.",
    )

    return parser


def validate_args(args, parser: argparse.ArgumentParser) -> None:
    if args.trajectory == TrajectoryType.HOVER:
        if args.hover_mode is None:
            parser.error("--hover-mode is required when --trajectory=hover")
    else:
        if args.hover_mode is not None:
            parser.error("--hover-mode is only valid when --trajectory=hover")
    if args.log_file is not None and not args.log:
        parser.error("--log-file requires --log")


def _auto_log_filename(args) -> str:
    parts = [args.platform.value, "contraction", args.trajectory.value]
    if args.trajectory == TrajectoryType.HOVER and args.hover_mode is not None:
        parts.append(f"mode{args.hover_mode}")
    return "_".join(parts) + ".csv"


def main():
    parser = create_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    logging_enabled = args.log
    if logging_enabled:
        log_file = args.log_file if args.log_file else _auto_log_filename(args)
        if not log_file.lower().endswith(".csv"):
            log_file += ".csv"
    else:
        log_file = None

    print("\n" + "=" * 60)
    print("Contraction Neural-Network Offboard Controller")
    print("=" * 60)
    print(f"Platform:      {args.platform.value.upper()}")
    print(f"Trajectory:    {args.trajectory.value.upper()}")
    print(f"Hover Mode:    {args.hover_mode if args.hover_mode is not None else 'N/A'}")
    print(f"Flight Period: {args.flight_period or ('30s' if args.platform == PlatformType.SIM else '60s')}")
    print(f"Controller:    {args.controller_dir or '(default: src/controller_params)'}")
    print(f"Data Logging:  {'Enabled → ' + log_file if logging_enabled else 'Disabled'}")
    print("=" * 60 + "\n")

    base_path = os.path.dirname(os.path.abspath(__file__))

    rclpy.init(args=None)
    node = ContractionOffboardControl(
        platform_type=args.platform,
        trajectory=args.trajectory,
        hover_mode=args.hover_mode,
        controller_dir=args.controller_dir,
        flight_period_=args.flight_period,
        logging_enabled=logging_enabled,
    )

    logger = Logger(log_file, base_path) if logging_enabled else None

    if logger is not None:
        install_shutdown_logging(logger, node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt.")
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        if logger is not None and logging_enabled:
            print("Saving log data...")
            logger.log(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Node shut down.")


if __name__ == "__main__":
    main()
