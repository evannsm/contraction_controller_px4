"""Entry point for the contraction controller ROS2 node."""

import argparse
import os
import traceback

import rclpy

from quad_platforms import PlatformType  # type: ignore[import]
from quad_trajectories import TrajectoryType  # type: ignore[import]

from .ros2px4_node import ContractionOffboardControl


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Contraction Neural-Network Offboard Controller for Quadrotor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run contraction_controller_px4 run_node --platform sim --trajectory hover --hover-mode 1
  ros2 run contraction_controller_px4 run_node --platform sim --trajectory circle_horz
  ros2 run contraction_controller_px4 run_node --platform hw  --trajectory helix
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
             "Defaults to new_project/Controller_1/Controller relative to install.",
    )
    parser.add_argument(
        "--flight-period",
        type=float,
        default=None,
        help="Flight duration in seconds (default: 30s sim / 60s hw).",
    )

    return parser


def validate_args(args, parser: argparse.ArgumentParser) -> None:
    if args.trajectory == TrajectoryType.HOVER:
        if args.hover_mode is None:
            parser.error("--hover-mode is required when --trajectory=hover")
    else:
        if args.hover_mode is not None:
            parser.error("--hover-mode is only valid when --trajectory=hover")


def main():
    parser = create_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    print("\n" + "=" * 60)
    print("Contraction Neural-Network Offboard Controller")
    print("=" * 60)
    print(f"Platform:      {args.platform.value.upper()}")
    print(f"Trajectory:    {args.trajectory.value.upper()}")
    print(f"Hover Mode:    {args.hover_mode if args.hover_mode is not None else 'N/A'}")
    print(f"Flight Period: {args.flight_period or ('30s' if args.platform == PlatformType.SIM else '60s')}")
    print(f"Controller:    {args.controller_dir or '(default: new_project/Controller_1/Controller)'}")
    print("=" * 60 + "\n")

    rclpy.init(args=None)
    node = ContractionOffboardControl(
        platform_type=args.platform,
        trajectory=args.trajectory,
        hover_mode=args.hover_mode,
        controller_dir=args.controller_dir,
        flight_period_=args.flight_period,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt.")
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Node shut down.")


if __name__ == "__main__":
    main()
