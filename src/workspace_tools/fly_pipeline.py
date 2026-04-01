#!/usr/bin/env python3
"""Host-side SITL/bridge/controller orchestration for repeatable PX4 experiments."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import IO


IMAGE_NAME = "px4_controllers_jazzy"
CONTAINER_NAME = "px4_controllers"
PX4_MODEL = "gz_x500"
ROS_DOMAIN_ID = "31"

COMMON_TRAJECTORIES = {
    "hover",
    "hover_contraction",
    "yaw_only",
    "circle_horz",
    "circle_vert",
    "fig8_horz",
    "fig8_vert",
    "helix",
    "sawtooth",
    "triangle",
    "fig8_contraction",
    "fig8_heading_contraction",
    "spiral_contraction",
    "trefoil_contraction",
}

CONTRACTION_ONLY_TRAJECTORIES = {
    "hover_contraction",
    "fig8_contraction",
    "fig8_heading_contraction",
    "spiral_contraction",
    "trefoil_contraction",
}


@dataclass(frozen=True)
class ControllerSpec:
    key: str
    package: str
    build_up_to: str
    supports: set[str]
    needs_nmpc_solver: bool = False
    uses_cpp_build: bool = False
    ignores_trajectory: bool = False


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[str]
    log_handle: IO[str]
    log_path: Path


CONTROLLERS: dict[str, ControllerSpec] = {
    "contraction": ControllerSpec(
        key="contraction",
        package="contraction_controller_px4",
        build_up_to="contraction_controller_px4",
        supports=CONTRACTION_ONLY_TRAJECTORIES,
    ),
    "newton_raphson": ControllerSpec(
        key="newton_raphson",
        package="newton_raphson_px4",
        build_up_to="newton_raphson_px4",
        supports=COMMON_TRAJECTORIES,
    ),
    "newton_raphson_enhanced": ControllerSpec(
        key="newton_raphson_enhanced",
        package="newton_raphson_enhanced_px4",
        build_up_to="newton_raphson_enhanced_px4",
        supports=COMMON_TRAJECTORIES,
    ),
    "nr_diff_flat": ControllerSpec(
        key="nr_diff_flat",
        package="nr_diff_flat_px4",
        build_up_to="nr_diff_flat_px4",
        supports=COMMON_TRAJECTORIES,
    ),
    "newton_raphson_cpp": ControllerSpec(
        key="newton_raphson_cpp",
        package="newton_raphson_px4_cpp",
        build_up_to="newton_raphson_px4_cpp",
        supports=COMMON_TRAJECTORIES,
        uses_cpp_build=True,
    ),
    "newton_raphson_enhanced_cpp": ControllerSpec(
        key="newton_raphson_enhanced_cpp",
        package="newton_raphson_enhanced_px4_cpp",
        build_up_to="newton_raphson_enhanced_px4_cpp",
        supports=COMMON_TRAJECTORIES,
        uses_cpp_build=True,
    ),
    "nr_diff_flat_cpp": ControllerSpec(
        key="nr_diff_flat_cpp",
        package="nr_diff_flat_px4_cpp",
        build_up_to="nr_diff_flat_px4_cpp",
        supports=COMMON_TRAJECTORIES,
        uses_cpp_build=True,
    ),
    "nmpc": ControllerSpec(
        key="nmpc",
        package="nmpc_acados_px4",
        build_up_to="nmpc_acados_px4",
        supports=COMMON_TRAJECTORIES,
        needs_nmpc_solver=True,
    ),
    "nmpc_cpp": ControllerSpec(
        key="nmpc_cpp",
        package="nmpc_acados_px4_cpp",
        build_up_to="nmpc_acados_px4_cpp",
        supports=COMMON_TRAJECTORIES,
        needs_nmpc_solver=True,
        uses_cpp_build=True,
    ),
    "fbl": ControllerSpec(
        key="fbl",
        package="ff_f8_px4",
        build_up_to="ff_f8_px4",
        supports=CONTRACTION_ONLY_TRAJECTORIES,
    ),
    "ff_f8": ControllerSpec(
        key="ff_f8",
        package="ff_f8_px4",
        build_up_to="ff_f8_px4",
        supports={"fig8_contraction"},
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one or more PX4 controllers end-to-end.")
    parser.add_argument("--workspace-root", required=True)
    parser.add_argument("--platform", default="sim", choices=["sim"])
    parser.add_argument("--trajectory", default="hover")
    parser.add_argument("--controller", choices=sorted(CONTROLLERS), default="newton_raphson")
    parser.add_argument("--controllers", default="")
    parser.add_argument("--hover-mode", type=int, default=1)
    parser.add_argument("--flight-period", type=float, default=None)
    parser.add_argument("--nmpc-horizon", type=float, default=2.0)
    parser.add_argument("--nmpc-num-steps", type=int, default=50)
    parser.add_argument("--double-speed", action="store_true")
    parser.add_argument("--short", action="store_true")
    parser.add_argument("--spin", action="store_true")
    parser.add_argument("--ff", action="store_true")
    parser.add_argument("--nr-profile", default="baseline", choices=["baseline", "workshop"])
    parser.add_argument("--pyjoules", action="store_true")
    parser.add_argument("--ctrl-type", default="jax", choices=["jax", "numpy"])
    parser.add_argument("--controller-dir", default="")
    parser.add_argument("--no-feedforward", action="store_true")
    parser.add_argument("--p-feedback", action="store_true")
    parser.add_argument("--ramp-seconds", type=float, default=None)
    parser.add_argument("--kp-xy", type=float, default=None)
    parser.add_argument("--kv-xy", type=float, default=None)
    parser.add_argument("--kp-z", type=float, default=None)
    parser.add_argument("--kv-z", type=float, default=None)
    parser.add_argument("--kp-att", type=float, default=None)
    parser.add_argument("--kp-yaw", type=float, default=None)
    parser.add_argument("--kd-body-rates", type=float, default=None)
    parser.add_argument("--max-tilt-cmd", type=float, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--px4-dir", default=str(Path.home() / "PX4-Autopilot"))
    parser.add_argument("--px4-model", default=PX4_MODEL)
    parser.add_argument("--microxrce-port", type=int, default=8888)
    parser.add_argument("--results-dir", default="")
    parser.add_argument("--run-name", default="")
    parser.add_argument("--fly-all", action="store_true")
    parser.add_argument("--skip-analysis", action="store_true")
    parser.add_argument("--image-name", default=IMAGE_NAME)
    parser.add_argument("--container-name", default=CONTAINER_NAME)
    parser.add_argument("--ros-domain-id", default=ROS_DOMAIN_ID)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    workspace_root = Path(args.workspace_root).resolve()
    px4_dir = Path(args.px4_dir).resolve()
    results_dir = determine_results_dir(workspace_root, args)
    logs_dir = results_dir / "host_logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    ensure_host_prerequisites(px4_dir)
    ensure_container(workspace_root, args)

    controller_keys = resolve_controller_keys(args.trajectory, args.controllers, args.fly_all, args.controller)
    log_paths: list[Path] = []

    manifest = {
        "workspace_root": str(workspace_root),
        "results_dir": str(results_dir),
        "trajectory": args.trajectory,
        "headless": args.headless,
        "nr_profile": args.nr_profile,
        "px4_model": args.px4_model,
        "nmpc_horizon": args.nmpc_horizon,
        "nmpc_num_steps": args.nmpc_num_steps,
        "ctrl_type": args.ctrl_type,
        "controllers": controller_keys,
        "fbl_gains": {
            "kp_xy": args.kp_xy,
            "kv_xy": args.kv_xy,
            "kp_z": args.kp_z,
            "kv_z": args.kv_z,
            "kp_att": args.kp_att,
            "kp_yaw": args.kp_yaw,
            "kd_body_rates": args.kd_body_rates,
            "max_tilt_cmd": args.max_tilt_cmd,
        },
        "runs": [],
    }

    for index, controller_key in enumerate(controller_keys):
        if index > 0:
            print("[fly] Restarting SITL and bridge for the next controller...")

        sitl = bridge = None
        try:
            bridge = start_bridge(args, logs_dir)
            sitl = start_sitl(px4_dir, args, logs_dir)
            wait_for_px4_ready(args)

            log_stem = build_log_stem(controller_key, args, index)
            controller_log = logs_dir / f"{controller_key}.log"
            host_log_path = run_controller(
                workspace_root=workspace_root,
                controller_key=controller_key,
                log_stem=log_stem,
                args=args,
                log_path=controller_log,
            )
            log_paths.append(host_log_path)
            manifest["runs"].append(
                {
                    "controller": controller_key,
                    "log_file": str(host_log_path),
                    "controller_log": str(controller_log),
                    "sitl_log": str(sitl.log_path),
                    "bridge_log": str(bridge.log_path),
                }
            )
        finally:
            stop_process(sitl)
            stop_process(bridge)

    if not args.skip_analysis and log_paths:
        analysis_dir = results_dir / "analysis"
        analysis_dir.mkdir(parents=True, exist_ok=True)
        run_analysis(workspace_root, args, analysis_dir, log_paths)
        manifest["analysis_dir"] = str(analysis_dir)

    (results_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"[fly] Completed. Results written to {results_dir}")


def determine_results_dir(workspace_root: Path, args: argparse.Namespace) -> Path:
    run_name = args.run_name.strip()
    if not run_name:
        scope = "all" if args.fly_all else args.controller
        mode = "headless" if args.headless else "gazebo"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = f"{scope}_{args.trajectory}_{mode}_{timestamp}"

    if args.results_dir:
        return Path(args.results_dir).resolve()
    return workspace_root / "src" / "data_analysis" / "results" / run_name


def ensure_host_prerequisites(px4_dir: Path) -> None:
    required_bins = ["docker", "MicroXRCEAgent", "make", "python3"]
    missing = [name for name in required_bins if shutil.which(name) is None]
    if missing:
        raise RuntimeError(f"Missing required host commands: {', '.join(missing)}")
    if not px4_dir.is_dir():
        raise RuntimeError(f"PX4 directory not found: {px4_dir}")


def ensure_container(workspace_root: Path, args: argparse.Namespace) -> None:
    if not docker_image_exists(args.image_name):
        print(f"[fly] Docker image {args.image_name} is missing. Building it now...")
        run_checked(
            ["docker", "build", "-f", "docker/Dockerfile", ".", "-t", args.image_name],
            cwd=workspace_root,
            env=host_env(args),
        )

    if docker_container_running(args.container_name):
        return

    print(f"[fly] Starting controller container {args.container_name}...")
    subprocess.run(["docker", "rm", "-f", args.container_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    run_checked(
        [
            "docker",
            "run",
            "-d",
            "-i",
            "-t",
            "--rm",
            "--net",
            "host",
            "-e",
            f"ROS_DOMAIN_ID={args.ros_domain_id}",
            "-v",
            f"{workspace_root}:/workspace",
            "--name",
            args.container_name,
            args.image_name,
        ],
        env=host_env(args),
    )


def docker_image_exists(image_name: str) -> bool:
    result = subprocess.run(
        ["docker", "image", "inspect", image_name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def docker_container_running(container_name: str) -> bool:
    result = subprocess.run(
        ["docker", "inspect", "-f", "{{.State.Running}}", container_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return result.returncode == 0 and result.stdout.strip() == "true"


def resolve_controller_keys(
    trajectory: str,
    controllers_arg: str,
    fly_all: bool,
    controller: str,
) -> list[str]:
    if controllers_arg.strip():
        keys = [part.strip() for part in controllers_arg.split(",") if part.strip()]
    elif fly_all:
        keys = default_controller_set(trajectory)
    else:
        keys = [controller]

    unknown = [key for key in keys if key not in CONTROLLERS]
    if unknown:
        raise RuntimeError(f"Unknown controller keys: {', '.join(sorted(unknown))}")

    unsupported = [key for key in keys if trajectory not in CONTROLLERS[key].supports]
    if unsupported:
        raise RuntimeError(
            f"Trajectory {trajectory!r} is not supported by: {', '.join(unsupported)}"
        )
    return keys


def default_controller_set(trajectory: str) -> list[str]:
    if trajectory == "fig8_contraction":
        return [
            "contraction",
            "newton_raphson",
            "newton_raphson_enhanced",
            "nr_diff_flat",
            "newton_raphson_cpp",
            "nmpc",
            "nmpc_cpp",
            "fbl",
        ]
    if trajectory in CONTRACTION_ONLY_TRAJECTORIES:
        return [
            "contraction",
            "newton_raphson",
            "newton_raphson_enhanced",
            "nr_diff_flat",
            "newton_raphson_cpp",
            "nmpc",
            "nmpc_cpp",
            "fbl",
        ]
    return [
        "newton_raphson",
        "newton_raphson_enhanced",
        "nr_diff_flat",
        "newton_raphson_cpp",
        "nmpc",
        "nmpc_cpp",
    ]


def start_bridge(args: argparse.Namespace, logs_dir: Path) -> ManagedProcess:
    print("[fly] Starting MicroXRCEAgent on the host...")
    return start_process(
        name="bridge",
        cmd=["MicroXRCEAgent", "udp4", "-p", str(args.microxrce_port)],
        log_path=logs_dir / "microxrce.log",
        env=host_env(args),
    )


def start_sitl(px4_dir: Path, args: argparse.Namespace, logs_dir: Path) -> ManagedProcess:
    print(
        f"[fly] Starting PX4 SITL ({'headless' if args.headless else 'gazebo'}, "
        f"model={args.px4_model}) from {px4_dir}..."
    )
    env = host_env(args)
    if args.headless:
        env["HEADLESS"] = "1"
    return start_process(
        name="sitl",
        cmd=["make", "-C", str(px4_dir), "px4_sitl", args.px4_model],
        log_path=logs_dir / "px4_sitl.log",
        env=env,
    )


def wait_for_px4_ready(args: argparse.Namespace, timeout_seconds: int = 120) -> None:
    print("[fly] Waiting for PX4/ROS 2 bridge readiness...")
    deadline = time.time() + timeout_seconds
    ready_cmd = [
        "docker",
        "exec",
        args.container_name,
        "bash",
        "-lc",
        "ros2 topic echo /fmu/out/vehicle_status_v1 --once",
    ]
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ready_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=host_env(args),
                timeout=5,
            )
        except subprocess.TimeoutExpired:
            time.sleep(2)
            continue
        if result.returncode == 0:
            print("[fly] PX4 is ready.")
            return
        time.sleep(2)
    raise RuntimeError("Timed out waiting for /fmu/out/vehicle_status_v1")


def run_controller(
    *,
    workspace_root: Path,
    controller_key: str,
    log_stem: str,
    args: argparse.Namespace,
    log_path: Path,
) -> Path:
    spec = CONTROLLERS[controller_key]
    host_log_path = (
        workspace_root / "src" / "data_analysis" / "log_files" / spec.package / f"{log_stem}.csv"
    )
    host_log_path.parent.mkdir(parents=True, exist_ok=True)

    shell_steps: list[str] = ["cd /workspace"]
    if spec.needs_nmpc_solver:
        shell_steps.append(
            "python3 src/nmpc_acados_px4/ensure_solver.py "
            f"--platform {shlex.quote(args.platform)} "
            f"--horizon {shlex.quote(str(args.nmpc_horizon))} "
            f"--num-steps {shlex.quote(str(args.nmpc_num_steps))}"
        )
    shell_steps.append(
        "colcon build --symlink-install --packages-up-to "
        f"{spec.build_up_to} "
        "--cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    )
    shell_steps.append("source /workspace/install/setup.bash")
    shell_steps.append(build_ros2_run_command(spec, controller_key, log_stem, args))

    print(f"[fly] Running controller {controller_key}...")
    run_checked(
        ["docker", "exec", args.container_name, "bash", "-lc", " && ".join(shell_steps)],
        env=host_env(args),
        log_path=log_path,
    )
    wait_for_log_file(host_log_path)
    return host_log_path


def build_ros2_run_command(
    spec: ControllerSpec,
    controller_key: str,
    log_stem: str,
    args: argparse.Namespace,
) -> str:
    cmd = ["ros2", "run", spec.package, "run_node"]
    if not spec.ignores_trajectory:
        cmd.extend(["--platform", args.platform, "--trajectory", args.trajectory])

    if controller_key == "contraction":
        if args.trajectory == "hover_contraction":
            cmd.extend(["--hover-mode", str(args.hover_mode)])
        if args.flight_period is not None:
            cmd.extend(["--flight-period", str(args.flight_period)])
        if args.controller_dir:
            cmd.extend(["--controller-dir", args.controller_dir])
        if args.no_feedforward:
            cmd.append("--no-feedforward")
    elif controller_key in {"ff_f8", "fbl"}:
        if args.flight_period is not None:
            cmd.extend(["--flight-period", str(args.flight_period)])
        if args.double_speed:
            cmd.append("--double-speed")
        if controller_key == "fbl" or args.p_feedback:
            cmd.append("--p-feedback")
        if args.ramp_seconds is not None:
            cmd.extend(["--ramp-seconds", str(args.ramp_seconds)])
        if args.kp_xy is not None:
            cmd.extend(["--kp-xy", str(args.kp_xy)])
        if args.kv_xy is not None:
            cmd.extend(["--kv-xy", str(args.kv_xy)])
        if args.kp_z is not None:
            cmd.extend(["--kp-z", str(args.kp_z)])
        if args.kv_z is not None:
            cmd.extend(["--kv-z", str(args.kv_z)])
        if args.kp_att is not None:
            cmd.extend(["--kp-att", str(args.kp_att)])
        if args.kp_yaw is not None:
            cmd.extend(["--kp-yaw", str(args.kp_yaw)])
        if args.kd_body_rates is not None:
            cmd.extend(["--kd-body-rates", str(args.kd_body_rates)])
        if args.max_tilt_cmd is not None:
            cmd.extend(["--max-tilt-cmd", str(args.max_tilt_cmd)])
    else:
        if args.trajectory == "hover":
            cmd.extend(["--hover-mode", str(args.hover_mode)])
        if args.flight_period is not None:
            cmd.extend(["--flight-period", str(args.flight_period)])
        if controller_key == "nr_diff_flat":
            cmd.extend(["--ctrl-type", args.ctrl_type])
            cmd.extend(["--nr-profile", args.nr_profile])
        if controller_key == "nr_diff_flat_cpp":
            cmd.extend(["--nr-profile", args.nr_profile])
        if args.double_speed:
            cmd.append("--double-speed")
        if args.short:
            cmd.append("--short")
        if args.spin:
            cmd.append("--spin")
        if args.ff:
            cmd.append("--ff")
        if controller_key in {
            "newton_raphson",
            "newton_raphson_cpp",
            "newton_raphson_enhanced",
            "newton_raphson_enhanced_cpp",
        }:
            cmd.extend(["--nr-profile", args.nr_profile])
        if args.pyjoules and controller_key in {"newton_raphson", "newton_raphson_enhanced", "nr_diff_flat", "nmpc"}:
            cmd.append("--pyjoules")

    cmd.extend(["--log", "--log-file", log_stem])
    return shlex.join(cmd)


def build_log_stem(controller_key: str, args: argparse.Namespace, run_index: int) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"{timestamp}_{run_index + 1:02d}" if args.fly_all else timestamp
    mode = "headless" if args.headless else "gazebo"
    trajectory = args.trajectory
    ff_suffix = ""
    if args.ff and controller_key in {
        "newton_raphson",
        "newton_raphson_cpp",
        "newton_raphson_enhanced",
        "newton_raphson_enhanced_cpp",
        "nmpc",
        "nmpc_cpp",
        "nr_diff_flat",
        "nr_diff_flat_cpp",
    }:
        ff_suffix = "_ff"
    if controller_key == "contraction" and not args.no_feedforward:
        ff_suffix = "_ff"
    if args.no_feedforward and controller_key == "contraction":
        ff_suffix = "_noff"

    ctrl_suffix = ""
    if controller_key == "nr_diff_flat":
        ctrl_suffix = f"_{args.ctrl_type}"

    fbl_gain_suffix = ""
    if controller_key in {"ff_f8", "fbl"} and any(
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
        fbl_gain_suffix = "_tuned"

    nr_suffix = ""
    if controller_key in {
        "newton_raphson",
        "newton_raphson_cpp",
        "newton_raphson_enhanced",
        "newton_raphson_enhanced_cpp",
        "nr_diff_flat",
        "nr_diff_flat_cpp",
    } and args.nr_profile != "baseline":
        nr_suffix = f"_{args.nr_profile}"
    return (
        f"{args.platform}_{controller_key}_{trajectory}_{mode}"
        f"{ff_suffix}{ctrl_suffix}{nr_suffix}{fbl_gain_suffix}_{suffix}"
    )


def wait_for_log_file(path: Path, timeout_seconds: int = 30) -> None:
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        if path.is_file():
            return
        time.sleep(1)
    raise RuntimeError(f"Expected controller log was not written: {path}")


def run_analysis(
    workspace_root: Path,
    args: argparse.Namespace,
    output_dir: Path,
    log_paths: list[Path],
) -> None:
    print("[fly] Running analysis inside the container...")
    shell_cmd = [
        "cd /workspace",
        "python3 -c 'import pandas' >/dev/null 2>&1 || "
        "(echo \"[fly][ERROR] pandas is missing in the container. Rebuild the image with make build.\" && exit 1)",
        "python3 src/data_analysis/run_analysis.py "
        f"--output-dir {shlex.quote(str(container_path(output_dir, workspace_root)))} "
        + " ".join(
            f"--log {shlex.quote(str(container_path(path, workspace_root)))}" for path in log_paths
        ),
    ]
    run_checked(
        ["docker", "exec", args.container_name, "bash", "-lc", " && ".join(shell_cmd)],
        env=host_env(args),
    )


def container_path(path: Path, workspace_root: Path) -> Path:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(workspace_root)
    except ValueError as exc:
        raise RuntimeError(
            f"Path must stay under the workspace root for container access: {resolved}"
        ) from exc
    return Path("/workspace") / relative


def host_env(args: argparse.Namespace) -> dict[str, str]:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = args.ros_domain_id
    return env


def start_process(
    *,
    name: str,
    cmd: list[str],
    log_path: Path,
    env: dict[str, str],
) -> ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("w")
    process = subprocess.Popen(
        cmd,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        env=env,
        text=True,
        start_new_session=True,
    )
    return ManagedProcess(name=name, process=process, log_handle=log_handle, log_path=log_path)


def stop_process(process: ManagedProcess | None) -> None:
    if process is None:
        return
    try:
        if process.process.poll() is None:
            os.killpg(process.process.pid, signal.SIGTERM)
            process.process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.process.pid, signal.SIGKILL)
        process.process.wait(timeout=10)
    finally:
        process.log_handle.close()


def run_checked(
    cmd: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    log_path: Path | None = None,
) -> None:
    stdout = None
    log_handle = None
    try:
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_handle = log_path.open("w")
            stdout = log_handle
        result = subprocess.run(
            cmd,
            cwd=cwd,
            env=env,
            stdout=stdout,
            stderr=subprocess.STDOUT if stdout is not None else None,
            text=True,
        )
    finally:
        if log_handle is not None:
            log_handle.close()

    if result.returncode != 0:
        if log_path is not None and log_path.exists():
            tail = tail_text(log_path)
            raise RuntimeError(f"Command failed: {shlex.join(cmd)}\nLast log lines:\n{tail}")
        raise RuntimeError(f"Command failed: {shlex.join(cmd)}")


def tail_text(path: Path, max_lines: int = 40) -> str:
    lines = path.read_text(errors="replace").splitlines()
    return "\n".join(lines[-max_lines:])


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[fly][ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
