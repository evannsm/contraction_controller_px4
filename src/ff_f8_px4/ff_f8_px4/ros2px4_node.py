"""Flatness-based ROS 2 node for the contraction trajectory family.

Publishes [throttle, p, q, r] commands derived from differential-flatness
inversion of a shared `quad_trajectories` contraction trajectory. An optional
light feedback layer can be enabled on top of the feedforward command.
"""

import time
import math as m
import numpy as np
import jax
import jax.numpy as jnp
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleRatesSetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
    RcChannels,
)

from quad_platforms import PlatformType, PlatformConfig, PLATFORM_REGISTRY
from quad_trajectories import TrajContext, TrajectoryType, TRAJ_REGISTRY, flat_to_x_u

from ros2_logger import LogType  # type: ignore

GRAVITY: float = 9.8  # Match Gazebo world

# Flight phases (inline — no shared utils dependency)
from enum import Enum
class FlightPhase(Enum):
    HOVER  = "HOVER"
    CUSTOM = "CUSTOM"
    RETURN = "RETURN"
    LAND   = "LAND"


SUPPORTED_TRAJECTORY_TYPES = {
    TrajectoryType.HOVER_CONTRACTION,
    TrajectoryType.FIG8_CONTRACTION,
    TrajectoryType.FIG8_HEADING_CONTRACTION,
    TrajectoryType.SPIRAL_CONTRACTION,
    TrajectoryType.TREFOIL_CONTRACTION,
}


def _arm(node):
    _pub_cmd(node, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    node.get_logger().info("Arm command sent")

def _disarm(node):
    _pub_cmd(node, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
    node.get_logger().info("Disarm command sent")

def _engage_offboard(node):
    _pub_cmd(node, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    node.get_logger().info("Switching to offboard mode")

def _land(node):
    _pub_cmd(node, VehicleCommand.VEHICLE_CMD_NAV_LAND)
    node.get_logger().info("Switching to land mode")

def _pub_cmd(node, command, **params):
    msg = VehicleCommand()
    msg.command   = command
    msg.param1    = params.get("param1", 0.0)
    msg.param2    = params.get("param2", 0.0)
    msg.target_system    = 1
    msg.target_component = 1
    msg.source_system    = 1
    msg.source_component = 1
    msg.from_external    = True
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    node.vehicle_command_publisher.publish(msg)

def _heartbeat_position(node):
    msg = OffboardControlMode()
    msg.position   = True
    msg.body_rate  = False
    msg.timestamp  = int(node.get_clock().now().nanoseconds / 1000)
    node.offboard_control_mode_publisher.publish(msg)

def _heartbeat_bodyrate(node):
    msg = OffboardControlMode()
    msg.position  = False
    msg.body_rate = True
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    node.offboard_control_mode_publisher.publish(msg)


def _wrap_to_pi(angle: float) -> float:
    return (angle + m.pi) % (2.0 * m.pi) - m.pi


class FeedforwardControl(Node):
    """Flatness-based controller for contraction trajectories."""

    def __init__(
        self,
        platform_type: PlatformType,
        trajectory_type: TrajectoryType = TrajectoryType.FIG8_CONTRACTION,
        double_speed: bool = False,
        p_feedback: bool = False,
        ramp_seconds: float = 2.0,
        logging_enabled: bool = False,
        flight_period_: float | None = None,
    ) -> None:
        if trajectory_type not in SUPPORTED_TRAJECTORY_TYPES:
            supported = ", ".join(sorted(traj.value for traj in SUPPORTED_TRAJECTORY_TYPES))
            raise ValueError(
                f"Unsupported trajectory {trajectory_type.value!r} for ff_f8_px4. "
                f"Expected one of: {supported}"
            )

        super().__init__("fbl_offboard_node")
        self.get_logger().info("Initializing FeedforwardControl node")

        self.sim             = platform_type == PlatformType.SIM
        self.platform_type   = platform_type
        self.trajectory_type = trajectory_type
        self.trajectory_name = trajectory_type.value
        self.double_speed    = double_speed
        self.p_feedback      = p_feedback
        self.ramp_seconds    = max(float(ramp_seconds), 0.0)
        self.logging_enabled = logging_enabled

        flight_period = flight_period_ if flight_period_ is not None else (30.0 if self.sim else 60.0)

        # Platform
        self.platform: PlatformConfig = PLATFORM_REGISTRY[platform_type]()

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos)

        # Subscriber state
        self.mocap_initialized  = False
        self.mocap_k            = -1
        self.full_rotations     = 0
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.wx = self.wy = self.wz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.in_offboard_mode       = False
        self.armed                  = False
        self.in_land_mode           = False
        self.offboard_mode_rc_switch_on = True if self.sim else False
        self.mode_channel           = 5

        # Subscribers
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._odom_cb, qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1",
                                 self._status_cb, qos)
        self.create_subscription(RcChannels, "/fmu/out/rc_channels",
                                 self._rc_cb, qos)
        # Flight timing
        self.T0              = time.time()
        self.program_time    = 0.0
        self.cushion_period  = 10.0
        self.flight_period   = flight_period
        self.land_time       = self.flight_period + 2 * self.cushion_period
        self.flight_phase    = self._get_phase()
        print(f"Flight time: {self.land_time}s")

        # Control state
        self.HOVER_HEIGHT       = 3.0 if self.sim else 0.7
        self.LAND_HEIGHT        = 0.6 if self.sim else 0.45
        self.trajectory_started = False
        self.trajectory_time    = 0.0
        self.compute_time       = 0.0
        self.hover_ref          = (0.0, 0.0, -self.HOVER_HEIGHT, 0.0)

        first_thrust = self.platform.mass * GRAVITY
        self.hover_force = first_thrust
        self.normalized_input = [self.platform.get_throttle_from_force(first_thrust), 0.0, 0.0, 0.0]
        self._x_ff = None

        # Optional light feedback on top of feedforward for hover -> trajectory
        # handoff and disturbance rejection.
        self.kp_xy = 0.12
        self.kv_xy = 0.18
        self.kp_z = 0.35
        self.kv_z = 0.28
        self.kp_att = 1.2
        self.kp_yaw = 1.5
        self.kd_body_rates = 0.18
        self.max_tilt_cmd = 0.35

        # Feedforward JIT
        self._ff_jit = None

        self.offboard_setpoint_counter = 0

        # Timers
        self.create_timer(0.1, self._offboard_timer_cb)
        self.create_timer(0.01, self._publish_ctrl_cb)
        self.create_timer(0.01, self._compute_ctrl_cb)
        if logging_enabled:
            self.create_timer(0.1, self._data_log_cb)

        # JIT warm-up
        self._jit_compile()
        print("[FeedforwardControl] Node initialized successfully!\n")
        time.sleep(3)
        self.T0 = time.time()

        # Logging setup
        if logging_enabled:
            print("Data logging is ON")
            self.platform_logtype    = LogType("platform",      0)
            self.controller_logtype  = LogType("controller",    1)
            self.trajectory_logtype  = LogType("trajectory",    2)
            self.traj_double_logtype = LogType("traj_double",   3)
            self.lookahead_time_logtype = LogType("lookahead_time", 4)

            self.platform_logtype.append(self.platform_type.value.upper())
            self.controller_logtype.append(self._controller_log_label())
            self.trajectory_logtype.append(self.trajectory_name)
            self.traj_double_logtype.append("DblSpd" if self.double_speed else "NormSpd")
            self.lookahead_time_logtype.append(0.0)  # no lookahead for open-loop

            self.program_time_logtype    = LogType("time",       10)
            self.trajectory_time_logtype = LogType("traj_time",  11)
            self.comp_time_logtype       = LogType("comp_time",  13)

            self.x_logtype   = LogType("x",   20)
            self.y_logtype   = LogType("y",   21)
            self.z_logtype   = LogType("z",   22)
            self.yaw_logtype = LogType("yaw", 23)
            self.vx_logtype  = LogType("vx",  24)
            self.vy_logtype  = LogType("vy",  25)
            self.vz_logtype  = LogType("vz",  26)

            self.xref_logtype   = LogType("x_ref",   30)
            self.yref_logtype   = LogType("y_ref",   31)
            self.zref_logtype   = LogType("z_ref",   32)
            self.yawref_logtype = LogType("yaw_ref", 33)
            self.vxref_logtype  = LogType("vx_ref",  34)
            self.vyref_logtype  = LogType("vy_ref",  35)
            self.vzref_logtype  = LogType("vz_ref",  36)

            self.p_logtype = LogType("p", 40)
            self.q_logtype = LogType("q", 41)
            self.r_logtype = LogType("r", 42)

            self.throttle_input_logtype = LogType("throttle_input", 50)
            self.p_input_logtype        = LogType("p_input",        51)
            self.q_input_logtype        = LogType("q_input",        52)
            self.r_input_logtype        = LogType("r_input",        53)

    # ------------------------------------------------------------------ #
    #  JIT warm-up                                                         #
    # ------------------------------------------------------------------ #
    def _jit_compile(self):
        print("\n[JIT Compilation] Pre-compiling flat_to_x_u...")
        ctx = TrajContext(sim=self.sim, hover_mode=None, spin=False,
                          double_speed=self.double_speed, short=False)

        def _time_warp(t):
            if self.ramp_seconds <= 0.0:
                return t
            T = self.ramp_seconds
            return jnp.where(t < T, 0.5 * t * t / T, t - 0.5 * T)

        flat_output = lambda t: TRAJ_REGISTRY[self.trajectory_type](_time_warp(t), ctx)
        self._ff_jit = jax.jit(lambda t: flat_to_x_u(t, flat_output))

        def _timed(t):
            start = time.perf_counter()
            x, u = self._ff_jit(t)
            jax.block_until_ready((x, u))
            return x, u, time.perf_counter() - start

        x, u, t1 = _timed(0.0)
        print(f"  flat_to_x_u (NO JIT): {t1:.4f}s")
        x, u, t2 = _timed(0.0)
        print(f"  flat_to_x_u (JIT):    {t2:.4f}s")
        print(f"  Speed-up: {t1/t2:.1f}x   Good for {1.0/t2:.0f} Hz")

        # Use the actual first trajectory point for hover/return positioning.
        x0 = TRAJ_REGISTRY[self.trajectory_type](0.0, ctx)
        self.hover_ref = (float(x0[0]), float(x0[1]), float(x0[2]), float(x0[3]))

    def _controller_log_label(self) -> str:
        if self.p_feedback:
            return "fbl"
        if self.trajectory_type == TrajectoryType.FIG8_CONTRACTION:
            return "ff_f8"
        return "flat_ff"

    # ------------------------------------------------------------------ #
    #  Subscriber callbacks                                                #
    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg):
        self.x, self.y, self.z   = msg.position
        self.vx, self.vy, self.vz = msg.velocity
        self.wx, self.wy, self.wz = msg.angular_velocity

        ori = R.from_quat(msg.q, scalar_first=True)
        self.roll, self.pitch, raw_yaw = ori.as_euler("xyz", degrees=False)
        self.yaw = self._adjust_yaw(raw_yaw)

        self.get_logger().info(
            f"State: [{self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.yaw:.2f}]",
            throttle_duration_sec=0.3)

    def _adjust_yaw(self, yaw: float) -> float:
        if not self.mocap_initialized:
            self.mocap_initialized = True
            self._prev_yaw = yaw
            return yaw
        if self._prev_yaw > m.pi * 0.9 and yaw < -m.pi * 0.9:
            self.full_rotations += 1
        elif self._prev_yaw < -m.pi * 0.9 and yaw > m.pi * 0.9:
            self.full_rotations -= 1
        self._prev_yaw = yaw
        return yaw + 2 * m.pi * self.full_rotations

    def _status_cb(self, msg):
        self.in_offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.armed            = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.in_land_mode     = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

    def _rc_cb(self, msg):
        self.offboard_mode_rc_switch_on = (msg.channels[self.mode_channel - 1] >= 0.75)

    # ------------------------------------------------------------------ #
    #  Flight phase helpers                                                #
    # ------------------------------------------------------------------ #
    def _get_phase(self) -> FlightPhase:
        if self.program_time < self.cushion_period:
            return FlightPhase.HOVER
        elif self.program_time < self.cushion_period + self.flight_period:
            return FlightPhase.CUSTOM
        elif self.program_time < self.land_time:
            return FlightPhase.RETURN
        else:
            return FlightPhase.LAND

    def _killswitch(self) -> bool:
        if not self.offboard_mode_rc_switch_on:
            self.get_logger().warning(
                f"RC channel {self.mode_channel} not set to offboard",
                throttle_duration_sec=1.0)
            self.offboard_setpoint_counter = 0
            return False
        self.program_time = time.time() - self.T0
        self.flight_phase = self._get_phase()
        self.get_logger().warn(
            f"[{self.program_time:.2f}s] phase={self.flight_phase.name}",
            throttle_duration_sec=0.5)
        return True

    def _health(self) -> bool:
        ok = True
        if not self.in_offboard_mode:
            self.get_logger().warning("NOT in OFFBOARD mode.")
            ok = False
        if not self.armed:
            self.get_logger().warning("NOT ARMED.")
            ok = False
        if not self.mocap_initialized:
            self.get_logger().warning("No odometry yet.")
            ok = False
        return ok

    # ------------------------------------------------------------------ #
    #  Timer callbacks                                                     #
    # ------------------------------------------------------------------ #
    def _offboard_timer_cb(self):
        if not self._killswitch():
            return
        if self.offboard_setpoint_counter == 10:
            _engage_offboard(self)
            _arm(self)
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.flight_phase in (FlightPhase.HOVER, FlightPhase.RETURN, FlightPhase.LAND):
            _heartbeat_position(self)
        else:
            _heartbeat_bodyrate(self)

    def _publish_ctrl_cb(self):
        if self.in_land_mode:
            self.get_logger().info("In land mode...")
            threshold = 0.71 if self.sim else 0.64
            if abs(self.z) < threshold:
                _disarm(self)
                exit(0)

        if not self._killswitch():
            return
        if not self._health():
            return

        if self.flight_phase is FlightPhase.HOVER:
            self._pub_pos(*self.hover_ref)
        elif self.flight_phase is FlightPhase.CUSTOM:
            self._pub_rates(*self.normalized_input)
        elif self.flight_phase is FlightPhase.RETURN:
            self._pub_pos(*self.hover_ref)
        elif self.flight_phase is FlightPhase.LAND:
            self._pub_pos(0.0, 0.0, -self.LAND_HEIGHT, 0.0)
            if abs(self.z) < 0.64:
                _land(self)

    def _compute_ctrl_cb(self):
        if not self._killswitch():
            return
        if not self._health():
            return
        if self._get_phase() is not FlightPhase.CUSTOM:
            return

        if not self.trajectory_started:
            self.trajectory_T0      = time.time()
            self.trajectory_time    = 0.0
            self.trajectory_started = True

        self.trajectory_time = time.time() - self.trajectory_T0

        t0 = time.time()
        self._compute_ff()
        self.compute_time = time.time() - t0

        self.get_logger().warn(
            f"FF compute: {self.compute_time*1000:.2f}ms  "
            f"({1.0/self.compute_time:.0f} Hz)",
            throttle_duration_sec=0.3)

    def _data_log_cb(self):
        if self.flight_phase is not FlightPhase.CUSTOM:
            return
        self.program_time_logtype.append(self.program_time)
        self.trajectory_time_logtype.append(self.trajectory_time)
        self.comp_time_logtype.append(self.compute_time)

        self.x_logtype.append(self.x)
        self.y_logtype.append(self.y)
        self.z_logtype.append(self.z)
        self.yaw_logtype.append(self.yaw)
        self.vx_logtype.append(self.vx)
        self.vy_logtype.append(self.vy)
        self.vz_logtype.append(self.vz)

        self.xref_logtype.append(float(self._x_ff[0]))
        self.yref_logtype.append(float(self._x_ff[1]))
        self.zref_logtype.append(float(self._x_ff[2]))
        self.yawref_logtype.append(float(self._x_ff[9]))
        self.vxref_logtype.append(float(self._x_ff[3]))
        self.vyref_logtype.append(float(self._x_ff[4]))
        self.vzref_logtype.append(float(self._x_ff[5]))

        self.p_logtype.append(self.wx)
        self.q_logtype.append(self.wy)
        self.r_logtype.append(self.wz)

        self.throttle_input_logtype.append(self.normalized_input[0])
        self.p_input_logtype.append(self.normalized_input[1])
        self.q_input_logtype.append(self.normalized_input[2])
        self.r_input_logtype.append(self.normalized_input[3])

    # ------------------------------------------------------------------ #
    #  Core feedforward computation                                        #
    # ------------------------------------------------------------------ #
    def _compute_ff(self):
        """Compute and publish [throttle, p, q, r] from flat-output inversion."""
        x_ff, u_ff = self._ff_jit(self.trajectory_time)
        self._x_ff = x_ff  # cache for logging

        # x_ff = [px, py, pz, vx, vy, vz, f_specific, phi, th, psi]
        # u_ff = [df, dphi, dth, dpsi]
        f_specific = float(x_ff[6])
        phi_ff     = float(x_ff[7])   # roll from flat inversion
        th_ff      = float(x_ff[8])   # pitch from flat inversion

        dphi_cmd = float(u_ff[1])
        dth_cmd = float(u_ff[2])
        dpsi_cmd = float(u_ff[3])

        if self.p_feedback:
            x_ref = float(x_ff[0])
            y_ref = float(x_ff[1])
            z_ref = float(x_ff[2])
            vx_ref = float(x_ff[3])
            vy_ref = float(x_ff[4])
            vz_ref = float(x_ff[5])
            psi_ref = float(x_ff[9])

            x_err = x_ref - self.x
            y_err = y_ref - self.y
            z_err = z_ref - self.z
            vx_err = vx_ref - self.vx
            vy_err = vy_ref - self.vy
            vz_err = vz_ref - self.vz
            yaw_err = _wrap_to_pi(psi_ref - self.yaw)

            phi_des = np.clip(
                phi_ff + self.kp_xy * y_err + self.kv_xy * vy_err,
                -self.max_tilt_cmd,
                self.max_tilt_cmd,
            )
            th_des = np.clip(
                th_ff - self.kp_xy * x_err - self.kv_xy * vx_err,
                -self.max_tilt_cmd,
                self.max_tilt_cmd,
            )

            dphi_cmd += self.kp_att * (phi_des - self.roll)
            dth_cmd += self.kp_att * (th_des - self.pitch)
            dpsi_cmd += self.kp_yaw * yaw_err
            f_specific += -self.kp_z * z_err - self.kv_z * vz_err

        # Convert Euler rates -> body rates using the current attitude when
        # feedback is enabled, otherwise use the nominal flatness attitude.
        roll_for_T = self.roll if self.p_feedback else phi_ff
        pitch_for_T = self.pitch if self.p_feedback else th_ff
        sr, cr = m.sin(roll_for_T), m.cos(roll_for_T)
        sp, cp = m.sin(pitch_for_T), m.cos(pitch_for_T)
        cp = max(cp, 1e-3)
        tp = sp / cp
        T = np.array([
            [1.,  sr * tp,  cr * tp],
            [0.,  cr,      -sr     ],
            [0.,  sr / cp,  cr / cp],
        ])
        euler_rates = np.array([dphi_cmd, dth_cmd, dpsi_cmd])
        body_rates  = np.linalg.solve(T, euler_rates)  # [p_ff, q_ff, r_ff]

        if self.p_feedback:
            body_rates = body_rates - self.kd_body_rates * np.array([self.wx, self.wy, self.wz])

        # Thrust: f_specific (m/s²) → force (N) → normalized throttle
        force   = self.platform.mass * max(f_specific, 0.1)
        throttle_raw = self.platform.get_throttle_from_force(force)
        throttle  = throttle_raw

        cmd_ff = np.array([
            float(np.clip(throttle, 0.0, 1.0)),
            float(np.clip(body_rates[0], -0.6, 0.6)),
            float(np.clip(body_rates[1], -0.6, 0.6)),
            float(np.clip(body_rates[2], -0.6, 0.6)),
        ])

        hover_throttle = self.platform.get_throttle_from_force(self.hover_force)
        cmd_hover = np.array([float(np.clip(hover_throttle, 0.0, 1.0)), 0.0, 0.0, 0.0])

        if self.ramp_seconds > 0.0:
            s = float(np.clip(self.trajectory_time / self.ramp_seconds, 0.0, 1.0))
            alpha = s * s * (3.0 - 2.0 * s)
        else:
            alpha = 1.0

        cmd = (1.0 - alpha) * cmd_hover + alpha * cmd_ff
        self.normalized_input = [float(cmd[0]), float(cmd[1]), float(cmd[2]), float(cmd[3])]

    # ------------------------------------------------------------------ #
    #  Setpoint publishers                                                 #
    # ------------------------------------------------------------------ #
    def _pub_pos(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw      = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(
            f"Position setpoint: [{x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f}]",
            throttle_duration_sec=1.0)

    def _pub_rates(self, thrust, roll, pitch, yaw):
        msg = VehicleRatesSetpoint()
        msg.roll  = float(roll)
        msg.pitch = float(pitch)
        msg.yaw   = float(yaw)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -float(thrust)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.rates_setpoint_publisher.publish(msg)
        self.get_logger().info(
            f"Rates setpoint [thr,p,q,r]: [{thrust:.3f}, {roll:.3f}, {pitch:.3f}, {yaw:.3f}]",
            throttle_duration_sec=1.0)
