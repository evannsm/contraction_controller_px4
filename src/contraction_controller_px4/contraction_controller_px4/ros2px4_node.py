"""ROS2 node for the contraction-metric neural-network quadrotor controller."""

import os
import time
from pathlib import Path

import jax
import jax.numpy as jnp
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    RcChannels,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
)

from contraction_controller_px4_utils.px4_utils.core_funcs import (
    arm,
    disarm,
    engage_offboard_mode,
    land,
    publish_offboard_control_heartbeat_signal_bodyrate,
    publish_offboard_control_heartbeat_signal_position,
)
from contraction_controller_px4_utils.px4_utils.flight_phases import FlightPhase
from contraction_controller_px4_utils.transformations.adjust_yaw import adjust_yaw

from quad_platforms import PlatformConfig, PlatformType, PLATFORM_REGISTRY  # type: ignore[import]
from quad_trajectories import TrajContext, TrajectoryType, TRAJ_REGISTRY, GRAVITY, flat_to_x_u, flat_to_x

from .controller import (
    contraction_control,
    compute_lqr_gain,
    load_control_net,
    K_EQ,
    X_EQ,
)
from ros2_logger import LogType, VectorLogType  # type: ignore[import]


class ContractionOffboardControl(Node):
    def __init__(
        self,
        platform_type: PlatformType,
        trajectory: TrajectoryType = TrajectoryType.HOVER_CONTRACTION,
        hover_mode: int | None = None,
        controller_dir: str | None = None,
        flight_period_: float | None = None,
        logging_enabled: bool = False,
        use_feedforward: bool = True,
    ) -> None:
        super().__init__("contraction_offboard_control_node")
        self.get_logger().info("Initializing Contraction Controller ROS2 node.")

        self.sim = platform_type == PlatformType.SIM
        self.platform_type = platform_type
        self.trajectory_type = trajectory
        self.use_feedforward = use_feedforward
        self.hover_mode = hover_mode
        flight_period = flight_period_ if flight_period_ is not None else (30.0 if self.sim else 60.0)

        # Platform config
        platform_class = PLATFORM_REGISTRY[self.platform_type]
        self.platform: PlatformConfig = platform_class()

        # Trajectory enum
        traj_map = {t.value: t for t in TrajectoryType}
        if trajectory not in traj_map:
            raise ValueError(f"Unknown trajectory: {trajectory}")
        self.ref_type = traj_map[trajectory]

        # ── QoS ──────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.mocap_initialized: bool = False
        self.full_rotations: int = 0
        self.prev_mocap_psi: float = 0.0
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, qos
        )

        self.in_offboard_mode: bool = False
        self.armed: bool = False
        self.in_land_mode: bool = False
        self.vehicle_status = None
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status_callback, qos
        )

        self.offboard_mode_rc_switch_on = True if self.sim else False
        self.mode_channel = 5
        self.rc_channels_subscriber = self.create_subscription(
            RcChannels, "/fmu/out/rc_channels", self.rc_channel_callback, qos
        )

        self.a_world = np.array([0.0, 0.0, GRAVITY])  # NED world-frame acceleration (m/s^2)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_callback, qos
        )

        # ── Flight phases and timing ──────────────────────────────────────────
        self.T0 = time.time()
        self.program_time: float = 0.0
        self.cushion_period = 10.0
        self.flight_period = flight_period
        self.land_time = self.flight_period + 2 * self.cushion_period

        # ── Control state ─────────────────────────────────────────────────────
        self.HOVER_HEIGHT = 3.0 if self.sim else 0.7
        self.LAND_HEIGHT = 0.6 if self.sim else 0.45
        self.trajectory_started: bool = False
        self.trajectory_time: float = 0.0
        self.reference_time: float = 0.0
        self.T_LOOKAHEAD = 0.0  # contraction controller uses current reference (no lookahead)

        first_thrust = self.platform.mass * GRAVITY
        self.last_input = np.array([first_thrust, 0.0, 0.0, 0.0])
        self.normalized_input = [self.platform.get_throttle_from_force(first_thrust), 0.0, 0.0, 0.0]

        # ── Load neural-network controller ────────────────────────────────────
        if controller_dir is None:
            controller_dir = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "params",
            )
        self.controller_dir = Path(controller_dir).resolve()
        self.get_logger().info(f"Loading neural network from: {self.controller_dir}")
        self.control_net = load_control_net(self.controller_dir)
        self.get_logger().info("Neural network loaded successfully.")

        # ── Build trajectory callable (created once; reused every tick) ───────
        _ctx = TrajContext(sim=self.sim, hover_mode=self.hover_mode)
        _traj_func = TRAJ_REGISTRY[self.ref_type]
        self._flat_output = lambda t: _traj_func(t, _ctx)
        if self.use_feedforward:
            self._flat_to_x_u_jit = jax.jit(lambda t: flat_to_x_u(t, self._flat_output))
        else:
            self._flat_to_x_jit = jax.jit(lambda t: flat_to_x(t, self._flat_output))

        # JIT-compile contraction_control with control_net captured in closure
        # (control_net is a fixed PyTree; K changes at 10 Hz so stays dynamic)
        self._contraction_control_jit = jax.jit(
            lambda x, x_ff, u_ff, K: contraction_control(x, x_ff, u_ff, self.control_net, K)
        )

        # ── JIT warm-up ───────────────────────────────────────────────────────
        self._jit_warmup()

        # ── Timers ────────────────────────────────────────────────────────────
        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.offboard_mode_timer_callback)
        self.controller_period = 0.01
        self.publish_control_timer = self.create_timer(self.controller_period, self.publish_control_timer_callback)
        self.compute_control_timer = self.create_timer(self.controller_period, self.compute_control_timer_callback)

        self.compute_time: float = 0.0
        self.x_ff_last: np.ndarray = np.array(X_EQ)
        self.u_ff_last: np.ndarray = np.zeros(4)

        # ── LQR state ─────────────────────────────────────────────────────────
        self._K_current: jnp.ndarray = K_EQ

        self.lqr_timer = self.create_timer(0.1, self._lqr_update_callback)  # 10 Hz

        # ── Logging ───────────────────────────────────────────────────────────
        self.logging_enabled = logging_enabled
        if self.logging_enabled:
            # Metadata (written once)
            self.platform_logtype      = LogType("platform",   0)
            self.trajectory_logtype    = LogType("trajectory",  1)
            self.controller_logtype    = LogType("controller",  2)
            self.feedforward_logtype  = LogType("feedforward",  3)

            self.platform_logtype.append(platform_type.value.upper())
            self.trajectory_logtype.append(trajectory.value)
            self.controller_logtype.append("contraction")
            self.feedforward_logtype.append(1 if use_feedforward else 0)

            # Timing
            self.time_logtype          = LogType("time",        10)
            self.traj_time_logtype     = LogType("traj_time",   11)
            self.comp_time_logtype     = LogType("comp_time",   12)

            # True odometry state (from VehicleOdometry)
            self.x_logtype             = LogType("x",           20)
            self.y_logtype             = LogType("y",           21)
            self.z_logtype             = LogType("z",           22)
            self.yaw_logtype           = LogType("yaw",         23)
            self.vx_logtype            = LogType("vx",          24)
            self.vy_logtype            = LogType("vy",          25)
            self.vz_logtype            = LogType("vz",          26)
            self.roll_logtype          = LogType("roll",        27)
            self.pitch_logtype         = LogType("pitch",       28)
            self.f_logtype             = LogType("f",           29)  # specific thrust (N/kg)
            # True body rates (from odometry angular_velocity)
            self.p_logtype             = LogType("p",           30)
            self.q_logtype             = LogType("q",           31)
            self.r_logtype             = LogType("r",           32)

            # Reference state (x_ff from flat_to_x_u)
            self.x_ref_logtype         = LogType("x_ref",       40)
            self.y_ref_logtype         = LogType("y_ref",       41)
            self.z_ref_logtype         = LogType("z_ref",       42)
            self.yaw_ref_logtype       = LogType("yaw_ref",     43)
            self.vx_ref_logtype        = LogType("vx_ref",      44)
            self.vy_ref_logtype        = LogType("vy_ref",      45)
            self.vz_ref_logtype        = LogType("vz_ref",      46)
            self.roll_ref_logtype      = LogType("roll_ref",    47)
            self.pitch_ref_logtype     = LogType("pitch_ref",   48)
            self.f_ref_logtype         = LogType("f_ref",       49)

            # Reference control (u_ff from flat_to_x_u): rates of [f, phi, th, psi]
            self.u_ff_logtype          = VectorLogType("u_ff",  50, ["df", "dphi", "dth", "dpsi"])

            # Commanded inputs sent to PX4
            self.throttle_logtype      = LogType("throttle",    60)
            self.p_cmd_logtype         = LogType("p_cmd",       61)
            self.q_cmd_logtype         = LogType("q_cmd",       62)
            self.r_cmd_logtype         = LogType("r_cmd",       63)

            self.data_log_timer = self.create_timer(0.1, self._data_log_callback)

        self.T0 = time.time()
        self.get_logger().info("Contraction controller node ready.")
        time.sleep(2)

    # ── Warm-up ───────────────────────────────────────────────────────────────
    def _jit_warmup(self) -> None:
        if self.use_feedforward:
            self.get_logger().info("[JIT] Pre-compiling flat_to_x_u (with FF)...")
            t0 = time.perf_counter()
            x_ff, u_ff = self._flat_to_x_u_jit(0.0)
            jax.block_until_ready(x_ff); jax.block_until_ready(u_ff)
            t1 = time.perf_counter()
            self.get_logger().info(f"  Trace:  {(t1-t0)*1e3:.1f} ms")

            t0 = time.perf_counter()
            x_ff, u_ff = self._flat_to_x_u_jit(0.1)
            jax.block_until_ready(x_ff); jax.block_until_ready(u_ff)
            t1 = time.perf_counter()
            self.get_logger().info(f"  JIT:    {(t1-t0)*1e3:.1f} ms  x_ff={x_ff}")
        else:
            self.get_logger().info("[JIT] Pre-compiling flat_to_x (no FF)...")
            t0 = time.perf_counter()
            x_ff = self._flat_to_x_jit(0.0)
            jax.block_until_ready(x_ff)
            t1 = time.perf_counter()
            self.get_logger().info(f"  Trace:  {(t1-t0)*1e3:.1f} ms")

            t0 = time.perf_counter()
            x_ff = self._flat_to_x_jit(0.1)
            jax.block_until_ready(x_ff)
            t1 = time.perf_counter()
            self.get_logger().info(f"  JIT:    {(t1-t0)*1e3:.1f} ms  x_ff={x_ff}")
            u_ff = jnp.zeros(4, dtype=jnp.float32)

        self.get_logger().info("[JIT] Pre-compiling contraction control function...")
        dummy_K = K_EQ
        dummy_x   = jnp.array(X_EQ, dtype=jnp.float32)
        dummy_xff = jnp.array(X_EQ, dtype=jnp.float32)
        dummy_uff = jnp.zeros(4, dtype=jnp.float32)
        t0 = time.perf_counter()
        u = self._contraction_control_jit(dummy_x, dummy_xff, dummy_uff, dummy_K)
        jax.block_until_ready(u)
        t1 = time.perf_counter()
        self.get_logger().info(f"  Trace:  {(t1-t0)*1e3:.1f} ms")

        t0 = time.perf_counter()
        u = self._contraction_control_jit(dummy_x, dummy_xff, dummy_uff, dummy_K)
        jax.block_until_ready(u)
        t1 = time.perf_counter()
        self.get_logger().info(f"  JIT:    {(t1-t0)*1e3:.1f} ms  → {1.0/(t1-t0):.0f} Hz capable")

        self.get_logger().info("[LQR] Pre-compiling Jacobian + warming up scipy CARE solver...")
        t0 = time.perf_counter()
        K = compute_lqr_gain(dummy_xff, dummy_uff)
        t1 = time.perf_counter()
        self.get_logger().info(f"  First call (JIT trace + CARE): {(t1-t0)*1e3:.1f} ms")

        t0 = time.perf_counter()
        K = compute_lqr_gain(dummy_xff, dummy_uff)
        t1 = time.perf_counter()
        self.get_logger().info(f"  Second call (JIT + CARE):      {(t1-t0)*1e3:.1f} ms")
        # Do NOT assign _K_current here — leave it as K_EQ until LQR is validated

    # ── Subscriber callbacks ──────────────────────────────────────────────────
    def vehicle_odometry_callback(self, msg):
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]
        self.vx = msg.velocity[0]
        self.vy = msg.velocity[1]
        self.vz = msg.velocity[2]
        self.wx = msg.angular_velocity[0]
        self.wy = msg.angular_velocity[1]
        self.wz = msg.angular_velocity[2]

        orientation = R.from_quat(msg.q, scalar_first=True)
        self.roll, self.pitch, self._yaw = orientation.as_euler("xyz", degrees=False)
        self.yaw = adjust_yaw(self, self._yaw)

        self.position = np.array([self.x, self.y, self.z])
        self.velocity = np.array([self.vx, self.vy, self.vz])
        self.euler_angle_total_yaw = np.array([self.roll, self.pitch, self.yaw])

        # 10D contraction state: [px,py,pz, vx,vy,vz, az, roll,pitch,yaw]
        # az: specific thrust (N/kg). vehicle_local_position gives kinematic acceleration
        # (gravity removed), so subtract gravity to get specific force in NED, rotate to
        # body frame, then negate z (thrust acts in -z_body → specific force z = -f).
        a_specific_ned = self.a_world - np.array([0.0, 0.0, GRAVITY])
        a_specific_body = orientation.inv().apply(a_specific_ned)
        az = -a_specific_body[2]
        self.contraction_state = np.hstack((
            self.position,
            self.velocity,
            [az],
            self.euler_angle_total_yaw,
        ))

        self.mocap_initialized = True
        self.get_logger().info(
            f"State: pos={self.position}, yaw={self.yaw:.3f}",
            throttle_duration_sec=0.3,
        )

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status
        self.in_offboard_mode = (
            vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        self.armed = vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.in_land_mode = (
            vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        )

    def rc_channel_callback(self, rc_channels):
        flight_mode = rc_channels.channels[self.mode_channel - 1]
        self.offboard_mode_rc_switch_on = flight_mode >= 0.75

    def vehicle_local_position_callback(self, msg):
        self.a_world = np.array([msg.ax, msg.ay, msg.az])

    # ── Flight phase helpers ──────────────────────────────────────────────────
    def get_phase(self) -> FlightPhase:
        if self.program_time < self.cushion_period:
            return FlightPhase.HOVER
        elif self.program_time < self.cushion_period + self.flight_period:
            return FlightPhase.CUSTOM
        elif self.program_time < self.land_time:
            return FlightPhase.RETURN
        else:
            return FlightPhase.LAND

    def time_before_next_phase(self, phase: FlightPhase) -> float:
        if phase == FlightPhase.HOVER:
            return self.cushion_period - self.program_time
        elif phase == FlightPhase.CUSTOM:
            return (self.cushion_period + self.flight_period) - self.program_time
        elif phase == FlightPhase.RETURN:
            return self.land_time - self.program_time
        return 0.0

    def killswitch_and_flight_phase(self) -> bool:
        if not self.offboard_mode_rc_switch_on:
            self.get_logger().warning(
                f"RC channel {self.mode_channel} not in offboard position.", throttle_duration_sec=1.0
            )
            self.offboard_setpoint_counter = 0
            return False
        self.program_time = time.time() - self.T0
        self.flight_phase = self.get_phase()
        self.get_logger().warn(
            f"[{self.program_time:.1f}s] Phase: {self.flight_phase.name}, "
            f"next in {self.time_before_next_phase(self.flight_phase):.1f}s",
            throttle_duration_sec=0.5,
        )
        return True

    def get_offboard_health(self) -> bool:
        ok = True
        if not self.in_offboard_mode:
            self.get_logger().warning("NOT in OFFBOARD mode.")
            ok = False
        if not self.armed:
            self.get_logger().warning("NOT ARMED.")
            ok = False
        if not self.mocap_initialized:
            self.get_logger().warning("Odometry NOT received.")
            ok = False
        return ok

    # ── Timer callbacks ───────────────────────────────────────────────────────
    def offboard_mode_timer_callback(self) -> None:
        if not self.killswitch_and_flight_phase():
            return
        if self.offboard_setpoint_counter == 10:
            engage_offboard_mode(self)
            arm(self)
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        phase = self.flight_phase
        if phase is FlightPhase.HOVER:
            publish_offboard_control_heartbeat_signal_position(self)
        elif phase is FlightPhase.CUSTOM:
            publish_offboard_control_heartbeat_signal_bodyrate(self)
        elif phase in (FlightPhase.RETURN, FlightPhase.LAND):
            publish_offboard_control_heartbeat_signal_position(self)

    def publish_control_timer_callback(self) -> None:
        if self.in_land_mode:
            self.get_logger().info("In land mode...", throttle_duration_sec=1.0)
            threshold = 0.71 if self.sim else 0.64
            if abs(self.z) < threshold:
                self.get_logger().info("Landed, disarming.")
                disarm(self)
                rclpy.shutdown()
                return

        if not self.killswitch_and_flight_phase():
            return
        if not self.get_offboard_health():
            return

        phase = self.flight_phase
        if phase is FlightPhase.HOVER:
            self._publish_position(0.0, 0.0, -self.HOVER_HEIGHT, 0.0)
        elif phase is FlightPhase.CUSTOM:
            self._publish_rates(*self.normalized_input)
        elif phase is FlightPhase.RETURN:
            self._publish_position(0.0, 0.0, -self.HOVER_HEIGHT, 0.0)
        elif phase is FlightPhase.LAND:
            self._publish_position(0.0, 0.0, -self.LAND_HEIGHT, 0.0)
            if abs(self.z) < 0.64:
                land(self)

    def compute_control_timer_callback(self) -> None:
        if not self.killswitch_and_flight_phase():
            return
        if not self.get_offboard_health():
            return
        if self.get_phase() is not FlightPhase.CUSTOM:
            return

        if not self.trajectory_started:
            self.trajectory_T0 = time.time()
            self.trajectory_time = 0.0
            self.trajectory_started = True

        self.trajectory_time = time.time() - self.trajectory_T0
        self.reference_time = self.trajectory_time  # no lookahead for contraction

        t = float(self.reference_time)  # float64 for precision in autodiff
        if self.use_feedforward:
            x_ff, u_ff = self._flat_to_x_u_jit(t)
        else:
            x_ff = self._flat_to_x_jit(t)
            u_ff = jnp.zeros(4, dtype=jnp.float32)

        t0 = time.time()
        u_raw = self._contraction_control_jit(
            jnp.array(self.contraction_state, dtype=jnp.float32),
            x_ff,
            u_ff,
            self._K_current,
        )
        jax.block_until_ready(u_raw)
        self.compute_time = time.time() - t0
        self.get_logger().info(f"Control computed. Good for {1.0/self.compute_time:.1f} Hz", throttle_duration_sec=0.5)
        compute_time = self.compute_time
        self.x_ff_last = np.array(x_ff)
        self.u_ff_last = np.array(u_ff)

        # u_raw[0] is df (specific force rate, N/kg/s); scale by mass to get thrust rate (N/s)
        f_dot = float(u_raw[0])
        thrust = self.last_input[0] + (self.controller_period) * (f_dot * self.platform.mass)
        self.last_input = np.array([thrust, float(u_raw[1]), float(u_raw[2]), float(u_raw[3])])
        self.get_logger().warning(
            f"Compute: {compute_time*1e3:.2f} ms  u={np.array(u_raw)}  thrust={thrust:.3f}",
            throttle_duration_sec=0.5,
        )

        # Convert to PX4 normalized inputs
        throttle_raw = float(self.platform.get_throttle_from_force(thrust))
        throttle = throttle_raw

        self.normalized_input = [
            throttle,
            float(u_raw[1]),
            float(u_raw[2]),
            float(u_raw[3]),
        ]

    # ── LQR gain update (10 Hz) ───────────────────────────────────────────────
    def _lqr_update_callback(self) -> None:
        if not self.trajectory_started:
            return
        try:
            self._K_current = compute_lqr_gain(
                jnp.array(self.contraction_state, dtype=jnp.float32),
                jnp.array(self.u_ff_last, dtype=jnp.float32),
            )
        except Exception as e:
            self.get_logger().warning(
                f"[LQR] CARE solve failed, keeping previous K: {e}",
                throttle_duration_sec=1.0,
            )

    # ── Data logging ─────────────────────────────────────────────────────────
    def _data_log_callback(self) -> None:
        if self.get_phase() is not FlightPhase.CUSTOM:
            return
        if not self.mocap_initialized:
            return

        # Timing
        self.time_logtype.append(self.program_time)
        self.traj_time_logtype.append(self.trajectory_time)
        self.comp_time_logtype.append(self.compute_time)

        # True state from odometry
        self.x_logtype.append(self.x)
        self.y_logtype.append(self.y)
        self.z_logtype.append(self.z)
        self.yaw_logtype.append(self.yaw)
        self.vx_logtype.append(self.vx)
        self.vy_logtype.append(self.vy)
        self.vz_logtype.append(self.vz)
        self.roll_logtype.append(self.roll)
        self.pitch_logtype.append(self.pitch)
        self.f_logtype.append(self.last_input[0] / self.platform.mass)  # N → N/kg
        self.p_logtype.append(self.wx)
        self.q_logtype.append(self.wy)
        self.r_logtype.append(self.wz)

        # Reference state (x_ff): [px, py, pz, vx, vy, vz, f, phi, th, psi]
        xff = self.x_ff_last
        self.x_ref_logtype.append(xff[0])
        self.y_ref_logtype.append(xff[1])
        self.z_ref_logtype.append(xff[2])
        self.vx_ref_logtype.append(xff[3])
        self.vy_ref_logtype.append(xff[4])
        self.vz_ref_logtype.append(xff[5])
        self.f_ref_logtype.append(xff[6])
        self.roll_ref_logtype.append(xff[7])    # phi
        self.pitch_ref_logtype.append(xff[8])   # theta
        self.yaw_ref_logtype.append(xff[9])

        # Reference control (u_ff): [df, dphi, dth, dpsi]
        uff = self.u_ff_last
        self.u_ff_logtype.append(uff[0], uff[1], uff[2], uff[3])

        # Commanded inputs
        self.throttle_logtype.append(self.normalized_input[0])
        self.p_cmd_logtype.append(self.normalized_input[1])
        self.q_cmd_logtype.append(self.normalized_input[2])
        self.r_cmd_logtype.append(self.normalized_input[3])

    # ── Setpoint helpers ──────────────────────────────────────────────────────
    def _publish_position(self, x: float, y: float, z: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def _publish_rates(self, thrust: float, roll: float, pitch: float, yaw: float) -> None:
        msg = VehicleRatesSetpoint()
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -float(thrust)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.rates_setpoint_publisher.publish(msg)

