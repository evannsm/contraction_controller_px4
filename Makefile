# =============================================================================
# PX4 Quadrotor Controllers — Docker + ROS 2 workspace management
# =============================================================================
#
# HOW THE DOCKER IMAGE IS BUILT
# ──────────────────────────────
# The image is built from docker/Dockerfile with the repo root as the build
# context.  It contains:
#
#   • ROS 2 Jazzy          — from osrf/ros:jazzy-desktop-full base image
#   • px4_msgs             — pre-built into /opt/ws_px4_msgs/install/
#   • acados               — built from source at /opt/acados (v0.5.1)
#   • Python packages      — JAX, equinox, immrax, linrax, casadi, acados_template
#   • C++ deps             — Eigen3, OpenBLAS, LAPACK (for C++ controllers)
#
# HOW THE CONTAINER INTERACTS WITH THE HOST WORKSPACE
# ─────────────────────────────────────────────────────
# `make run` mounts the repo root → /workspace inside the container.
# The container uses --net host, so it sees all ROS 2 topics from the host
# PX4 sim and MicroXRCE bridge without any extra networking setup.
#
# TYPICAL WORKFLOW
# ────────────────
#   make build                   # build the image once
#   make run                     # start the container
#   make build_ros               # build all ROS 2 packages
#   make run_contraction ...     # run a controller
#
# NMPC C++ WORKFLOW
# ─────────────────
#   make generate_nmpc_solver    # generate acados C solver (once)
#   make build_ros               # then build all packages including nmpc_acados_px4_cpp
# =============================================================================

IMAGE_NAME     = px4_controllers_jazzy
CONTAINER_NAME = px4_controllers
WS_ROOT        := $(CURDIR)

# ── Docker image ──────────────────────────────────────────────────────────────
build:
	docker build -f docker/Dockerfile . -t $(IMAGE_NAME)

# ── Run container ─────────────────────────────────────────────────────────────
run:
	docker rm -f $(CONTAINER_NAME) 2>/dev/null || true
	docker run -itd --rm \
		--net host \
		-e ROS_DOMAIN_ID=31 \
		-v $(WS_ROOT):/workspace \
		--name $(CONTAINER_NAME) \
		$(IMAGE_NAME)

# ── Container lifecycle ───────────────────────────────────────────────────────
stop:
	docker stop $(CONTAINER_NAME)

kill:
	docker kill $(CONTAINER_NAME)

attach:
	docker exec -it $(CONTAINER_NAME) bash

# ── Build ROS 2 workspace ─────────────────────────────────────────────────────
# Optional: PACKAGES="pkg1 pkg2" to build only specific packages.
PACKAGES ?=

build_ros:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"cd /workspace && colcon build \
		   --symlink-install \
		   --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		   $(if $(PACKAGES),--packages-select $(PACKAGES),)"

# Wipe build/install/log then rebuild from scratch.
clean_build_ros:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"rm -rf /workspace/build /workspace/install /workspace/log"
	$(MAKE) build_ros PACKAGES="$(PACKAGES)"

# ── Generate NMPC acados solver ──────────────────────────────────────────────
# Must be run before building nmpc_acados_px4_cpp.
generate_nmpc_solver:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"cd /workspace && python3 src/nmpc_acados_px4/generate_solver.py"

# =============================================================================
# Controller run targets
# =============================================================================
# Shared variables — override on the command line, e.g.:
#   make run_newton_raphson PLATFORM=hw TRAJECTORY=helix DOUBLE_SPEED=1 LOG=1

PLATFORM        ?= sim
TRAJECTORY      ?= hover
HOVER_MODE      ?= 1
FLIGHT_PERIOD   ?=
DOUBLE_SPEED    ?=
SHORT           ?=
SPIN            ?=
FF              ?=
LOG             ?=
LOG_FILE        ?=

# Contraction-specific
CONTROLLER_DIR  ?=
NO_FEEDFORWARD  ?=

# ── Contraction controller (Python) ──────────────────────────────────────────
run_contraction:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run contraction_controller_px4 run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(CONTROLLER_DIR),--controller-dir $(CONTROLLER_DIR),) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),) \
		   $(if $(filter 1,$(NO_FEEDFORWARD)),--no-feedforward,)"

# ── Newton-Raphson (Python) ──────────────────────────────────────────────────
run_newton_raphson:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run newton_raphson_px4 run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(DOUBLE_SPEED),--double-speed,) \
		   $(if $(SHORT),--short,) \
		   $(if $(SPIN),--spin,) \
		   $(if $(FF),--ff,) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),)"

# ── Newton-Raphson (C++) ─────────────────────────────────────────────────────
run_newton_raphson_cpp:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run newton_raphson_px4_cpp run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(DOUBLE_SPEED),--double-speed,) \
		   $(if $(SHORT),--short,) \
		   $(if $(SPIN),--spin,) \
		   $(if $(FF),--ff,) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),)"

# ── NMPC Acados (Python) ─────────────────────────────────────────────────────
run_nmpc:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run nmpc_acados_px4 run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(DOUBLE_SPEED),--double-speed,) \
		   $(if $(SHORT),--short,) \
		   $(if $(SPIN),--spin,) \
		   $(if $(FF),--ff,) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),)"

# ── NMPC Acados (C++) ────────────────────────────────────────────────────────
run_nmpc_cpp:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run nmpc_acados_px4_cpp run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(DOUBLE_SPEED),--double-speed,) \
		   $(if $(SHORT),--short,) \
		   $(if $(SPIN),--spin,) \
		   $(if $(FF),--ff,) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),)"

# ── Feedforward figure-8 (Python) ────────────────────────────────────────────
run_ff_f8:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run ff_f8_px4 run_node \
		   --platform $(PLATFORM) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(DOUBLE_SPEED),--double-speed,) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),)"

# Backward compat alias
run_controller: run_contraction

.PHONY: build run stop kill attach build_ros clean_build_ros \
        generate_nmpc_solver \
        run_controller run_contraction \
        run_newton_raphson run_newton_raphson_cpp \
        run_nmpc run_nmpc_cpp run_ff_f8
