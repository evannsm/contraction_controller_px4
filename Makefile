# =============================================================================
# contraction_controller_px4 — Docker + ROS 2 workspace management
# =============================================================================
#
# HOW THE DOCKER IMAGE IS BUILT
# ──────────────────────────────
# The image (px4_contraction_jazzy) is built from docker/Dockerfile with
# the repo root as the build context.  It contains:
#
#   • ROS 2 Jazzy          — from osrf/ros:jazzy-desktop-full base image
#   • px4_msgs             — cloned from github.com/evannsm/px4_msgs
#                            (branch: v1.16_minimal_msgs) and pre-built into
#                            /opt/ws_px4_msgs/install/ so it is always
#                            available as an upstream ROS 2 overlay
#   • Python venv          — /opt/px4-venv with JAX, equinox, immrax, linrax
#                            (jax-0.9-support branches of both)
#
# HOW THE CONTAINER INTERACTS WITH THE HOST WORKSPACE
# ─────────────────────────────────────────────────────
# `make run` mounts the repo root → /workspace inside the container.
# This means:
#   • /workspace/src/    — all ROS 2 packages (controller + submodule deps)
#   • /workspace/build/  — colcon build artifacts  (persisted on host)
#   • /workspace/install/— colcon install tree      (persisted on host)
#   • /workspace/log/    — colcon logs              (persisted on host)
#
# The container uses --net host, so it sees all ROS 2 topics from the host
# PX4 sim and MicroXRCE bridge without any extra networking setup.
#
# Source chain inside the container (.bashrc / profile.d):
#   /opt/ros/jazzy/setup.bash
#   → /opt/ws_px4_msgs/install/setup.bash   (px4_msgs overlay)
#   → /opt/px4-venv/bin/activate            (Python venv)
#   → /workspace/install/setup.bash         (your built packages, if present)
#
# TYPICAL WORKFLOW
# ────────────────
#   make build          # build the image once (or after Dockerfile changes)
#   make run            # start the container
#   make build_ros      # build all packages in the workspace
#   make run_controller # run the contraction controller node
# =============================================================================

IMAGE_NAME     = px4_contraction_jazzy
CONTAINER_NAME = px4_contraction
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

# ── Run the contraction controller ───────────────────────────────────────────
PLATFORM        ?= sim
TRAJECTORY      ?= hover
HOVER_MODE      ?= 1
FLIGHT_PERIOD   ?=
CONTROLLER_DIR  ?=
LOG             ?=
LOG_FILE        ?=
NO_FEEDFORWARD  ?=

run_controller:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"ros2 run contraction_controller_px4 run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),) \
		   $(if $(FLIGHT_PERIOD),--flight-period $(FLIGHT_PERIOD),) \
		   $(if $(CONTROLLER_DIR),--controller-dir $(CONTROLLER_DIR),) \
		   $(if $(LOG),--log,) \
		   $(if $(LOG_FILE),--log-file $(LOG_FILE),) \
		   $(if $(filter 1,$(NO_FEEDFORWARD)),--no-feedforward,)"

.PHONY: build run stop kill attach build_ros clean_build_ros run_controller
