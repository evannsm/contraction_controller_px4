IMAGE_NAME     = px4_contraction_jazzy
CONTAINER_NAME = px4_contraction
# Package root (contraction_controller_px4/) is one level up from docker/
PKG_ROOT       := $(abspath ..)
# Workspace root (ws_px4_work/) is three levels up from docker/
WS_ROOT        := $(abspath ../../..)
PX4_ROOT       ?= /home/$(shell id -un)/PX4-Autopilot
PYTHON         ?= /opt/px4-venv/bin/python

# ── Docker image ─────────────────────────────────────────────────────────────
# Build context is the package root so COPY docker/requirements.txt resolves.
build:
	docker build -f docker/Dockerfile $(PKG_ROOT) -t $(IMAGE_NAME)

# ── Run (headless) ───────────────────────────────────────────────────────────
run:
	docker run -itd --rm \
		--ipc host --net host --pid host \
		-v $(WS_ROOT):/workspace/ws_px4_work \
		-v $(PX4_ROOT):/workspace/PX4-Autopilot \
		--name $(CONTAINER_NAME) \
		$(IMAGE_NAME)

# ── Run with GUI (Gazebo) via X11 ────────────────────────────────────────────
run_gui:
	xhost +local:docker
	docker run -itd --rm \
		--ipc host --net host --pid host \
		-e DISPLAY=$(DISPLAY) \
		-e QT_X11_NO_MITSHM=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(WS_ROOT):/workspace/ws_px4_work \
		-v $(PX4_ROOT):/workspace/PX4-Autopilot \
		--name $(CONTAINER_NAME) \
		$(IMAGE_NAME)

# ── Container lifecycle ──────────────────────────────────────────────────────
start:
	docker start $(CONTAINER_NAME)

stop:
	docker stop $(CONTAINER_NAME)

kill:
	docker kill $(CONTAINER_NAME)

attach:
	docker exec -it $(CONTAINER_NAME) bash

# ── ROS 2 workspace ──────────────────────────────────────────────────────────
build_ros:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"cd /workspace/ws_px4_work && colcon build --symlink-install"

# ── PX4 Gazebo simulation (runs inside container) ────────────────────────────
sim:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"cd /workspace/PX4-Autopilot && make px4_sitl gz_x500"

# ── MicroXRCE-DDS bridge ─────────────────────────────────────────────────────
bridge:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"MicroXRCEAgent udp4 -p 8888"

# ── Contraction controller ───────────────────────────────────────────────────
PLATFORM   ?= sim
TRAJECTORY ?= hover
HOVER_MODE ?= 1

run_controller:
	docker exec -it $(CONTAINER_NAME) bash -lc \
		"source /workspace/ws_px4_work/install/setup.bash && \
		 ros2 run contraction_controller_px4 run_node \
		   --platform $(PLATFORM) --trajectory $(TRAJECTORY) \
		   $(if $(filter hover,$(TRAJECTORY)),--hover-mode $(HOVER_MODE),)"

# ── Full sim + bridge in parallel (two tmux panes, or use & for background) ──
launch_sim_and_bridge:
	docker exec -d $(CONTAINER_NAME) bash -lc \
		"cd /workspace/PX4-Autopilot && make px4_sitl gz_x500"
	sleep 5
	docker exec -d $(CONTAINER_NAME) bash -lc \
		"MicroXRCEAgent udp4 -p 8888"

.PHONY: build run run_gui start stop kill attach build_ros sim bridge run_controller launch_sim_and_bridge
