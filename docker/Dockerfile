# NOTE: PX4 publishes official dev containers at px4io/px4-dev-* (see
# https://docs.px4.io/main/en/test_and_ci/docker).  Those images are Ubuntu
# 22.04 (Jammy) CI environments useful for building PX4 from scratch.
# Because this project requires ROS 2 Jazzy (Ubuntu 24.04) and Gazebo
# Harmonic, we base on osrf/ros:jazzy-desktop-full instead and mount the
# already-built PX4-Autopilot directory from the host.  If you ever need to
# rebuild PX4 inside the container, the PX4 system-dependencies script is:
#   bash Tools/setup/ubuntu.sh --no-nuttx
# run from the mounted /workspace/PX4-Autopilot directory.

FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV VIRTUAL_ENV=/opt/px4-venv
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    git \
    make \
    cmake \
    ninja-build \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    gcc \
    g++ \
    # X11 / Gazebo GUI support
    libgl1-mesa-glx \
    libglu1-mesa \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

# ── MicroXRCE-DDS-Agent (PX4-recommended version) ───────────────────────────
# PX4 v1.14+ uses eProsima Micro-XRCE-DDS-Agent; build from source as
# recommended in https://docs.px4.io/main/en/middleware/uxrce_dds.html
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/MicroXRCEAgent \
    && cmake \
         -S /tmp/MicroXRCEAgent \
         -B /tmp/MicroXRCEAgent/build \
         -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/MicroXRCEAgent/build --target install -j$(nproc) \
    && ldconfig \
    && rm -rf /tmp/MicroXRCEAgent

# ── Python virtual environment (system-site-packages for ROS 2 bindings) ─────
COPY docker/requirements.txt /tmp/requirements.txt

RUN python3 -m venv "${VIRTUAL_ENV}" --system-site-packages && \
    "${VIRTUAL_ENV}/bin/pip" install --no-cache-dir --upgrade pip setuptools wheel && \
    "${VIRTUAL_ENV}/bin/pip" install --no-cache-dir -r /tmp/requirements.txt

# ── immrax (jax-0.9-support branch) ─────────────────────────────────────────
RUN git clone --depth 1 --branch jax-0.9-support \
        https://github.com/Akash-Harapanahalli/immrax.git /tmp/immrax && \
    cd /tmp/immrax && \
    sed -i 's|git@github.com:|https://github.com/|g' .gitmodules 2>/dev/null || true && \
    git submodule sync --recursive 2>/dev/null || true && \
    git submodule update --init --recursive 2>/dev/null || true && \
    "${VIRTUAL_ENV}/bin/pip" install --no-cache-dir /tmp/immrax && \
    rm -rf /tmp/immrax

# ── linrax (jax-0.9-support branch) ─────────────────────────────────────────
RUN git clone --depth 1 --branch jax-0.9-support \
        https://github.com/gtfactslab/linrax.git /tmp/linrax && \
    cd /tmp/linrax && \
    sed -i 's|git@github.com:|https://github.com/|g' .gitmodules 2>/dev/null || true && \
    git submodule sync --recursive 2>/dev/null || true && \
    git submodule update --init --recursive 2>/dev/null || true && \
    "${VIRTUAL_ENV}/bin/pip" install --no-cache-dir /tmp/linrax && \
    rm -rf /tmp/linrax

# ── Shell setup ──────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${VIRTUAL_ENV}/bin/activate" >> /root/.bashrc && \
    echo 'if [ -f /workspace/ws_px4_work/install/setup.bash ]; then source /workspace/ws_px4_work/install/setup.bash; fi' >> /root/.bashrc

WORKDIR /workspace

CMD ["/bin/bash"]
