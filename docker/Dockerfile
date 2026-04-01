FROM osrf/ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# ── System deps ───────────────────────────────────────────────────────────────
# ros-base is ~1.8 GB smaller than desktop-full; we only need rclpy/rclcpp +
# colcon build tools, not rviz2/rqt/perception stacks.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    git \
    cmake \
    build-essential \
    libgmp-dev \
    libcdd-dev \
    libeigen3-dev \
    liblapack-dev \
    libopenblas-dev \
    wget \
    && apt-get remove -y python3-matplotlib 2>/dev/null || true \
    && rm -rf /var/lib/apt/lists/*

# ── ws_px4_msgs: pre-built inside the image, sourced as upstream overlay ──────
RUN mkdir -p /opt/ws_px4_msgs/src && \
    git clone --depth 1 --branch v1.16_minimal_msgs \
        https://github.com/evannsm/px4_msgs.git /opt/ws_px4_msgs/src/px4_msgs && \
    source /opt/ros/jazzy/setup.bash && \
    cd /opt/ws_px4_msgs && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /opt/ws_px4_msgs/build /opt/ws_px4_msgs/log /opt/ws_px4_msgs/src

# ── acados: build from source ────────────────────────────────────────────────
ENV ACADOS_ROOT=/opt/acados
RUN git clone https://github.com/acados/acados.git ${ACADOS_ROOT} && \
    cd ${ACADOS_ROOT} && \
    git checkout v0.5.1 && \
    git submodule update --recursive --init && \
    mkdir -p build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON \
          -DCMAKE_INSTALL_PREFIX=${ACADOS_ROOT} \
          .. && \
    make install -j$(nproc) && \
    rm -rf ${ACADOS_ROOT}/build

ENV LD_LIBRARY_PATH="${ACADOS_ROOT}/lib:${LD_LIBRARY_PATH}"
ENV ACADOS_SOURCE_DIR="${ACADOS_ROOT}"

# ── acados Python interface + tera_renderer ──────────────────────────────────
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        -e ${ACADOS_ROOT}/interfaces/acados_template && \
    mkdir -p ${ACADOS_ROOT}/bin && \
    wget -q https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
         -O ${ACADOS_ROOT}/bin/t_renderer && \
    chmod +x ${ACADOS_ROOT}/bin/t_renderer

# ── Python deps — installed to system Python ──────────────────────────────────
COPY docker/requirements.txt /tmp/requirements.txt

RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed -r /tmp/requirements.txt

# ── immrax (jax-0.9-support branch) ──────────────────────────────────────────
RUN git clone --depth 1 --branch jax-0.9-support \
        https://github.com/Akash-Harapanahalli/immrax.git /tmp/immrax && \
    sed -i 's|git@github.com:|https://github.com/|g' /tmp/immrax/.gitmodules 2>/dev/null || true && \
    git -C /tmp/immrax submodule sync --recursive 2>/dev/null || true && \
    git -C /tmp/immrax submodule update --init --recursive 2>/dev/null || true && \
    pip3 install --no-cache-dir --break-system-packages /tmp/immrax && \
    rm -rf /tmp/immrax

# ── linrax (jax-0.9-support branch) ──────────────────────────────────────────
RUN git clone --depth 1 --branch jax-0.9-support \
        https://github.com/gtfactslab/linrax.git /tmp/linrax && \
    pip3 install --no-cache-dir --break-system-packages /tmp/linrax && \
    rm -rf /tmp/linrax

# ── Shell setup: source chain ROS → px4_msgs → controller ws ─────────────────
# /etc/profile.d/ is sourced by login shells (bash -lc); .bashrc for interactive.
RUN echo "source /opt/ros/jazzy/setup.bash" > /etc/profile.d/ros_setup.sh && \
    echo "source /opt/ws_px4_msgs/install/setup.bash" >> /etc/profile.d/ros_setup.sh && \
    echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /etc/profile.d/ros_setup.sh && \
    echo "export ACADOS_SOURCE_DIR=${ACADOS_ROOT}" >> /etc/profile.d/ros_setup.sh && \
    echo "export LD_LIBRARY_PATH=${ACADOS_ROOT}/lib:\${LD_LIBRARY_PATH}" >> /etc/profile.d/ros_setup.sh && \
    echo "source /etc/profile.d/ros_setup.sh" >> /root/.bashrc

WORKDIR /workspace

CMD ["bash"]
