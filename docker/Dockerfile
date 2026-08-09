FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-venv \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN source /opt/ros/jazzy/setup.bash \
  && rosdep update \
  && apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  && rm -rf /var/lib/apt/lists/* \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

RUN python3 -m venv /opt/vif-venv \
  && /opt/vif-venv/bin/pip install --no-cache-dir -e 'evaluation[test]'

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "vif_bringup", "synthetic_demo.launch.py"]
