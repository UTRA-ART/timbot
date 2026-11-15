# Headless Timbot simulation: Gazebo (server only) + full autonomy stack +
# foxglove_bridge. No GPU or display needed; view it with Foxglove.
# Runs on linux/amd64 and linux/arm64 (Apple Silicon). See docker/README.md.
#
# Targets (build from the repo root, e.g. docker build -f docker/Dockerfile --target sim .):
#   sim  (default) dependencies + a built copy of the source. What CI publishes.
#   dev  dependencies only; the source is mounted in and built at startup
#        (compose.yaml's `dev` service), so code changes don't need an image rebuild.

# ---- package.xml files only, so dependency layers are only rebuilt when they change.
FROM docker.io/library/ros:humble-ros-base-jammy AS manifests
COPY . /src
RUN mkdir /manifests && cd /src \
    && find . -name package.xml -not -path "*/build/*" -not -path "*/install/*" \
       -exec cp --parents {} /manifests \;

# ---- Everything the workspace needs to build and run, but not the workspace itself.
FROM docker.io/library/ros:humble-ros-base-jammy AS deps
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1

# ROS / Gazebo packages the launch files use that rosdep doesn't pick up,
# plus Xvfb + Mesa so Gazebo's sensors can render offscreen on the CPU.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-robot-localization \
        ros-humble-cartographer-ros \
        ros-humble-xacro \
        ros-humble-foxglove-bridge \
        python3-pip python3-scipy \
        xvfb libgl1-mesa-dri \
        git \
    && rm -rf /var/lib/apt/lists/*

# utm: GPS conversion in load_waypoints. torch is left out: the classical lane
# detector (the sim default) doesn't need it, and it would triple the image size.
RUN pip3 install --no-cache-dir utm

# The one Gazebo model track.world loads.
RUN git clone --depth 1 --filter=blob:none --sparse https://github.com/osrf/gazebo_models.git /tmp/gm \
    && git -C /tmp/gm sparse-checkout set construction_barrel \
    && mkdir -p /root/.gazebo/models \
    && mv /tmp/gm/construction_barrel /root/.gazebo/models/ \
    && rm -rf /tmp/gm

COPY --from=manifests /manifests /ws/src/timbot
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths /ws/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* /ws/src

WORKDIR /ws
COPY docker/start_sim.sh /start_sim.sh
EXPOSE 8765
ENTRYPOINT ["/start_sim.sh"]

# ---- Development: mount the repo at /ws/src/timbot; start_sim.sh builds it first.
FROM deps AS dev
ENV TIMBOT_BUILD=1

# ---- Self-contained image with the workspace built in.
FROM deps AS sim
COPY . /ws/src/timbot
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-skip zed_open_source \
    && rm -rf build log
