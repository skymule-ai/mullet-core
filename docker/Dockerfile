# mullet_core for the drone: the mullet-ros2-base image (ROS 2 Humble, px4_msgs, Micro
# XRCE-DDS Agent) plus mullet_interfaces and mullet_core, built and unit-tested at image build
# time, so a red test means no image. Build through compose.yaml, which supplies the
# mullet_interfaces context from the sibling checkout.
ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-ros2-base:v0.4.0
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /opt/mullet_ws

COPY --from=mullet_interfaces . src/mullet-interfaces
RUN source /usr/local/share/mullet-ros/setup.bash \
 && colcon build --packages-select mullet_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf build log

COPY . src/mullet-core
RUN source /usr/local/share/mullet-ros/setup.bash && source install/setup.bash \
 && colcon build --packages-select mullet_core --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && colcon test --packages-select mullet_core \
 && colcon test-result --verbose \
 && rm -rf build log

CMD ["bash", "-c", "source /opt/mullet_ws/install/setup.bash && exec ros2 launch mullet_core mullet_core.launch.py"]
