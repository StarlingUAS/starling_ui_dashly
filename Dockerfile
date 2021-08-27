FROM ros:foxy-ros-base-focal

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        python3-pip \
    && rm -rf /var/lib/apt/lists/

RUN pip3 install dash dash-bootstrap-components gunicorn pandas

RUN mkdir -p ros_ws/src

COPY starling_ui_dashly ros_ws/src/

COPY ros_entrypoint.sh /ros_entrypoint.sh

# Add custom ROS DDS configuration (force UDP always)
COPY fastrtps_profiles.xml /ros_ws/
ENV FASTRTPS_DEFAULT_PROFILES_FILE /ros_ws/fastrtps_profiles.xml

WORKDIR ros_ws

RUN . /opt/ros/foxy/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --packages-select starling_ui_dashly --cmake-force-configure \
    && rm -r build

EXPOSE 3000

cmd ["ros2", "launch", "starling_ui_dashly", "dashboard_gunicorn.launch.xml"]