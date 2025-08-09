FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

ARG user_name=rosuser

# Create user and group with home directory
RUN useradd -ms /bin/bash $user_name

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    libompl-dev \
    libsdl1.2-dev \
    libsdl-image1.2-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Force software rendering (CPU)
ENV LIBGL_ALWAYS_SOFTWARE=1


# Create workspace src directory and set ownership
RUN mkdir -p /home/${user_name}/opt_sfc_ws/src && chown -R ${user_name}:${user_name} /home/${user_name}/opt_sfc_ws

WORKDIR /home/${user_name}/opt_sfc_ws

RUN git clone https://github.com/KumarRobotics/kr_param_map.git src/kr_param_map || true
RUN git clone https://github.com/yuwei-wu/GCOPTER.git src/GCOPTER || true


# Source ROS environment in user's bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${user_name}/.bashrc

# Switch to non-root user
USER ${user_name}
# Setup entrypoint
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

