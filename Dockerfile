FROM osrf/ros:kinetic-desktop-full

# Arguments
ARG home
ARG user=ros
ARG uid=1000
ARG gid=1000

RUN apt-get update
RUN apt-get install sudo git python-wstool python-rosdep ninja-build -y

RUN export uid="${uid}" gid="${gid}" && \
    groupadd -g "${gid}" "${user}" && \
    useradd -m -u "${uid}" -g "${user}" -s /bin/bash "${user}" && \
    passwd -d "${user}" && \
    usermod -aG sudo "${user}"

WORKDIR "/home/${user}"

# Copy the current directory contents into the container at /app
ADD . "/home/${user}"
RUN chown -R "${user}:${user}" "/home/${user}"

# Update repositories
RUN wstool update -t src --delete-changed-uris

USER "${user}"

# Install deb dependencies.
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r

# Build and install.
SHELL ["/bin/bash", "-c"]
RUN source "/opt/ros/kinetic/setup.bash" &&\
    catkin_make_isolated --install --use-ninja

# Sourcing this before .bashrc runs breaks ROS completions
RUN echo "\nsource /home/ros/install_isolated/setup.bash" >> "/home/${user}/.bashrc"

# Make SSH available
EXPOSE 22

# Mount the user's home directory
VOLUME "${home}"
