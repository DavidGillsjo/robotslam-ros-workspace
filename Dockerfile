FROM osrf/ros:kinetic-desktop-full

# Arguments
ARG user=ros
ARG uid=1000
ARG gid=1000

RUN apt-get update
#Build tools and other
RUN apt-get install sudo git python-wstool python-rosdep ninja-build -y
#Turtlebot packages
RUN apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps \
                    ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator \
                    ros-kinetic-kobuki-ftdi -y

RUN export uid="${uid}" gid="${gid}" && \
    groupadd -g "${gid}" "${user}" && \
    useradd -m -u "${uid}" -g "${user}" -s /bin/bash "${user}" && \
    passwd -d "${user}" && \
    usermod -aG sudo "${user}"

WORKDIR "/ros"

# Copy the current directory contents into the container at /app
ADD . "/ros"

# Update repositories
RUN wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_turtlebot/master/cartographer_turtlebot.rosinstall
RUN wstool update -t src --delete-changed-uris
RUN chown -R "${user}:${user}" "/ros"

USER "${user}"

# Install deb dependencies.
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r

# Build and install.
SHELL ["/bin/bash", "-c"]
RUN source "/opt/ros/kinetic/setup.bash" &&\
    catkin_make_isolated --install --use-ninja

# Sourcing this before .bashrc runs breaks ROS completions
RUN echo "\nsource /ros/install_isolated/setup.bash" >> "/home/${user}/.bashrc"

# Make SSH available
EXPOSE 22

# Mount the user's home directory
VOLUME "/home/${user}"
