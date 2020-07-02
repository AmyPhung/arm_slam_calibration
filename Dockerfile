ARG ROS_DISTRO=melodic
ARG PROJECT=min_variance_calibration

FROM ros:$ROS_DISTRO

ARG DOCKER_USER=robot
ARG LIBSBP_V=2.3.10

MAINTAINER Amy Phung aphung@olin.edu

RUN bash -c \
    'useradd -lmG video $DOCKER_USER \
    && mkdir -p /home/$DOCKER_USER/catkin_ws/src/$PROJECT'

COPY . /home/$DOCKER_USER/catkin_ws/src/$PROJECT/

RUN bash -c \
    'apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y wget \
    && apt-get install -y sudo \
    && cd ~ \
    && git clone https://github.com/davisking/dlib.git \
    && cd /home/$DOCKER_USER/catkin_ws \
    && rosdep update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep install -iry --from-paths src \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd /home/$DOCKER_USER/catkin_ws/ \
    && catkin_make \
    && source /home/$DOCKER_USER/catkin_ws/devel/setup.bash \
    && echo "source ~/catkin_ws/devel/setup.bash" >> /home/$DOCKER_USER/.bashrc \
    && chown -R $DOCKER_USER /home/$DOCKER_USER'

WORKDIR /home/$DOCKER_USER/catkin_ws
USER $DOCKER_USER

WORKDIR /home/$DOCKER_USER
