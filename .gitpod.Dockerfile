FROM gitpod/workspace-full

USER root

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' --allow-unauthenticated\ 
    && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && sudo apt update

RUN sudo apt install ros-melodic-desktop-full

RUN sudo rosdep init \
    && rosdep update

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

RUN sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

USER root
