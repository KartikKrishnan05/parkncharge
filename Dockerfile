FROM osrf/ros:humble-desktop-full

# Zero interaction while installing or upgrading the system via apt.
ENV DEBIAN_FRONTEND=noninteractive

# Set up proxy
ARG proxy
ENV http_proxy ${proxy}
ENV https_proxy ${proxy}
ENV no_proxy "localhost,127.0.0.1,::1,.iabg.de"

RUN if [ -n "$proxy" ]; then /bin/echo -e '\
    Acquire::http::Proxy "http://proxy.iabg.de:8080";\n\
    Acquire::https::Proxy "http://proxy.iabg.de:8080";' > /etc/apt/apt.conf; \
    fi;


# Update the system
RUN apt-get update && apt-get -y upgrade
RUN apt-get -y install apt-utils pciutils psmisc \
    git build-essential python3-pip \
    wget curl vim nano catimg tmux \
    iputils-ping net-tools iproute2
RUN python3 -m pip install --upgrade pip setuptools wheel

# Set locale. The default locale is not set in container
RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    dpkg-reconfigure --frontend=noninteractive locales

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Getting Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O \
    /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Installing ros and gazebo packages
RUN apt update && apt install -y ros-${ROS_DISTRO}-ros-gz

# Install required packages
RUN apt-get -y install gcc-arm-none-eabi
COPY requirements-px4dev.txt requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt

# colcon build fix
RUN python3 -m pip install setuptools==58.2.0

# Add a user
ARG UID=1000
RUN groupadd -g 1018 -o aiolos-indoors
RUN useradd -m -r -u ${UID} -g 1018 -o -s /bin/bash \
    -G adm,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,aiolos-indoors \
    dockeruser && \
    echo "dockeruser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    passwd -d dockeruser

# Set up bash environment
RUN echo 'PS1="(px4-docker) $PS1"' >> ~/.bashrc && \
    echo 'PS1="(px4-docker) $PS1"' >> /home/dockeruser/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/dockeruser/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/dockeruser/.bashrc && \
    echo "source /indoors/ros2_ws/install/setup.bash" >> /home/dockeruser/.bashrc && \
    echo "set -g mouse on" > /home/dockeruser/.tmux.conf
