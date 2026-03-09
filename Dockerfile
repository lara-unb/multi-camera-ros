# Escolha a imagem ROS oficial que preferir (noetic, melodic, etc.)
FROM ros:noetic

# Evita perguntas no apt
ENV DEBIAN_FRONTEND=noninteractive

# Atualiza sistema + instala ferramentas essenciais
RUN apt-get update && \
    apt-get install -y \
      build-essential \
      vim \
      git \
      wget \
      cmake \
      pkg-config \
      psmisc \
      python3-catkin-tools \
      python3-pip \
      libeigen3-dev 

# Instalação de ROS, pacotes e afins

RUN apt-get update && \
    apt-get install -y \
    ros-noetic-tf \
    ros-noetic-pcl-ros \
    ros-noetic-rgbd-launch \
    ros-noetic-depthimage-to-laserscan \
    ros-noetic-image-transport \
    ros-noetic-image-common \
    ros-noetic-nodelet \
    ros-noetic-diagnostic-updater \
    ros-noetic-image-view \
    ros-noetic-aruco-ros \
    ros-noetic-media-export \
    ros-noetic-tf2-geometry-msgs \
    rviz

# Instala libfreenect e dependências
RUN apt-get update && \
    apt-get install -y \
      libusb-1.0-0-dev \
      libfreenect-dev \
      libfreenect-demos 

# pip packages
RUN pip3 install --upgrade pip

RUN apt-get install usbutils

# Cria workspace ROS
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws/src

# Instala o libfreenect stack (bridge do libfreenect com o ros)
RUN git clone -b melodic-devel https://github.com/ros-drivers/freenect_stack.git

WORKDIR /root/catkin_ws

# Inicializa workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace src"

# Build do workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Configura o entrypoint para ROS
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
