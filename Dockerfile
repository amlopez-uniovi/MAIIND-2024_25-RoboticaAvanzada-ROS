# Establecemos la imagen base de ROS 2 Humble con el escritorio completo
FROM osrf/ros:humble-desktop

# Actualizamos los paquetes e instalamos dependencias necesarias
RUN apt update && apt install -y curl gnupg2 lsb-release \
    vim terminator wget iputils-ping

RUN echo "deb [signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Actualizamos e instalamos las actualizaciones necesarias
RUN apt update && apt upgrade -y

# Instalamos rviz2 para ROS 2 Humble
RUN apt install -y ros-humble-rviz2

# Establecemos la variable de entorno DISPLAY para permitir gráficos en X11
ENV DISPLAY=host.docker.internal:0.0

# Configuramos el entorno de ROS 2 para que se cargue automáticamente
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Creamos el espacio de trabajo de ROS 2
WORKDIR /root/maiind_ws
RUN mkdir -p src

# Establecemos el comando predeterminado para el contenedor
CMD ["/bin/bash"]
