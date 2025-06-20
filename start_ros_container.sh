#!/bin/bash

# Stellen Sie sicher, dass das Skript im Projekt-Stammverzeichnis ausgeführt wird
# Das Skript muss von dort ausgeführt werden, wo ros_ws_prolab und Docker liegen
# Beispiel: /path/to/your/project/start_ros_container.sh

# Überprüfen, ob das ros-prolab Image existiert, andernfalls bauen
# (Optional, aber hilfreich)
if [[ "$(docker images -q ros-prolab 2> /dev/null)" == "" ]]; then
  echo "Docker image 'ros-prolab' not found. Building it now..."
  docker build -t ros-prolab ./Docker
  if [ $? -ne 0 ]; then
    echo "Failed to build Docker image. Exiting."
    exit 1
  fi
fi

echo "Starting ROS Prolab container..."
docker run -it --rm \
    --name ros_prolab_container \
    --volume "$(pwd)"/ros_ws_prolab:/app/ros_ws_prolab \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --gpus all \
    --privileged \
    --env="NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute" \
    --runtime=nvidia \
    ros-prolab \
    bash # <--- GANZ WICHTIG: KEINE LEERZEICHEN VOR ODER NACH 'bash' UND KEINE LEERZEICHEN NACH DEM '\' IN DER VORHERIGEN ZEILE!

echo "Container stopped."
