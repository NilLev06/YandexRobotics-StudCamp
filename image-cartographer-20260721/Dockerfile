FROM registry.robotics-lab.ru/robomarvel:v6

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-cartographer-ros \
      ros-jazzy-nav2-collision-monitor \
 && rm -rf /var/lib/apt/lists/* \
 && ldconfig
