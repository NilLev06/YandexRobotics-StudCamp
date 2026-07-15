# https://hub.docker.com/layers/library/ros/jazzy-ros-base
FROM ros@sha256:c5705f613a6427d07be6f3d0aba40068e16c3dea05604e1c95c63eac79fea658

ENV ROS_VERSION=2
ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV CMAKE_BUILD_TYPE=Release
ENV DEBIAN_FRONTEND=noninteractive

# --no-install-recommends
RUN echo '\
APT::Install-Recommends "0";\n\
APT::Install-Suggests "0";\n\
' > /etc/apt/apt.conf.d/01norecommend

# Install cyclonedds RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN apt update && \
    apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Install foxglove bridge v0.8.5
RUN . $ROS_ROOT/setup.sh \
    && mkdir /tmp/foxglove-build \
    && cd /tmp/foxglove-build \
    && mkdir src \
    && echo "\
    - git:\n\
        local-name: foxglove-sdk/foxglove_bridge\n\
        uri: https://github.com/ros2-gbp/foxglove_bridge-release.git\n\
        version: release/$ROS_DISTRO/foxglove_bridge/0.8.5-1\n\
    " | vcs import src \
    && rosdep update \
    && apt update \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/* \
    && rm -rf /var/lib/apt/lists/*

# Install Nav2 dependencies
RUN apt update && \
    apt install -y \
    ros-$ROS_DISTRO-nav2-core \
    ros-$ROS_DISTRO-nav2-controller \
    ros-$ROS_DISTRO-nav2-bt-navigator \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-nav2-behaviors \
    ros-$ROS_DISTRO-nav2-planner \
    ros-$ROS_DISTRO-nav2-navfn-planner \
    ros-$ROS_DISTRO-nav2-regulated-pure-pursuit-controller \
    ros-$ROS_DISTRO-nav2-loopback-sim \
    ros-$ROS_DISTRO-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

# Install slam toolbox with patch
WORKDIR /tmp/slam-toolbox-build
ADD docker/slam-toolbox.patch .
RUN . $ROS_ROOT/setup.sh \
    && mkdir src \
    && echo "\
    - git:\n\
        local-name: slam_toolbox\n\
        uri: https://github.com/SteveMacenski/slam_toolbox-release.git\n\
        version: release/jazzy/slam_toolbox/2.8.3-1\n\
    " | vcs import src \
    && (cd src/slam_toolbox && git apply ../../slam-toolbox.patch) \
    && rosdep update \
    && apt update \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/* \
    && rm -rf /var/lib/apt/lists/*

# Patch nav2 loopback simulator
WORKDIR /tmp/nav2-loopback-sim-patch
ADD docker/nav2-loopback-sim.patch .
RUN patch -p1 -i nav2-loopback-sim.patch \
    $(find /opt/ros/$ROS_DISTRO -name loopback_simulator.py)

# Install LD19 lidar driver with patch
WORKDIR /tmp/ld19-lidar-build
ADD docker/ld19-lidar.patch .
RUN . $ROS_ROOT/setup.sh \
    && mkdir src \
    && git clone https://github.com/richardw347/ld19_lidar src/ld19_lidar \
    && (cd src/ld19_lidar && git apply ../../ld19-lidar.patch) \
    && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/*

# Build libcamera for raspberry
WORKDIR /tmp/libcamera-rpi-build
RUN . $ROS_ROOT/setup.sh \
    && git clone --depth 1 --branch "v0.6.0+rpt20251202" https://github.com/raspberrypi/libcamera \
    && cd libcamera \
    && apt update \
    && apt install -y \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff-dev \
    pybind11-dev \
    meson \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    && meson setup build --buildtype=release -Dgstreamer=disabled -Dpycamera=enabled \
    && ninja -C build install \
    && mv /usr/local/lib/python3/dist-packages/libcamera /usr/local/lib/python3.12/dist-packages \
    && mv /usr/local/include/libcamera/libcamera /usr/include \
    && mv /usr/local/lib/aarch64-linux-gnu/pkgconfig/* /usr/lib/pkgconfig \
    && rmdir /usr/local/lib/aarch64-linux-gnu/pkgconfig \
    && rm -rf /usr/local/include \
    && rm -rf /tmp/* \
    && rm -rf /var/lib/apt/lists/*
    # && mv /usr/local/lib/aarch64-linux-gnu/* /usr/lib \

# Build camera-ros
WORKDIR /tmp/camera-ros-build
RUN . $ROS_ROOT/setup.sh \
    && mkdir src \
    && git clone https://github.com/christianrauch/camera_ros.git src/camera_ros \
    && rosdep update \
    && apt update \
    && rosdep install --from-paths src --ignore-src --skip-keys=libcamera -t build -t exec -y \
    && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_CXX_FLAGS="-I/usr/include" \
    && rm -rf /tmp/*

# Install extras
RUN apt update && \
    apt install -y \
    tmux \
    tmuxp \
    nano \
    wget \
    python3-pip \
    python3-venv \
    python3-serial \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-rtabmap-odom \
    && rm -rf /var/lib/apt/lists/*

# Setup venv, picamera2, yolo, yandex speechkit
WORKDIR /tmp/venv-setup
ADD docker/picamera2.patch /tmp/venv-setup/picamera2.patch
RUN apt update \
    && apt install -y linux-libc-dev libcap-dev portaudio19-dev python3-pyaudio \
    && python3 -m venv /root/venv --system-site-packages \
    && . /root/venv/bin/activate \
    && pip install picamera2 \
    && patch -p2 -i /tmp/venv-setup/picamera2.patch -d /root/venv/lib/python3.12/site-packages/picamera2 \
    && pip install --extra-index-url https://download.pytorch.org/whl/cpu torch torchvision "numpy<2.0.0" ncnn ultralytics-opencv-headless \
    && mkdir /root/weights \
    && python3 -c 'from ultralytics import YOLO; YOLO("/root/weights/yolo11n.pt").export(format="ncnn"); YOLO("/root/weights/yolo11n_ncnn_model", task="detect")' \
    && pip install grpcio-tools \
    && git clone https://github.com/yandex-cloud/cloudapi \
    && cd cloudapi \
    && mkdir output \
    && python3 -m grpc_tools.protoc -I . -I third_party/googleapis \
        --python_out=output \
        --grpc_python_out=output \
        google/api/http.proto \
        google/api/annotations.proto \
        yandex/cloud/api/operation.proto \
        google/rpc/status.proto \
        yandex/cloud/operation/operation.proto \
        yandex/cloud/validation.proto \
        yandex/cloud/ai/stt/v3/stt_service.proto \
        yandex/cloud/ai/stt/v3/stt.proto \
        yandex/cloud/ai/tts/v3/tts_service.proto \
        yandex/cloud/ai/tts/v3/tts.proto \
    && cp -r output/* /root/venv/lib/python3.12/site-packages/ \
    && rm -rf /tmp/* \
    && rm -rf /var/lib/apt/lists/*

# Setup Jupyter Lab
RUN . /root/venv/bin/activate \
    && pip install jupyterlab \
    && mkdir -p /root/venv/share/jupyter/lab/settings \
    && echo '{"@jupyterlab/apputils-extension:themes": {"theme": "JupyterLab Dark"}}' \
    > /root/venv/share/jupyter/lab/settings/overrides.json \
    && mkdir -p /root/.jupyter \
    && echo '\
c = get_config()\n\
c.ServerApp.allow_root = True\n\
c.ServerApp.ip = "0.0.0.0"\n\
c.ServerApp.port = 8080\n\
c.ServerApp.open_browser = False\n\
c.ServerApp.token = ""\n\
c.ServerApp.password = ""\n\
    ' > /root/.jupyter/jupyter_lab_config.py

# Fix setuptools version
# https://github.com/ros2/ros2/issues/1702
# https://github.com/ros2/ros2/issues/1094
RUN . /root/venv/bin/activate && pip install "setuptools<80.0.0"

# Setup mediamtx
# Mediamtx bre-built custom binary is downloaded from S3
# Built from tag v1.18.1 with docker/mediamtx.patch applied
# https://github.com/bluenviron/mediamtx/issues/5744
RUN wget -O /bin/mediamtx https://storage.yandexcloud.net/the-lab-storage/rbm/mediamtx-1.18.1-arm64-patched \
    && chmod +x /bin/mediamtx

# Setup .bashrc
RUN echo '\
export LD_LIBRARY_PATH=$ROS_ROOT/lib/$(gcc -dumpmachine):/usr/local/lib/$(gcc -dumpmachine)\n\
source $ROS_ROOT/setup.bash\n\
LOCAL_SETUP="/src/install/setup.bash";\n\
if [ -f "$LOCAL_SETUP" ]; then source $LOCAL_SETUP; fi\n\
source /root/venv/bin/activate\n\
' >> /root/.bashrc

# Entrypoint
WORKDIR /src
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["trap : TERM INT; sleep infinity & wait"]
