FROM ros:jazzy

ARG USERNAME=ur_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Remove any existing user with the same UID to avoid conflicts
RUN if id -u $USER_UID > /dev/null 2>&1; then userdel -r `id -un $USER_UID`; fi

# Create development user with sudo privileges 
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ------------------------------------------------------------
# # Install system, Python, and core ROS 2 dependencies 
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    # Basic tools
    build-essential cmake git tree gedit curl gnupg2 software-properties-common pkg-config lsb-release iputils-ping iproute2 \
    # Python tools
    python3-pip python3-dev python3-opencv python3-empy python3-defusedxml python3-numpy\
    # ROS tools
    ros-jazzy-vision-opencv ros-jazzy-rviz2 ros-jazzy-tf-transformations ros-jazzy-cv-bridge ros-jazzy-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

RUN printf "numpy<2" > /opt/pip-constraints.txt
ENV PIP_CONSTRAINT=/opt/pip-constraints.txt \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1


# ------------------------------------------------------------
# Install Python packages
# ------------------------------------------------------------
# With dependencies
RUN pip install --break-system-packages \
    scipy==1.16.1 \
    supervision==0.26.1 \
    spatialmath-python==1.1.14 \
    ultralytics==8.3.179 \
    matplotlib==3.10.5 \
    segmentation-models-pytorch==0.3.3

RUN python3 -m pip install --break-system-packages pyrealsense2 

# Add src/ to Python path for imports
ENV PYTHONPATH=/home/$USERNAME/ur_alignment_ws/src:$PYTHONPATH

# ------------------------------------------------------------
# Install C++ dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libv4l-dev \
    libdc1394-dev \
    libapriltag-dev \
    libboost-all-dev \
    libx11-dev \
    liblapack-dev \
    libjpeg-dev \
    libpng-dev \
    zlib1g-dev \
 && rm -rf /var/lib/apt/lists/*


 # ------------------------------------------------------------
# Install Universal Robots RTDE library
# ------------------------------------------------------------
RUN add-apt-repository -y ppa:sdurobotics/ur-rtde && \
    apt-get update && \
    apt-get install -y librtde librtde-dev && \
    rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# Install realsense2 
# ------------------------------------------------------------
RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg apt-transport-https; \
    mkdir -p /etc/apt/keyrings; \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
      | tee /etc/apt/keyrings/librealsense.pgp >/dev/null; \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/librealsense.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      librealsense2 librealsense2-utils librealsense2-dev \
      libusb-1.0-0-dev libudev-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
      && rm -rf /var/lib/apt/lists/*

      
# ------------------------------------------------------------
# Install visp from source (with realsense2 support) 
# ------------------------------------------------------------
RUN set -eux; \
    git clone --depth=1 https://github.com/lagadic/visp.git /tmp/visp; \
    cmake -S /tmp/visp -B /tmp/visp/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_DEMOS=OFF -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF \
      -DUSE_REALSENSE2=ON \
      -DCMAKE_PREFIX_PATH="/usr/local:/usr"; \
    cmake --build /tmp/visp/build -j"$(nproc)"; \
    cmake --install /tmp/visp/build; \
    ldconfig; \
    rm -rf /tmp/visp


# ------------------------------------------------------------
# Setup ROS 2 workspace
# ------------------------------------------------------------
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc && \
     echo '[ -f /home/'"$USERNAME"'/ur_alignment_ws/install/setup.bash ] && source /home/'"$USERNAME"'/ur_alignment_ws/install/setup.bash' >> /home/$USERNAME/.bashrc \
     && echo 'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH' >> /home/$USERNAME/.bashrc

# ------------------------------------------------------------
# Final container setup: configure shell, logs folder, and switch to non-root user
# ------------------------------------------------------------
ENV SHELL=/bin/bash
RUN mkdir -p /home/$USERNAME/.colcon/logs && chown -R $USERNAME:$USERNAME /home/$USERNAME
USER $USERNAME
CMD ["/bin/bash"]
