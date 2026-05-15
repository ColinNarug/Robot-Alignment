FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

ARG USERNAME=ur_user
ARG USER_UID=1000
ARG USER_GID=1000

RUN if id -u ${USER_UID} >/dev/null 2>&1; then userdel -r "$(id -un ${USER_UID})" || true; fi \
 && if getent group ${USER_GID} >/dev/null 2>&1; then groupdel "$(getent group ${USER_GID} | cut -d: -f1)" || true; fi \
 && groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    libgl1 \
    libglib2.0-0 \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

RUN printf "numpy<2\n" > /opt/pip-constraints.txt

# Install CPU-only PyTorch first so pip does not pull multi-GB CUDA wheels.
RUN python3 -m pip install --break-system-packages --no-cache-dir \
    --extra-index-url https://download.pytorch.org/whl/cpu \
    torch==2.5.1+cpu \
    torchvision==0.20.1+cpu

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    --constraint /opt/pip-constraints.txt \
    scipy==1.16.1 \
    spatialmath-python==1.1.14 \
    supervision==0.26.1 \
    ultralytics==8.3.179 \
    segmentation-models-pytorch==0.3.3

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ra_ws