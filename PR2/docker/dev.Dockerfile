FROM nvcr.io/nvidia/isaac-sim:5.0.0 

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt-get install -y \
        ca-certificates gnupg lsb-release wget \
        libgflags2.2 \
        libsdl2-2.0-0 \
        libsdl2-ttf-2.0-0 \
    && wget https://github.com/RobotLocomotion/drake/releases/download/v1.23.0/drake-dev_1.23.0-1_amd64-jammy.deb \
    && DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt-get install -y ./drake-dev_1.23.0-1_amd64-jammy.deb \
    && rm drake-dev_1.23.0-1_amd64-jammy.deb \
    && rm -rf /var/lib/apt/lists/*
 

RUN cd / \
    && wget "https://github.com/pr2-humanoid/PR2-Platform/releases/download/v0.1.0/leju_controller-v4.zip" \
    && unzip leju_controller-v4.zip \
    && rm -rf leju_controller-v4.zip


RUN /isaac-sim/python.sh -m pip install --upgrade pip

WORKDIR /PR2/
ENTRYPOINT ["bash", "/PR2/docker/script/main-pr2.sh"]
