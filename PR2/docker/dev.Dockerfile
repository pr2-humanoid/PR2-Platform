FROM nvcr.io/nvidia/isaac-sim:2023.1.0


RUN apt-get update \
    && apt-get install -y \
        ca-certificates gnupg lsb-release wget \
    && wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - > /etc/apt/trusted.gpg.d/drake.gpg \
    && echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" > /etc/apt/sources.list.d/drake.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt-get install -y \
        libgflags2.2 \
        libsdl2-2.0-0 \
        libsdl2-ttf-2.0-0 \
        drake-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd / \
    && wget "https://github.com/pr2-humanoid/PR2-Platform/releases/download/beta/leju_controller-v4.zip" \
    && unzip leju_controller-v4.zip \
    && rm -rf leju_controller-v4.zip


RUN /isaac-sim/python.sh -m pip install --upgrade pip

WORKDIR /PR2/
ENTRYPOINT ["bash", "/PR2/docker/scripts/main-dev.sh"]
