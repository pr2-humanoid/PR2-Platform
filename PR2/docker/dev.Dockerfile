FROM nvcr.io/nvidia/isaac-sim:2023.1.0

 
RUN apt-get update \
    && apt-get install -y \
        ca-certificates gnupg lsb-release wget \
        openssh-server \
        vim \
    && wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - > /etc/apt/trusted.gpg.d/drake.gpg \
    && echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" > /etc/apt/sources.list.d/drake.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt-get install -y \
        libgflags2.2 \
        libsdl2-2.0-0 \
        libsdl2-ttf-2.0-0 \
        drake-dev=1.23* \
    && rm -rf /var/lib/apt/lists/*

 
RUN mkdir /var/run/sshd 
RUN echo 'root:password' | chpasswd

 
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
    && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config  
   
 
RUN cd / \
    && wget "https://github.com/pr2-humanoid/PR2-Platform/releases/download/v0.1.0/leju_controller-v4.zip" \
    && unzip leju_controller-v4.zip \
    && rm -rf leju_controller-v4.zip

 
RUN /isaac-sim/python.sh -m pip install --upgrade pip

 
WORKDIR /PR2/

 
ENTRYPOINT ["/bin/bash", "-c", "service ssh start && bash /PR2/docker/script/main-pr2.sh"]
