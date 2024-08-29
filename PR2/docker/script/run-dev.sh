SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../")

xhost +
docker run --name PR2-dev -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  --ulimit rtprio=99 \
  -e "PRIVACY_CONSENT=Y" \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e DISPLAY \
  -v ~/docker/isaac-sim_2023.1.0/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim_2023.1.0/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim_2023.1.0/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim_2023.1.0/documents:/root/Documents:rw \
  -v $PROJECT_DIR:/PR2:rw \
  PR2-dev bash
