SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../")

PORT_8211=8110 # NOTE： it can't be 8311 8011 8211 etc
PORT_49100=49103 # NOTE： it can't be 49100 etc
SSH_PORT=2222
FILE_1="/isaac-sim/extscache/omni.kit.livestream.webrtc-2.2.1+105.1.lx64.r.cp310/config/extension.toml"
FILE_2="/isaac-sim/extscache/omni.services.streamclient.webrtc-1.3.8/web/js/kit-player.js"
CONTAINER_NAME="pr2-challenge-student"

# docker network create --subnet=172.28.0.0/16 isaac
xhost +
# docker run --name pr2-challenge-student1 -it --gpus all -e "ACCEPT_EULA=Y" --rm --net isaac -h isaac1 --ip 172.28.0.12 \
docker run --name $CONTAINER_NAME -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e "PORT_8211=$PORT_8211" \
  -e "PORT_49100=$PORT_49100" \
  -e "FILE_1=$FILE_1" \
  -e "FILE_2=$FILE_2" \
  --rm \
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
  -v /var/run/utmp:/var/run/utmp:rw \
  -p $SSH_PORT:22 \
  -p $PORT_8211:8211 \
  -p $PORT_49100:$PORT_49100 \
  -v $PROJECT_DIR:/PR2:rw \
  pr2-challenge-ssh bash -c "source /isaac-sim/setup_python_env.sh && sed -i 's|49100|$PORT_49100|g' $FILE_1 && sed -i 's|49100|$PORT_49100|g' $FILE_2 && bash"
