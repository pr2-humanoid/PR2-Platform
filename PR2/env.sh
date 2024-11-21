xhost +

echo "Current directory: $(pwd)"
echo "Bash source directory: $(dirname -- "${BASH_SOURCE[0]}")"

 
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$(realpath "$SCRIPT_DIR/")
FILE_1="/isaac-sim/extscache/omni.kit.livestream.webrtc-2.2.1+105.1.lx64.r.cp310/config/extension.toml"
FILE_2="/isaac-sim/extscache/omni.services.streamclient.webrtc-1.3.8/web/js/kit-player.js"

#--------Student 1 Configuration--------------
PORT_8211_STU1=8113 # NOTE： it can't be 8311 8011 8211 8111 etc
PORT_49100_STU1=49101 # NOTE： it can't be 49100 etc
SSH_PORT_STU1=2221

#--------Student 2 Configuration--------------
PORT_8211_STU2=8112 # NOTE： it can't be 8311 8011 8211 etc
PORT_49100_STU2=49102 # NOTE： it can't be 49100 etc
SSH_PORT_STU2=2222

 
export PROJECT_DIR
export PORT_8211_STU1 PORT_49100_STU1 SSH_PORT_STU1
export PORT_8211_STU2 PORT_49100_STU2 SSH_PORT_STU2
export FILE_1 FILE_2

bash