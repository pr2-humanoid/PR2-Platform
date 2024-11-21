source /isaac-sim/setup_python_env.sh

# use isaac-sim python.sh
python() {
    /isaac-sim/python.sh "$@"
}
export -f python

pip() {
    /isaac-sim/python.sh -m pip "$@"
}
export -f pip
 
echo "Using localhost port: $PORT_8211"
echo "Using WS port: $PORT_49100"
 

 
if [[ -n "$PORT_49100" && -n "$FILE_1" ]]; then
    sed -i "s|49100|$PORT_49100|g" "$FILE_1"
fi

if [[ -n "$PORT_49100" && -n "$FILE_2" ]]; then
    sed -i "s|49100|$PORT_49100|g" "$FILE_2"
fi

service ssh start

 
exec "$@" 

# cd /leju_controller/ && ./biped_sim_server_nonblocking &
cd /PR2/

pip install -e /PR2/

 
bash