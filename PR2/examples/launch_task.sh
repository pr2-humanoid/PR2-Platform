# kill potential launched controller server
pkill task
sleep 1
cd /leju_controller;
./task$1_controller &
# wait for server
sleep 1
cd /PR2
python examples/task_launcher.py $1
