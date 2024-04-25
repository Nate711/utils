SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/robot.service /etc/systemd/system/robot.service
sudo systemctl enable robot