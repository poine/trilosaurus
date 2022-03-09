


## Service

Not using robot_upstart in order to be able to start a non-ROS part

sudo ln -s /home/ubuntu/work/trilosaurus/trilosaurus_noros/system/robot.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable robot

systemctl status robot
systemctl start robot
systemctl stop robot
journalctl -u robot
