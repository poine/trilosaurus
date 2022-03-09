## Calibration

./scripts/calibrate_extr_node.py _camera:=camera1

rostopic pub /vision/calibrate_extr/save_to_disk std_msgs/String "/tmp/extcalib.yaml"

./scripts/bird_eye_node.py  _camera:=camera1 _robot_name:=trilopi _ref_frame:=base_link_footprint

## Service

rosrun robot_upstart install --job robot_ros --user ubuntu trilosaurus_bringup/launch/robot.launch  --logdir /home/ubuntu/.ros/log
sudo systemctl daemon-reload && sudo systemctl start robot_ros
sudo systemctl enable robot_ros

journalctl -u robot_ros


