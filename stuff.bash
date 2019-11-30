cd catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/automove/src/test.py
chmod +x src/navigation/msg/Target.msg
chmod +x src/navigation/src/test.py
cd src/pf_localisation/src/laser_trace
./compile.sh
cd ../../../..
catkin_make
