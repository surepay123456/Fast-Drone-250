sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch realsense2_camera rs_camera.launch & sleep 10;
roslaunch mavros px4.launch & sleep 10;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch vins fast_drone_250.launch
wait;
