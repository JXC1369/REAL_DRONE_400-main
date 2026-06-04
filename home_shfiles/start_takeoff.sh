cd REAL_DRONE_400-main;
source devel/setup.bash;
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
