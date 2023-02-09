cd ~/group6_ws/src/group6_pkg/scripts/
roslaunch group6_pkg hardware_A.launch &
roslaunch group6_pkg spawn.launch &
rosrun group6_pkg online_cube_detectio.py & 
python3 emergency_shutdown.py &
python3 blind_navigation.py &
python3 Parking_with_sub.py &
wait
