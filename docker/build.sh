mkdir /root/c1t_ws/src/navigation2_extensions
cp -r . /root/c1t_ws/src/navigation2_extensions/
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y -r
. /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
