rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y -r
. /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
