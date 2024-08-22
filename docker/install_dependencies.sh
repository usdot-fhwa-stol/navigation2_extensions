git clone https://github.com/usdot-fhwa-stol/carma-msgs /root/c1t_ws/src/carma-msgs && git clone -b nav2_route_server_humble https://github.com/usdot-fhwa-stol/navigation2 /root/c1t_ws/src/navigation2
apt update && \
        apt install -y libnanoflann-dev
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y -r
