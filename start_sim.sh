#!/bin/bash
xhost +

CONTAINER_NAME="cyberdog"

# 停止并删除同名旧容器
echo "清理旧容器..."
docker stop $CONTAINER_NAME 2>/dev/null
docker rm $CONTAINER_NAME 2>/dev/null
sleep 1

# 启动容器（固定名字）
echo "启动容器..."
docker run -d \
  --name $CONTAINER_NAME \
  --shm-size="1g" \
  --privileged=true \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/kaka/Mi/cyberdog_sim/src:/home/cyberdog_sim/src \
  -v /home/kaka/Mi/cyberdog_sim/install:/home/cyberdog_sim/install \
  -v /home/kaka/Mi/cyberdog_sim/build:/home/cyberdog_sim/build \
  cyberdog_sim:v2026 sleep infinity

sleep 2

SETUP="cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash"
BUILD="cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && colcon build --merge-install --packages-select cyberdog_race"

# 先编译 cyberdog_race
echo "编译 cyberdog_race..."
gnome-terminal --title="Build" -- bash -c "docker exec -it $CONTAINER_NAME bash -c '$BUILD && echo 编译完成'; bash"

sleep 3

gnome-terminal --title="Gazebo" -- bash -c "docker exec -it $CONTAINER_NAME bash -c '$SETUP && ros2 launch cyberdog_gazebo race_gazebo.launch.py'; bash"

gnome-terminal --title="Control" -- bash -c "docker exec -it $CONTAINER_NAME bash -c 'until ls /dev/shm/development-simulator 2>/dev/null; do sleep 1; done && $SETUP && ros2 launch cyberdog_gazebo cyberdog_control_launch.py'; bash"

gnome-terminal --title="Rviz" -- bash -c "sleep 5 && docker exec -it $CONTAINER_NAME bash -c '$SETUP && ros2 launch cyberdog_visual cyberdog_visual.launch.py'; bash"

echo "仿真启动完成，容器名: $CONTAINER_NAME"
echo "进入容器: docker exec -it $CONTAINER_NAME bash"
