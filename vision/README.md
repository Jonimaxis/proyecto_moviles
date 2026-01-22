#en cada terminal cargar dependencias con source install/setup.bash
#Navegación
1:ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/administrador/aruco_ws/src/map2gazebo/mapa_editado.yaml

2:ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:=['map_server']

3:ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

#Visión
4:ros2 run camera_subscriber controlador_video ó driver_ip

5:ros2 run camera_subscriber detectar_markers

#Paquete a parte para el entorno de yolo y el nodo detección ocupación
6:source ~/ros_yolo_env/bin/activate
source ~/aruco_ws/install/setup.bash
python3 ~/aruco_ws/src/silla_detector/silla_detector/detector_node.py

#Visualización rviz
7:ros2 run camera_subscriber fusion_topics

8:ros2 run camera_subscriber visualizador_mesas 

9:rviz2

#Si se utiliza el vídeo real
10:ros2 topic pub --once /control_video std_msgs/msg/String "data: 'play'"


