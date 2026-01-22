# Robot Camarero 

Este proyecto implementa un sistema robótico distribuido para patrullar una habitación (en este caso el laboratorio de robótica), identificar mesas mediante marcadores **ArUco** y detectar la ocupación de sillas utilizando visión artificial (**YOLOv8**). El estado del laboratorio se visualiza en tiempo real en RViz mediante marcadores 3D interactivos.

---

##  Arquitectura del Sistema

Debido a la dinámica de grupo y a la dificultad para usar un único ordenador con todas las tareas,
hemos dividido el proyecto  en **3 Paquetes ROS 2** independientes, que hemos probado en el laboratorio 
desde dos de nuestros ordenadores:

1.  `lab_navigation` (PC 1): Gestión de mapas, navegación autónoma (Nav2) y lógica de patrulla.
2.  `chair_detector` (PC 2): Detección de personas mediante YOLO (requiere entorno virtual).
3.  `aruco_detector` (PC 2): Detección de IDs de mesas, fusión de datos (ID + Ocupación) y visualización.

---

##  Requisitos

### Prerrequisitos
* **ROS 2 Jazzy**.
* Librerías Python: `opencv-python`, `opencv-contrib-python`, `ultralytics`.
* **Entorno Virtual para YOLO**: Se asume que existe un venv en `~/ros_yolo_env` (necesario para evitar conflictos con librerías del sistema).


##  Instrucciones

NAVEGACIÓN

-USO DEL ENTORNO SIMULADO

1. Lanzar el mundo de simulación
   Desde la carpeta "world" (o donde se encuentre el mapa a simular):

     ros2 launch mvsim launch_world.launch.py world_file:=lab_sim.world.xml do_fake_localization:=false use_rviz:=false

2. Lanzar la navegación
   En otra terminal, situada en la misma carpeta que el paso anterior:

     ros2 launch nav2_bringup bringup_launch.py map:=./mapa_lab_simulado.yaml params_file:=./nav2.yaml use_sim_time:=false

3. Configuración de RViz
   En una tercera terminal, abrir rviz2 y realizar las siguientes acciones:
     - Añadir el tópico /map y cambiar su Durability Policy a "Transient Local".
     - Añadir el tópico PoseWithCovariance y configurarlo también como "Transient Local".
     - Usar la herramienta 2D Pose Estimate para establecer la posición inicial del robot.
     - Añadir el tópico MarkerArray para visualizar el estado de las mesas.
     - Opcionalmente, añadir el tópico /path para visualizar la ruta planificada.

--------------------------------------------------

-USO CON TURTLEBOT REAL

1. Configuración previa
   Seguir los pasos indicados en el siguiente enlace:
     https://moodle2025-26.ua.es/moodle/mod/page/view.php?id=104664

2. Lanzar la navegación
   Desde la carpeta "world":

     ros2 launch nav2_bringup bringup_launch.py map:=./mapa_editado.yaml params_file:=./nav2.yaml use_sim_time:=false

3. Configuración de RViz
   En otra terminal, abrir rviz2 y:
     - Añadir el tópico /map y cambiar su Durability Policy a "Transient Local".
     - Añadir el tópico PoseWithCovariance y configurarlo como "Transient Local".
     - Usar la herramienta 2D Pose Estimate para indicar la posición inicial del robot.
     - Añadir el tópico MarkerArray para visualizar el estado de las mesas (es necesario haber iniciado previamente el nodo de "landmark_publisher").
     - Opcionalmente, añadir el tópico /path para visualizar la ruta que sigue el turtlebot.














    
