import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@dominio.com',
    description='Paquete de vision para detectar ArUco, fusionar datos y visualizar mesas',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            # 1. Detectar ArUcos (Antes 'detectar_markers')
            'aruco_detector_node = aruco_detector.aruco_detector_node:main',

            # 2. Driver para cámara IP 
            'driver_ip = aruco_detector.driver_ip:main',

            # 3. Controlador de Video (Si usas video grabado)
            'video_controller = aruco_detector.video_controller:main',

            # 4. Fusión de Topics (ArUco + YOLO)
            'fusion_node = aruco_detector.fusion_topics:main',

            # 5. Visualizador de Mesas en RViz 
            'lab_visualizer = aruco_detector.landmark_publisher:main',
        ],
    },
)
