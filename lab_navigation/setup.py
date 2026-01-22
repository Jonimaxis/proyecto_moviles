from setuptools import setup
import os
from glob import glob

package_name = 'lab_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Mundos de simulación (World)
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        
        # 3. Modelos 3D (Si los usas)
        (os.path.join('share', package_name, 'models'), glob('models/*')),

        # --- AÑADIDOS NECESARIOS PARA NAVEGACIÓN ---
        # 4. Mapas (.yaml y .pgm) -> IMPORTANTE para Nav2
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        
        # 5. Configuración (nav2_params.yaml) -> IMPORTANTE para Nav2
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usuario',
    maintainer_email='usuario@todo.todo',
    description='Paquete destinado a la navegación del robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_patrol = lab_navigation.robot_patrol:main',
        ],
    },
)
