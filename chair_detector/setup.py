from setuptools import find_packages, setup

package_name = 'chair_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='administrador',
    maintainer_email='administrador@todo.todo',
    description='Detector de ocupación de sillas con YOLO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # FORMATO: 'nombre_ejecutable = nombre_carpeta.nombre_archivo:main'
            'chair_detector_node = chair_detector.chair_detector_node:main',
        ],
    },
)
