from setuptools import find_packages, setup

package_name = 'robot_1dof_control'

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
    maintainer='abdiel',
    maintainer_email='abdiel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Nombre_del_ejecutable = nombre_del_paquete.nombre_del_script:main
            'computed_torque = robot_1dof_control.computed_torque_node:main',
            'imu_state_estimator = robot_1dof_control.imu_state_estimator:main',
        ],
    },
)
