from glob import glob

from setuptools import find_packages, setup

package_name = 'chess_arm_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py', 'launch/moveit.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/chess_ur.urdf.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/chess.sdf']),
        ('share/' + package_name + '/assets/pieces', glob('assets/pieces/*.obj') + ['assets/pieces/ATTRIBUTION.md']),
        ('share/' + package_name + '/config', ['config/controllers.yaml', 'config/ompl_planning.yaml', 'config/gazebo_gui.config']),
        ('share/' + package_name + '/config/moveit', ['config/moveit/chess_ur.srdf', 'config/moveit/kinematics.yaml',
                                                      'config/moveit/joint_limits.yaml', 'config/moveit/moveit_controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Capacho',
    maintainer_email='dcapacho3@gmail.com',
    description='UR5e plays chess in Gazebo',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'chess_player = chess_arm_demo.player:main',
    ]},
)
