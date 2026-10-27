import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TABLE_H = 0.72  # the robot base sits on the table top
CAMS = ('overview', 'side', 'top')


def setup(context):
    share = get_package_share_directory('chess_arm_demo')
    urdf = subprocess.check_output([
        'xacro', os.path.join(share, 'urdf', 'chess_ur.urdf.xacro'),
        'simulation_controllers:=' + os.path.join(share, 'config', 'controllers.yaml')]).decode()
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    flags = ' -s -r -v2 --headless-rendering '   # the server always renders on the GPU; the window is a separate viewer
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': flags + os.path.join(share, 'worlds', 'chess.sdf')}.items())
    window = TimerAction(period=6.0, actions=[ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v2', '--gui-config', os.path.join(share, 'config', 'gazebo_gui.config')],
        output='screen')]) if gui else None
    bridge = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
              '/world/chess/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
              '/gripper/left_cmd@std_msgs/msg/Float64]gz.msgs.Double',
              '/gripper/right_cmd@std_msgs/msg/Float64]gz.msgs.Double',
              '/gripper/joint_states_gz@sensor_msgs/msg/JointState[gz.msgs.Model']
    bridge += [f'/cam/{c}@sensor_msgs/msg/Image[gz.msgs.Image' for c in CAMS]
    spawner = lambda *a: Node(package='controller_manager', executable='spawner', arguments=[*a, '-c', '/controller_manager'])
    return [
        gz,
        *([window] if window else []),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': urdf}]),
        Node(package='ros_gz_sim', executable='create', arguments=['-string', urdf, '-name', 'ur', '-world', 'chess', '-z', str(TABLE_H)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge,
             remappings=[('/world/chess/dynamic_pose/info', '/gz_poses'),
                         ('/gripper/joint_states_gz', '/gripper/joint_states')]),
        spawner('joint_state_broadcaster'), spawner('arm_controller'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='open the Gazebo window; false runs headless'),
        OpaqueFunction(function=setup)])
