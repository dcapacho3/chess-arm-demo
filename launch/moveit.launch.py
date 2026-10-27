"""MoveIt 2 move_group for the chess cell. Start sim.launch.py first (it provides arm_controller)."""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg = (MoveItConfigsBuilder('ur', package_name='chess_arm_demo')
           .robot_description(file_path='urdf/chess_ur.urdf.xacro',
                              mappings={'fixed_fingers': 'true', 'simulation_controllers': ''})
           .robot_description_semantic(file_path='config/moveit/chess_ur.srdf')
           .robot_description_kinematics(file_path='config/moveit/kinematics.yaml')
           .joint_limits(file_path='config/moveit/joint_limits.yaml')
           .trajectory_execution(file_path='config/moveit/moveit_controllers.yaml')
           .planning_pipelines(pipelines=['ompl'])
           .to_moveit_configs())
    return LaunchDescription([Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[cfg.to_dict(), {'use_sim_time': True, 
                                    'publish_robot_description_semantic': True,
                                    'publish_planning_scene': True, 
                                    'publish_geometry_updates': True,
                                    'publish_state_updates': True, 
                                    'publish_transforms_updates': True}])])
