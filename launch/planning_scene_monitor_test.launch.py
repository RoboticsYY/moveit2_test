import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# https://github.com/ros-planning/moveit2/pull/166
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    # https://github.com/ros-planning/moveit2/pull/166
    robot_description_config = load_file('moveit_resources', 'panda_description/urdf/panda.urdf')
    robot_description = {'robot_description' : robot_description_config}
    
    robot_description_semantic_config = load_file('moveit_resources', 'panda_moveit_config/config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}] - {message}'

    return LaunchDescription( [
        Node(package='moveit2_test',
             node_executable='planning_scene_monitor_test',
             output='screen',
             parameters=[
                  robot_description,
                  robot_description_semantic
             ]),
    ])