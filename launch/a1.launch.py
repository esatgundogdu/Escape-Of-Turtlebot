import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    worldFileName = 'maze-6.world'
    worldPath = os.path.join("~/ros2_ws/src/assignment1/",'worlds', worldFileName)

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    executeProcessGazebo = ExecuteProcess(
        cmd=[['gazebo'], [worldPath]],
        shell = True,
    )

    robotStatePublisherLD = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    executeProcessCartograph = ExecuteProcess(
        cmd=[['ros2',' launch ','turtlebot3_cartographer ','cartographer.launch.py ', 'use_sim_time:=true']],
        shell=True)
    executeReferee = ExecuteProcess(
        cmd=[['ros2',' run ','assignment1 ','a1_referee']],
        shell=True)
    ld = LaunchDescription()
    
    ld.add_action(executeProcessGazebo)
    ld.add_action(robotStatePublisherLD)
    ld.add_action(executeProcessCartograph)
    ld.add_action(executeReferee)

    return ld