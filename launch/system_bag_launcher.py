
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    
    queue_size = LaunchConfiguration('queue_size')
    pub_freq = LaunchConfiguration('pub_freq')
    record_all_topics = LaunchConfiguration('record_all_topics')

    return LaunchDescription([

        DeclareLaunchArgument(
            'queue_size',
            default_value='10.0'
        ),
        
        DeclareLaunchArgument(
            'pub_freq',
            default_value='2.0'
        ),

        DeclareLaunchArgument(
            'record_all_topics',
            default_value='True'
        ),

        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            parameters=[{
                "queue_size": LaunchConfiguration('queue_size'),
                "pub_freq": LaunchConfiguration('pub_freq'),
            }]
        ),


        ExecuteProcess(
        condition=IfCondition(record_all_topics),
        cmd=[
            'ros2', 'bag', 'record', '-o package_bag', '-a'
        ],
        shell=True
        )

    ])