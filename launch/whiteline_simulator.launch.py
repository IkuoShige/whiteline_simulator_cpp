from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Whiteline simulator node with automatic movement
        Node(
            package='whiteline_simulator_cpp',
            executable='whiteline_simulator_node',
            name='whiteline_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'keyboard_control': False,  # Automatic movement
                'generate_soccer_field': True,
                'point_spacing': 0.05,
                'fov': 1.5708,  # 90 degrees
                'max_distance': 8.0,
                'field_length': 14.0,
                'field_width': 9.0
            }]
        )
    ])