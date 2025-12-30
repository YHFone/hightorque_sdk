from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    dof_type = LaunchConfiguration('dof_type')
    mcu_type = LaunchConfiguration('mcu_type')
    model_type = LaunchConfiguration('model_type')
    design = LaunchConfiguration('design')

    param_filename = PythonExpression([
        TextSubstitution(text="'"),
        LaunchConfiguration('dof_type'),
        TextSubstitution(text='dof_'),
        LaunchConfiguration('mcu_type'),
        TextSubstitution(text='_model_'),
        LaunchConfiguration('model_type'),
        TextSubstitution(text='_'),
        LaunchConfiguration('design'),
        TextSubstitution(text='_params.yaml'),
        TextSubstitution(text="'"),
    ])

    param_file = PathJoinSubstitution([
        FindPackageShare('livelybot_description'),
        'robot_param',
        param_filename,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('dof_type', default_value='12'),
        DeclareLaunchArgument('mcu_type', default_value='STM32H730'),
        DeclareLaunchArgument('model_type', default_value='test'),
        DeclareLaunchArgument('design', default_value='Orin'),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value=param_file,
            description='Absolute path to the robot parameter YAML file.',
        ),
    ])
