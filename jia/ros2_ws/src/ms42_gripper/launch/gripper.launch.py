from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_node(context):
    overrides = {}
    for name, converter in [('port', str), ('travel_turns', float), ('speed_rad_s', float)]:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = converter(value)
    dry_run = LaunchConfiguration('dry_run').perform(context)
    if dry_run:
        if dry_run.lower() not in ('true', 'false'):
            raise ValueError('dry_run must be true or false')
        overrides['dry_run'] = dry_run.lower() == 'true'
    return [Node(
        package='ms42_gripper', executable='gripper_node', name='ms42_gripper',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
    )]


def generate_launch_description():
    config = PathJoinSubstitution([
        get_package_share_directory('ms42_gripper'), 'config', 'gripper.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('port', default_value='', description='Empty uses YAML value'),
        DeclareLaunchArgument('dry_run', default_value='', description='Empty uses YAML value'),
        DeclareLaunchArgument('travel_turns', default_value='', description='Empty uses YAML value'),
        DeclareLaunchArgument('speed_rad_s', default_value='', description='Empty uses YAML value'),
        OpaqueFunction(function=launch_node),
    ])
