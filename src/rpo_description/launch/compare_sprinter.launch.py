import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rpo_description')

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='full_assembly_sprinter',
        description='Exported URDF to compare against rpo_model.urdf'
    )

    # Reuse compare.launch.py: rpo_model on one side, the sprinter on the other
    compare_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'compare.launch.py')),
        launch_arguments={'model': LaunchConfiguration('model')}.items(),
    )

    return LaunchDescription([
        declare_model,
        compare_launch,
    ])
