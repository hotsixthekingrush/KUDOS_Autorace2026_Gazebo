import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start_arena(context):
    arena = Path(get_package_share_directory('kudos_arena'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    generator = Path(get_package_prefix('kudos_arena')) / 'lib/kudos_arena/make_arena.py'
    temporary = Path(tempfile.mkdtemp(prefix='kudos_arena_'))
    world = temporary / 'track_2026.world'
    try:
        subprocess.run([
            sys.executable, str(generator),
            '--config', str(arena / 'config/track.json'),
            '--output', str(world),
            '--branch-sign', LaunchConfiguration('branch_sign').perform(context),
        ], check=True)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    def cleanup(context, *args, **kwargs):
        shutil.rmtree(temporary, ignore_errors=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', os.pathsep.join(filter(None, [
            str(Path(get_package_prefix('kudos_arena')) / 'lib'), os.environ.get('GAZEBO_PLUGIN_PATH', ''),
        ]))),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(filter(None, [
            str(arena / 'models'), os.environ.get('GAZEBO_MODEL_PATH', ''),
        ]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(gazebo / 'launch/gazebo.launch.py')),
            launch_arguments={'world': str(world), 'verbose': 'true'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('branch_sign', default_value='random', choices=['random', 'A', 'B'],
                              description='Select branch sign once for this Gazebo run.'),
        OpaqueFunction(function=start_arena),
    ])
