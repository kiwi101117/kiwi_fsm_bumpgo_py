from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bumpgo_node = Node(package = 'kiwi_fsm_bumpgo_py',
                       executable = 'bumpgo_main',
                       output = 'screen',
                       parameters=[{
                           'use_sim_time': True
                       }],
                       remappings=[
                           ('input_scan', '/scan_raw'),
                           ('output_vel', '/nav_vel')
                       ])
    ld = LaunchDescription()
    ld.add_action(bumpgo_node)

    return ld