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


# from launch import LaunchDescription
# from launch_ros.actions import Node

# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_dir = get_package_share_directory('kiwi_fsm_bumpgo_py')
#     param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

#     bumpgo_node = Node(package = 'kiwi_fsm_bumpgo_py',
#                        executable = 'bumpgo_main',
#                        output = 'screen',
#                        parameters=[param_file], 
#                        remappings=[
#                            ('input_scan', '/scan_raw'),
#                            ('output_vel', '/nav_vel')
#                        ])
#     ld = LaunchDescription()
#     ld.add_action(bumpgo_node)

#     return ld