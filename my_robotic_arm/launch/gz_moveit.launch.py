import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# --- HELPER FUNCTIONS ---
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except (EnvironmentError, FileNotFoundError):
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (EnvironmentError, FileNotFoundError):
        return None

def generate_launch_description():
    
    # 1. SETUP VARIABLES
    robot_pkg = "my_robotic_arm"
    moveit_pkg = "my_robotic_arm_moveit_config"
    
    # 2. ROBOT DESCRIPTION (URDF)
    # Calculate absolute path to controllers file for the URDF argument
    controllers_file_path = os.path.join(
        get_package_share_directory(robot_pkg), 
        'config', 
        'my_controllers.yaml'
    )
    
    urdf_path = os.path.join(get_package_share_directory(robot_pkg), 'urdf', 'arm_urdf.urdf')
    doc = xacro.process_file(urdf_path, mappings={'controllers_yaml': controllers_file_path})
    robot_description_content = doc.toxml()
    robot_description = {'robot_description': robot_description_content}

    # 3. SRDF (SEMANTICS)
    srdf_content = load_file(moveit_pkg, 'config/arm_urdf.srdf')
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # 4. KINEMATICS
    kinematics_yaml = load_yaml(moveit_pkg, 'config/kinematics.yaml')

    # 5. OMPL PLANNING (Fixing the crash here)
    # Since ompl_planning.yaml is missing, we define a default RRTConnect planner here.
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/ResolveConstraintFrames""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    # Try to load the file, but if it fails, use the default dictionary above without crashing
    ompl_yaml_loaded = load_yaml(moveit_pkg, 'config/ompl_planning.yaml')
    if ompl_yaml_loaded is not None:
        ompl_planning_pipeline_config['move_group'].update(ompl_yaml_loaded)
    else:
        # Inject standard planners if file is missing
        ompl_planning_pipeline_config['move_group'].update({
            'planner_configs': {
                'RRTConnectkConfigDefault': {
                    'type': 'geometric::RRTConnect',
                    'range': 0.0,
                },
                'RRTstarConfigDefault': {
                    'type': 'geometric::RRTstar',
                    'range': 0.0,
                }
            }
        })

    # 6. TRAJECTORY EXECUTION
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # 7. CONTROLLERS
    moveit_controllers_yaml = load_yaml(moveit_pkg, 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # 8. NODES
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Spawn Robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robotic_arm",
            "-z", "0.1",
        ],
        output="screen",
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Robot State Publisher
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            {"use_sim_time": True},
        ],
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory(moveit_pkg), 'config', 'moveit.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True}
        ],
    )

    # Spawners
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # EXECUTION FLOW
    return LaunchDescription([
        gazebo,
        bridge,
        node_rsp,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[arm_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_spawner,
                on_exit=[run_move_group_node, rviz_node],
            )
        ),
    ])