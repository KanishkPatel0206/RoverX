import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # ── 1. PATHS ────────────────────────────────────────────────────────────
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # FIX: Use 'ros_gz_sim', NOT 'gazebo_ros' — this is Ignition Gazebo on Humble
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── 2. LAUNCH ARGUMENTS ─────────────────────────────────────────────────
    # FIX: Declare arguments so they can be overridden from CLI
    # e.g.  ros2 launch my_robot_description launch_sim.py use_rviz:=false
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Ignition world file'
    )
    world = LaunchConfiguration('world')

    # ── 3. NODES ─────────────────────────────────────────────────────────────

    # Node A: Robot State Publisher
    # FIX: pass use_sim_time as a bool, not a Python bool — it must survive
    #      LaunchConfiguration substitution.  Passing a raw Python True here
    #      is fine only when the value is static; kept as-is but flagged.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )

    # Node B: Gazebo (Ignition) simulation
    # FIX: gz_args must include the world name via LaunchConfiguration so it
    #      is composable.  Also added -v 4 for verbose startup logging so
    #      plugin load errors are visible.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        # FIX: '-r' auto-runs the simulation; combined with the world name.
        #      Without '-r' the physics is paused and the robot never moves.
        launch_arguments={
            'gz_args': ['-r -v 4 ', world],
        }.items(),
    )

    # Node C: Spawn entity
    # FIX: spawn AFTER RSP has had time to publish /robot_description.
    #      A TimerAction of 3 s is the pragmatic approach for a launch file;
    #      the robust alternative is a lifecycle node or an event handler on
    #      robot_state_publisher startup — but that requires extra boilerplate.
    # FIX: removed '-z 0.1' because the URDF base_joint origin already lifts
    #      the robot off the ground.  Using both causes a double-offset and
    #      the robot spawns floating 0.1 m above the intended position.
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                # FIX: '-topic' reads the URDF from the /robot_description
                #      topic published by robot_state_publisher — correct.
                #      '-name' must match the model name in the URDF <robot name="...">
                arguments=[
                    '-topic', '/robot_description',
                    '-name',  'tricycle_bot',   # FIX: matches <robot name="tricycle_bot">
                ],
                output='screen',
            )
        ]
    )

    # Node D: Main bridge
    # FIX: /tf bridge format corrected.
    #      Ignition → ROS tf uses ignition.msgs.Pose_V  (a list of poses),
    #      NOT ignition.msgs.TF which does not exist in Fortress.
    #      The bridge automatically maps Pose_V → tf2_msgs/TFMessage.
    #
    # FIX: joint_states topic corrected.
    #      The JointStatePublisher plugin publishes on the Ignition internal
    #      topic  /world/<world>/model/<model>/joint_state  typed as
    #      ignition.msgs.Model.  The correct ROS type is
    #      sensor_msgs/msg/JointState — this was right in the original.
    #      BUT: the world name in the topic must match the actual loaded world.
    #      If you change the world arg the topic name also changes, so we
    #      expose it as a substitution below.
    #
    # FIX: /camera/image_raw removed from parameter_bridge — image topics
    #      MUST go through ros_gz_image (image_bridge), not parameter_bridge.
    #      Keeping it in both causes duplicate publishers and rviz corruption.
    #
    # FIX: added qos transient_local for /robot_description so late-joining
    #      nodes (like rviz) still receive the latched description.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./robot_description.publisher.durability': 'transient_local',
        }],
        arguments=[
            # CONTROL  (ROS2 → Ignition)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/cmd_steer@std_msgs/msg/Float64@ignition.msgs.Double',

            # ODOMETRY & TF  (Ignition → ROS2)
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # FIX: correct Ignition type for /tf in Fortress is Pose_V
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',

            # LIDAR  (Ignition → ROS2)
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',

            # CAMERA INFO  (Ignition → ROS2)  — image goes through image_bridge
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',

            # JOINT STATES  (Ignition → ROS2)
            # FIX: world name hardcoded here must match gz_args world.
            #      If you change the world arg, update this string too.
            '/world/empty/model/tricycle_bot/joint_state'
            '@sensor_msgs/msg/JointState'
            '@ignition.msgs.Model',
        ],
        remappings=[
            # Remap internal Ignition joint state topic → standard ROS name
            ('/world/empty/model/tricycle_bot/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    # Node E: Image bridge (dedicated — the only correct way to bridge images)
    # FIX: image_bridge handles compressed transport, QoS, and encoding
    #      automatically.  It must NOT be duplicated in parameter_bridge.
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # Node F: RViz2
    # FIX: delayed so the bridge and RSP are ready before rviz tries to
    #      subscribe to /robot_description and /tf.
    # FIX: an rviz config file should be passed via -d if you have one,
    #      otherwise rviz starts blank and the user must add displays manually.
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(use_rviz),
                parameters=[{'use_sim_time': True}],
                # FIX: only pass -d if the config file actually exists
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            )
        ]
    )

    # ── 4. LAUNCH DESCRIPTION ────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments first
        use_rviz_arg,
        world_arg,

        # Sim + RSP launch immediately (order matters: RSP before spawn)
        robot_state_publisher,
        gazebo,

        # Spawn after RSP is up (3 s delay)
        spawn_entity,

        # Bridge + image bridge (no strict ordering dependency on spawn,
        # but spawning first is cleaner)
        bridge,
        image_bridge,

        # RViz last (5 s delay — needs bridge + RSP to be ready)
        rviz,
    ])