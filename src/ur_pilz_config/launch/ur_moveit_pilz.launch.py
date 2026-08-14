# Copyright (c) 2021 PickNik, Inc.
#
# UR MoveIt + OMPL + Pilz configuration for ROS 2 Humble.
# This version keeps UR's official robot/config files and loads Pilz settings
# from this package's YAML files instead of duplicating them in Python.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    # ------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    _publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # ------------------------------------------------------------
    # UR robot description
    # ------------------------------------------------------------
    # joint_limit_params = PathJoinSubstitution(
    #     [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    # )
    joint_limit_params = "/home/zzz/work/src/ur_pilz_config/config/ur5e_joint_limits.yaml"
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "visual_parameters.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # ------------------------------------------------------------
    # SRDF
    # ------------------------------------------------------------
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "srdf",
                    moveit_config_file,
                ]
            ),
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    publish_robot_description_semantic = {
        "publish_robot_description_semantic":
            _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package),
            "config",
            "kinematics.yaml",
        ]
    )

    # ------------------------------------------------------------
    # Joint limits from official ur_moveit_config
    # ------------------------------------------------------------
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join(
                "config",
                str(moveit_joint_limits_file.perform(context)),
            ),
        )
    }

    # ------------------------------------------------------------
    # Pilz joint deceleration limits
    #
    # NOTE:
    # Official UR MoveIt joint_limits.yaml does not provide Pilz
    # max_deceleration entries in this setup.
    #
    # Keep these conservative/test values until you choose limits
    # appropriate for the actual robot/application.
    # ------------------------------------------------------------
    joint_limits = robot_description_planning[
        "robot_description_planning"
    ]["joint_limits"]

    for joint_name in [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]:
        joint_limits[joint_name]["has_deceleration_limits"] = True
        joint_limits[joint_name]["max_deceleration"] = -5.0

    # ------------------------------------------------------------
    # Cartesian limits for Pilz LIN / CIRC
    #
    # Load from:
    # ur_pilz_config/config/pilz_cartesian_limits.yaml
    # ------------------------------------------------------------
    pilz_cartesian_yaml = load_yaml(
        "ur_pilz_config",
        "config/pilz_cartesian_limits.yaml",
    )

    robot_description_planning[
        "robot_description_planning"
    ].update(
        pilz_cartesian_yaml["robot_description_planning"]
    )

    # ------------------------------------------------------------
    # OMPL planning pipeline
    #
    # Give the pipeline the clear name "ompl".
    # ------------------------------------------------------------
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters":
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(
        "ur_moveit_config",
        "config/ompl_planning.yaml",
    )
    ompl_planning_pipeline_config["ompl"].update(
        ompl_planning_yaml
    )

    # ------------------------------------------------------------
    # Pilz planning pipeline
    #
    # Load from:
    # ur_pilz_config/config/
    # pilz_industrial_motion_planner_planning_planner.yaml
    # ------------------------------------------------------------
    pilz_planning_pipeline_config = load_yaml(
        "ur_pilz_config",
        "config/pilz_industrial_motion_planner_planning_planner.yaml",
    )

    # ------------------------------------------------------------
    # Tell MoveIt which pipelines exist
    # ------------------------------------------------------------
    planning_pipelines_config = {
        "planning_pipelines": [
            "ompl",
            "pilz_industrial_motion_planner",
        ],
        "default_planning_pipeline": "ompl",
    }

    # ------------------------------------------------------------
    # Trajectory execution
    # ------------------------------------------------------------
    controllers_yaml = load_yaml(
        "ur_moveit_config",
        "config/controllers.yaml",
    )

    # Official UR behavior:
    # with use_sim_time=true, use joint_trajectory_controller
    # instead of scaled_joint_trajectory_controller.
    change_controllers = context.perform_substitution(use_sim_time)

    if change_controllers == "true":
        controllers_yaml[
            "scaled_joint_trajectory_controller"
        ]["default"] = False
        controllers_yaml[
            "joint_trajectory_controller"
        ]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/"
            "MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin":
            "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # ------------------------------------------------------------
    # move_group
    # ------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,

            planning_pipelines_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package),
            "rviz",
            "view_robot.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,

            planning_pipelines_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,

            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------
    # MoveIt Servo
    # ------------------------------------------------------------
    servo_yaml = load_yaml(
        "ur_moveit_config",
        "config/ur_servo.yaml",
    )

    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            {"moveit_servo": servo_yaml},
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    return [
        move_group_node,
        rviz_node,
        servo_node,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser(
                "~/.ros/warehouse_ros.sqlite"
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
