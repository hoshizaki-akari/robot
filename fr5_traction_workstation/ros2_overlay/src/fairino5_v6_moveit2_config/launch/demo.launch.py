from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()
    moveit_config = (
    MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
    .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"],default_planning_pipeline="ompl",).to_moveit_configs()
)
    # 生成MoveIt的demo launch
    moveit_demo_launch = generate_demo_launch(moveit_config)


    return moveit_demo_launch
