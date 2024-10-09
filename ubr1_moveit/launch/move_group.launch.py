from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ubr1", package_name="ubr1_moveit").to_moveit_configs()
    moveit_config.move_group_capabilities["capabilities"] = ["move_group/ExecuteTaskSolutionCapability"]
    return generate_move_group_launch(moveit_config)
