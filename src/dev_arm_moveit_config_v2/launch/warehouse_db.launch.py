from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dev_arm", package_name="dev_arm_moveit_config_v2").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
