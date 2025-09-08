import time
import rclpy
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import PoseStamped


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """一个辅助函数，用于规划和执行轨迹。"""
    logger.info("Starting trajectory planning.")
    
    plan_result = None
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        logger.info("No specific parameters provided. Planning with defaults.")
        plan_result = planning_component.plan()

    if plan_result and plan_result.trajectory:
        logger.info("Planning succeeded. Executing plan.")
        robot_trajectory = plan_result.trajectory
        
        execution_result = robot.execute(robot_trajectory, controllers=[])
        
        if execution_result:
            logger.info("Plan executed successfully.")
        else:
            logger.error("Plan execution failed!")
            
    else:
        logger.error("Planning failed. No valid trajectory found.")

def main():
    # MoveItPy Setup
    rclpy.init()
    logger = get_logger("moveit_py_pose_goal")

    path = "/home/lzy/arm_ws/src/moveit_motion_api/config/moveit_cpp.yaml"
    print(f'moveit cpp config path is :{path}')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="arm", package_name="moveit_config"
        )
        .moveit_cpp(path)
        .to_moveit_configs()
    )

    params = moveit_config.to_dict()  # 节点moveitpy的参数

    # instantiate MoveItPy instance and get planning component
    robot = MoveItPy(node_name="moveit_py", config_dict=params)
    arm_group = robot.get_planning_component("arm")
    hand_group = robot.get_planning_component("hand")

    logger.info("MoveItPy instance created")
    print(robot)
    
    # 延时时间
    sleep_duration = 5.0

    # 1. 规划到预定义的“ready”位置
    arm_group.set_start_state(configuration_name="stand")
    hand_group.set_start_state(configuration_name="open")
    arm_group.set_goal_state(configuration_name="ready")
    hand_group.set_goal_state(configuration_name="close")
    plan_and_execute(robot, arm_group, logger)
    time.sleep(1.0)
    plan_and_execute(robot, hand_group, logger)
    time.sleep(sleep_duration)
    print("-------------------- First plan executed. --------------------")
    
    # 2. 规划到指定的关节角位置
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('arm', [0, -0.4, -0.79, -0.79, -1.10, 1.55])
    arm_group.set_goal_state(robot_state=robot_state)
    arm_group.set_start_state_to_current_state()
    plan_and_execute(robot, arm_group, logger)
    time.sleep(sleep_duration)
    print("-------------------- Second plan executed. --------------------")
    
    # # 3. 规划到指定的笛卡尔坐标(i can't run it ,because i can't find a vaild goal)
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = "base_link"
    # pose_goal.pose.orientation.x = 0.71
    # pose_goal.pose.orientation.y = 0.71
    # pose_goal.pose.position.x = 0.01
    # pose_goal.pose.position.y = 0.01
    # pose_goal.pose.position.z = 0.01
    
    # arm_group.set_start_state_to_current_state()
    # arm_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="gripper_base_link")
    # plan_and_execute(robot, arm_group, logger)
    # time.sleep(sleep_duration)
    # print("-------------------- Third plan executed. --------------------")

    rclpy.shutdown()

if __name__ == "__main__":
    main()