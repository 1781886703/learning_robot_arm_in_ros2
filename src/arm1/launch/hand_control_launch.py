from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

package_path = get_package_share_directory('arm1')
print("package_path:",package_path)
urdf_path = package_path + "/urdf/arm_ros2_control.urdf"
ros2_control_yaml_path = package_path + "/config/arm_controllers.yaml"
rviz_config_path = package_path + "/config/rviz.rviz"
robot_desc = open(urdf_path).read()

def generate_launch_description():
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name = "robot_state_publisher",
        output = "both",
        parameters=[
            {"use_sim_time":True},
            {"robot_description":robot_desc}
        ],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name= "rviz",
        arguments=["-d",rviz_config_path],
    )
    
    contorller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output = "both",
        parameters=[ros2_control_yaml_path],
    )
    
    contorller_node = Node(
        package="controller_manager",
        executable="spawner",
        name = "controller_node",
        output = "both",
        arguments=['arm_controller','hand_controller','joint_state_broadcaster'],
    )
    
    test_hand_control_node = Node(
        package="arm1",
        executable="test_hand_control",
    )
    return LaunchDescription(
        [robot_desc_node,
         rviz_node,
         contorller_manager_node,
         contorller_node,
         test_hand_control_node
        ]
    )