from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

#------prepare------------------
#get the package path, write the package's name
package_path = get_package_share_directory("arm1")
#print the path
print("package_path:",package_path)
urdf_name = 'arm.urdf'    #arm.urdf  / robot.urdf

#define the urdf path
urdf_path = package_path + '/urdf/' + urdf_name
#open the urdf for robot_state_publisher
robot_desc = open(urdf_path).read()
#define the path of rviz_config file
rviz_config_path = package_path + '/config/rviz.rviz'



#------define the main launch--------
def generate_launch_description():
#robot_state_publisher node
    robot_desc_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_desc},
        ]
    )
#joint_gui node
    joint_state_pub_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
#rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name = 'rviz',
        arguments=['-d',rviz_config_path,]
    )
#return
    return LaunchDescription([
        robot_desc_node,
        joint_state_pub_node,
        rviz_node
    ])