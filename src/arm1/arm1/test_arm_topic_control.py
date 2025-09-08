import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmTrajectoryControl(Node):
    
    def __init__(self):
        super().__init__('arm_trajectory_control_node')
        self.publisher = self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',10)
        period = 6.0 #six seconds send once
        self.timer = self.create_timer(period,self.publish_control)
        
    def publish_control(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        msg.header.stamp = self.get_clock().now().to_msg()
        
        points = []
        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(sec=3,nanosec=0)   
        point1.positions = [1.0,1.0,1.0,1.0,1.0,0.0]          
        points.append(point1)  
        
        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(sec=6,nanosec=0)   
        point2.positions = [0.0,0.0,0.0,0.0,0.0,0.0]          
        points.append(point2)                                      
        
        msg.points = points
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args= args)
    node = ArmTrajectoryControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if "__main__" == __name__:
    main()