import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class HandController(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        self.publiser = self.create_publisher(Float64MultiArray,"/hand_controller/commands",10)
        period = 2.0
        self.timer = self.create_timer(period,self.move)
        self.value = 0.1
        
    def move(self):
        commands = Float64MultiArray()
        if self.value == 0.0:
            self.value = 0.1
        else:
            self.value = 0.0
        commands.data = [self.value]
        self.publiser.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    node = HandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()