import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class TestMotor(Node):
    def __init__(self):
        super().__init__("test_motor")
        self.publisher = self.create_publisher(Int16, "/wamv_caleuche/thrusters/left/cmd_vel", 1)
        self.timer = self.create_timer(3, self.timer_callback)
        self.delta = 50
        
        self.msg = Int16()  
        self.msg.data = 0

    def timer_callback(self):
        msg = Int16()

        if self.msg.data == 1000:
            self.delta = -50
        
        if self.msg.data == -1000:
            self.delta = 50
        
        self.publisher.publish(self.msg)

        print(self.msg.data)

        self.msg.data = self.msg.data + self.delta

def main():
    rclpy.init()
    test_motor = TestMotor()
    rclpy.spin(test_motor)
    test_motor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()