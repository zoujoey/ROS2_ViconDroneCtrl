import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class PublisherNode(Node):
    def __init__(self):
        super().__init__('pose_pub')
        self.pos_pub = self.create_publisher(String, '/Wifi/Channel_Four', 10)
        self.bind = 'x'
        instructions = """
        w: Forward, (East, Neg Y)                                      i: Up
        a: Left, (North, Pos X)                                        j: Yaw Counter Clockwise
        s: Back, (West, Pos Y)                                         k: Down
        d: Right, (South, Neg X)                                       l: Yaw Clockwise
        _______________________________________________________
        c: Control Mode/Hold
        x: Failsafe Land
        r: Land (Return)
        f: Float (Hover 20cm)
        q: Square Loop
        e: Circle Loop
        h: Helix Loop
        t: Linear Setpoints
        p: Random Continuous Setpoints
        """
        self.get_logger().info(instructions)
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()

    def on_key_press(self, key):
        try:
            self.bind = key.char
            self.publish_key()
        except AttributeError:
            pass

    def on_key_release(self, key):
        if key == keyboard.Key.esc:
            # Stop the listener
            return False

    def publish_key(self):
        msg = String()
        msg.data = self.bind
        self.pos_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C pressed, exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()