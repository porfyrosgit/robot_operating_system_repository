# NOT a test.
# It is a runtime ROS node (a monitor / observer node).:
# ✔ Inherits from Node
# ✔ Calls rclpy.spin()
# ✔ Subscribes to real topic
# ✔ Intended to run alongside system
# This is application-side diagnostic tooling, not pytest test code.

import rclpy                           # ROS 2 Python library
from rclpy.node import Node           # The 'Parent' kit for all nodes
from std_msgs.msg import String       # import String, the blueprint for 'Text' messages (what the Brain speaks) from the package std_msgs.mgs

#Go into the std_msgs library, find the msgs sub-folder, and get the blueprint."
#std_msgs/msg/String is the "Swiss Army Knife" of ROS 2. 
# For learning, prototyping, and simple integration tests, it is the fastest and most common way to get nodes talking.
#The imported sensor.msgs library defines the msg names (like data, x, velocity).
#Python just follows those names like a map.
#msg is the "envelope," and .data is the "letter" inside.

# (Node) links this to the ROS 2 system. BrainMonitor is our custom name.
# monitoring the behavior of the Brain.observing the system over time to ensure it stays within "normal" parameters.
class BrainMonitor(Node):
    # 'self' represents this specific monitor object.
    def __init__(self):
        # super() reaches out to the Parent (Node) and names us 'brain_monitor_node'.
        super().__init__('brain_monitor_node')

        # We create a 'Subscriber' (the Ears). 
        # It listens for 'String' messages on the '/posture_command' topic.
        # Whenever a message arrives, it triggers the 'report_callback' function.
        self.sub = self.create_subscription(
            String,                                # Language: Text
            '/posture_command',                   # Topic: The Brain's output channel
            self.report_callback,                # The function to run when we hear something
            10                                   # Buffer: Queue up to 10 messages
        )

        # This function is the 'Callback'. It only runs when a message is received.
        # 'msg' is the data object that just arrived.
    def report_callback(self,msg):
        # We print the data inside the message, to the terminal.
        # This shows us exactly what the C++ Brain decided.
        print(f'MONITOR REPORT: The brain sent command -> {msg.data}')  #.data is a pre-defined field in the imported String class.

def main(args=None):
        rclpy.init(args=args)
        monitor=BrainMonitor()
        print("Monitor started: waiting for brain decisions...")

        try:
            rclpy.spin(monitor)
        except KeyboardInterrupt:

            pass
        monitor.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
     main()