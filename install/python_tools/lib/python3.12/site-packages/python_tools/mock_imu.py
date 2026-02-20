#mocks imu sensor data, publishes them to /imu_mock topic. Mockdata to be consumed by the C++ brain_node.cpp decision logic
import rclpy                                # Import the ROS 2 library for Python
from rclpy.node import Node                 # Import the tool to create a "Node"
from sensor_msgs.msg import Imu           # Import the specific IMU data structure from the package sensor.msgs.msg
import random                              # Import tool to generate random numbers

#Go into the sensor_msgs library, find the msg sub-folder, and get the blueprint."
#The imported sensor.msgs Library defines the names in the msg (like data, x, velocity).
#Python just follows those names like a map.
#msg is the "envelope," and .data is the "letter" inside.


class MockIMU(Node):              # Create a class named MockIMU that is a ROS 2 Node

    def __init__(self):             # The "Setup" function that runs once at the start

        super().__init__('mock_imu_node')    # Initialize the node with the name 'mock_imu_node'

        self.publisher_ = self.create_publisher(Imu,'/imu_mock',10)  # Create a Publisher: sends 'Imu' messages to topic '/imu_mock', buffer size is 10

# Create a Timer: calls 'self.publish_message' function that will run every 0.5 seconds (2Hz).
        self.timer = self.create_timer(0.5,self.publish_message)  

    def publish_message(self):
        msg=Imu()                  # Create a new, empty Imu message object = the 'Big Box' from the imported Imu blueprint


#msg: The giant box (The Imu object).
#msg.orientation: A smaller section inside labeled "Orientation." (this is technically a Quaternion type object). 
#orientation is the fixed name for the section of the IMU object that tracks rotation.
#msg.orientation.x: reach the 'x' value. the fixed name for the axis within that rotation inside the imu object
        msg.orientation.x = random.uniform(-1.0,1.0)    # Pick a random decimal between -1.0 and 1.0 to simulate sensor "tilt"

        self.publisher_.publish(msg)          # Broadcast the message to the ROS 2 network
 # Print log in the terminal to see what this node name (pretending to be the IMU sensor) is sending
        self.get_logger().info(f'publishing Tilt: {msg.orientation.x:.2f}')  

# The entry point for the script
def main(args=None):
    # Start the ROS 2 communication system
    rclpy.init(args=args)
    # Create an instance of our MockIMU class
    node=MockIMU()
    # Keep the script running in a loop (spin)
    rclpy.spin(node)
    # Clean up the node memory
    node.destroy_node()
    # Shut down ROS 2 properly
    rclpy.shutdown()

# Standard Python boiler-plate to run main()
if __name__=='__main__':
    main()


