import rclpy
from rclpy.node import Node
import sys

def main():
    try:
        rclpy.init()
        node = Node('sanity_check')
        print("\n" + "="*40)
        print("✅ SUCCESS: ROS 2 Jazzy is pre-installed in this container!")
        print(f"Node Name: {node.get_name()}")
        print("="*40 + "\n")
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
