
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import threading
import time
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()

    rosnode = Node("parameter_tester")
    rosnode.declare_parameter("/parint1", 1)
    rosnode.declare_parameter("/parint2", 2)
    rosnode.declare_parameter("/parbool", False)
    rosnode.declare_parameter("/parfloat1", 1.23456)
    rosnode.declare_parameter("/parfloat2", 2.34567)
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(rosnode)

    spinning_thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    spinning_thread.start()

    try:
        # Keep node alive and handle ROS interactions
        rclpy.spin(rosnode)
    except KeyboardInterrupt:
        pass
    
if __name__ == "__main__":
    main()
