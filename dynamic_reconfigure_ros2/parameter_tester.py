
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterNotDeclaredException
from ros_param_manager import list_remote_parameters, get_remote_parameters, extract_param_value

def setparam(rosnode, parameter_name, parameter_value, user_data):
    param_type = user_data["type"]
    
    if not rosnode.has_parameter(parameter_name):
        rosnode.declare_parameter(parameter_name, parameter_value)  # Declare the parameter
    
    try:
        print("setting parameter!")
        rosnode.set_parameters([rclpy.parameter.Parameter(parameter_name, param_type, parameter_value)])
        rosnode.get_logger().info(f"Parameter {parameter_name} set to {parameter_value}")
    except ParameterNotDeclaredException as e:
        rosnode.get_logger().error(f"Failed to set parameter {parameter_name}: {e}")

def get_params(rosnode):
    """Returns List of Triplets. Each triplet has (name, value, parameter type).
    """
    list_result = list_remote_parameters(rosnode)
    if list_result is None or len(list_result.result.names) == 0: 
        return None # No client online
    
    param_names = list_result.result.names
    get_result = get_remote_parameters(rosnode, param_names)
    if get_result is None:
        return None # No client online
    
    param_list = []
    for i in range(len(get_result.values)):
        value, type = extract_param_value(get_result.values[i])
        param_list.append((param_names[i], value, type))
    return param_list

def main():
    rclpy.init()

    # Declare some parameters
    rosnode = Node("parameter_tester")
    rosnode.declare_parameter("/test", 1)
    rosnode.declare_parameter("/parint2", 2)
    rosnode.declare_parameter("/parbool", False)
    rosnode.declare_parameter("/parfloat1", 1.23456)
    rosnode.declare_parameter("/parfloat2", 2.34567)
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(rosnode)
    spinning_thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    spinning_thread.start()

    print(get_params(rosnode))

    time.sleep(3)
    
    setparam(rosnode, "/test", 2, user_data={"type": Parameter.Type.INTEGER})
    setparam(rosnode, "/parint2", 30, user_data={"type": Parameter.Type.INTEGER})

    print(get_params(rosnode))

    time.sleep(3)

    setparam(rosnode, "/test", 3, user_data={"type": Parameter.Type.INTEGER})
    setparam(rosnode, "/parfloat1", 123.456, user_data={"type": Parameter.Type.DOUBLE})
    setparam(rosnode, "/parfloat2", 234.567, user_data={"type": Parameter.Type.DOUBLE})
    
    print(get_params(rosnode))

    time.sleep(3)

    setparam(rosnode, "/test", 2, user_data={"type": Parameter.Type.INTEGER})
    setparam(rosnode, "/parbool", True, user_data={"type": Parameter.Type.BOOL})

    print(get_params(rosnode))

    try:
        # Keep node alive and handle ROS interactions
        rclpy.spin(rosnode)
    except KeyboardInterrupt:
        pass
    
if __name__ == "__main__":
    main()
