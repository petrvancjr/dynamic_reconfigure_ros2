
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import dearpygui.dearpygui as dpg
import threading
from rclpy.executors import MultiThreadedExecutor

from map_rostype_to_widget import get_widget
from ros_param_manager import get_parameters
from rclpy.exceptions import ParameterNotDeclaredException

UPDATE_TAU = 20

class DynamicReconfigure(Node):
    def __init__(self):
        super().__init__("dynamic_reconfigure")

        self.parameter_set_requests = []
        self.param_group = None  # A group to hold the parameter widgets
        self.create_gui()
        self.update_ros_params()

    def create_gui(self):
        with dpg.window(label="Dynamic Reconfigure", width=300, height=300):
            # Create a group to hold all parameter widgets, so we can update them
            self.param_group = dpg.add_group()

    def update_ros_params(self):
        params_in_gui = self.get_gui_parameters()
        params_ros = get_parameters(self)
        print(params_ros)

        # Update parameters inside the group by adding/removing widgets
        for p_name, p_value, p_type in params_ros:
            if p_name in params_in_gui:
                assert dpg.get_item_configuration(p_name)['user_data']['type'] == p_type, \
                    "WARN: The parameter found in ROS has different type than previously loaded parameter"
                params_in_gui.remove(p_name)
                print(f"parameter {p_name} found")
                continue
            else:  # Create new param
                add_widget = get_widget(p_type)
                add_widget(
                        label=p_name,
                        tag=p_name,
                        callback=self.setparam_callback,
                        user_data={"name": p_name, "type": p_type},
                        default_value=p_value,
                        parent=self.param_group  # Add to the group
                    )
                print(f"parameter {p_name} not found, creating")

        # Delete parameters that are no longer declared in ROS, but are in GUI
        for param_to_delete in params_in_gui:
            print(f"deleting parameter {param_to_delete}")
            dpg.delete_item(param_to_delete)

    def get_gui_parameters(self):
        ret = []
        for it in dpg.get_all_items():
            if dpg.get_item_configuration(it)['user_data'] is not None:
                ret.append(dpg.get_item_configuration(it)['user_data']['name'])
        return ret

    def setparam_callback(self, parameter_name, parameter_value, user_data):
        param_type = user_data["type"]
        
        if not self.has_parameter(parameter_name):
            self.declare_parameter(parameter_name, parameter_value)  # Declare the parameter
        
        try:
            self.set_parameters([rclpy.parameter.Parameter(parameter_name, param_type, parameter_value)])
            self.get_logger().info(f"Parameter {parameter_name} set to {parameter_value}")
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f"Failed to set parameter {parameter_name}: {e}")



def main():
    rclpy.init()

    dpg.create_context()
    dpg.create_viewport(title="Dynamic Reconfigure", width=300, height=300)
    dpg.setup_dearpygui()
    node = DynamicReconfigure()
    dpg.show_viewport()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def spin_executor():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spinning_thread = threading.Thread(target=spin_executor, daemon=True)
    spinning_thread.start()

    i = 0
    while dpg.is_dearpygui_running():
        i += 1
        dpg.render_dearpygui_frame()
        if i % UPDATE_TAU == 0:
            node.update_ros_params()
    dpg.destroy_context()

if __name__ == "__main__":
    main()