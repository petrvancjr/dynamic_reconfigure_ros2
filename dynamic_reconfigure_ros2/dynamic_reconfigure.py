
from typing import List, Tuple, Any
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import dearpygui.dearpygui as dpg
import threading
from rclpy.executors import MultiThreadedExecutor

from map_rostype_to_widget import get_widget
from ros_param_manager import list_remote_parameters, get_remote_parameters, extract_param_value, set_remote_parameters, list_all_rosnodes

UPDATE_PERIOD_FRAMES = 20

class DynamicReconfigure(Node):
    def __init__(self):
        super().__init__("dynamic_reconfigure")


        self.parameter_set_requests = []
        self.dynamic_reconfigure_group = None  # A group to hold the parameter widgets
        self.tester_group = None  # A group to hold the parameter widgets
        self.create_gui()
        self.update_ros_params()

    def create_gui(self):
        with dpg.window(label="Dynamic Reconfigure", width=300, height=300):
            # Create a group to hold all parameter widgets, so we can update them
            self.dynamic_reconfigure_group = dpg.add_group(tag="dynamic_reconfigure_group")
            self.tester_group = dpg.add_group(tag="tester_group")

    def update_ros_params(self):
        param_names_in_gui = self.get_gui_param_names()
        params_ros = self.get_ros_params()
        print("====S====")
        print(params_ros)
        print("====E====")
        if params_ros is None:
            self.get_logger().info('Warning: No ROS topic online (Parameter Service is not responding)')
            return

        # Update parameters inside the group by adding/removing widgets
        for p_server, p_name, p_value, p_type in params_ros:
            if p_server == "dynamic_reconfigure":
                parent = self.dynamic_reconfigure_group
            elif p_server == "parameter_tester":
                parent = self.tester_group
            else: raise Exception()
            
            if p_name in param_names_in_gui:
                assert dpg.get_item_configuration(p_server+p_name)['user_data']['type'] == p_type, \
                    "WARN: The parameter found in ROS has different type than previously loaded parameter"
                param_names_in_gui.remove(p_name)
                print(f"Externally set {p_name} to {p_value}")
                dpg.configure_item(p_server+p_name, default_value=p_value)
                continue
            else:  # Create new param
                add_widget = get_widget(p_type)
                print("==addwidget==")
                print(p_server, p_name, p_value, p_type)
                print("=============")
                add_widget(
                        label=p_server+p_name,
                        tag=p_server+p_name,
                        callback=self.set_ros_param_callback,
                        user_data={"name": p_name, "type": p_type, "server": p_server},
                        default_value=p_value,
                        parent=parent  # Add to the group
                    )

        # Delete parameters that are no longer declared in ROS, but are in GUI
        for param_to_delete in param_names_in_gui:
            dpg.delete_item(p_server+param_to_delete)

    def get_gui_param_names(self) -> List[str]:
        """ Returns List of parameter names. """
        ret = []
        for it in dpg.get_all_items():
            if dpg.get_item_configuration(it)['user_data'] is not None:
                ret.append(dpg.get_item_configuration(it)['user_data']['name'])
        return ret

    def set_ros_param_callback(self, parameter_name, parameter_value, user_data):
        self.set_ros_param(parameter_name, parameter_value, user_data["type"], user_data["server"])

    def set_ros_param(self, name: str, value: Any, type: Parameter.Type, server: str):
        """ Value type is based on type (Parameter.Type) """        
        if not self.has_parameter(name):
            self.declare_parameter(name, value)  # Declare the parameter
        
        try:
            set_remote_parameters(self, name, value, type, server)
            self.get_logger().info(f"Parameter {name} set to {value}")
        except:
            self.get_logger().error(f"Failed to set parameter {name}")

    def get_ros_params(self) -> List[Tuple[str, str, Any, Parameter.Type]]:
        param_list = []
        for server in list_all_rosnodes(self):
            r = self.get_ros_params_for_server(server)
            if r is not None:
                for i in range(len(r)):
                    param_list.append((server, r[i][0], r[i][1], r[i][2])) # server, name, value, type
        return param_list

    def get_ros_params_for_server(self, server) -> List[Tuple[str, Any, Parameter.Type]]:
        """Returns List of Triplets. Each triplet has (name, value, parameter type).
        """
        list_result = list_remote_parameters(self, server=server)
        if list_result is None or len(list_result.result.names) == 0: 
            return None # No client online
        
        param_names = list_result.result.names
        get_result = get_remote_parameters(self, param_names, server=server)
        if get_result is None:
            return None # No client online
        
        param_list = []
        for i in range(len(get_result.values)):
            value, type = extract_param_value(get_result.values[i])
            param_list.append((param_names[i], value, type))
        return param_list

import time
def test(rosnode):
    time.sleep(5)
    rosnode.set_ros_param(name = "/parint1", value = 2024, type = Parameter.Type.INTEGER)
    rosnode.set_ros_param(name = "/test2", value = 2025.2, type = Parameter.Type.DOUBLE)


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

    spinning_thread = threading.Thread(target=test, args=(node, ), daemon=True)
    spinning_thread.start()


    i = 0
    while dpg.is_dearpygui_running():
        i += 1
        dpg.render_dearpygui_frame()
        if i % UPDATE_PERIOD_FRAMES == 0:
            node.update_ros_params()
    dpg.destroy_context()

if __name__ == "__main__":
    main()