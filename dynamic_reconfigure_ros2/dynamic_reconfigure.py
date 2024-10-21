
from typing import List, Tuple, Any
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import dearpygui.dearpygui as dpg
import threading
from rclpy.executors import MultiThreadedExecutor

from dynamic_reconfigure_ros2.map_rostype_to_widget import create_widget
from dynamic_reconfigure_ros2.ros_param_manager import list_remote_parameters, get_remote_parameters, extract_param_value, set_remote_parameters, list_all_rosnodes

UPDATE_PERIOD_FRAMES = 20
DIV_CHAR = "__"
WIDTH = 600
HEIGHT = 600
WINDOW_HEIGHT = 200

class DynamicReconfigure(Node):
    def __init__(self):
        super().__init__("dynamic_reconfigure")
        self.parameter_set_requests = []
        self.node_windows = {}
        self.create_base_gui()
        self.update_ros_params()

    def create_base_gui(self):
        dpg.create_context()
        dpg.create_viewport(title="Dynamic Reconfigure", width=WIDTH, height=WIDTH)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def create_window_for_node(self, node_name):
        """Creates a new window for a ROS node dynamically."""
        if node_name not in self.node_windows:
            window_tag = f"{node_name}_window"
            group_tag = f"{node_name}_group"
            with dpg.window(tag=window_tag, label=node_name, width=WIDTH, height=WINDOW_HEIGHT):
                self.node_windows[node_name] = dpg.add_group(tag=group_tag)
            dpg.set_item_pos(window_tag, [0, (len(self.node_windows)-1) * WINDOW_HEIGHT])  # Stacks windows

    def remove_window_for_node(self, node_name):
        """Removes the window for the specified ROS node."""
        if node_name in self.node_windows:
            dpg.delete_item(f"{node_name}_window")
            del self.node_windows[node_name]

    def update_ros_params(self):
        param_topic_in_gui = self.get_gui_param_names()
        params_ros = self.get_ros_params()
        if params_ros is None:
            self.get_logger().info('Warning: No ROS topic online (Parameter Service is not responding)')
            return

        nodes_in_ros = set([p_server for p_server, _, _, _ in params_ros])

        # Create windows for new nodes
        for node_name in nodes_in_ros:
            self.create_window_for_node(node_name)
        # Remove windows for nodes that are no longer available
        nodes_in_gui = set(self.node_windows.keys())
        for node_name in nodes_in_gui - nodes_in_ros:
            self.remove_window_for_node(node_name)

        for p_server, p_name, p_value, p_type in params_ros:
            group_tag = f"{p_server}_group"
            widget_tag = p_server + DIV_CHAR + p_name
            
            if widget_tag in param_topic_in_gui:
                assert dpg.get_item_configuration(widget_tag)['user_data']['type'] == p_type, \
                    "WARN: The parameter found in ROS has different type than previously loaded parameter"
                param_topic_in_gui.remove(widget_tag)
                # print(f"Externally set {p_name} to {p_value}")
                dpg.configure_item(widget_tag, default_value=p_value)
            else:  # Create new param
                create_widget(
                        type=p_type,
                        label=widget_tag,
                        tag=widget_tag,
                        callback=self.set_ros_param_callback,
                        user_data={"name": p_name, "type": p_type, "server": p_server},
                        default_value=p_value,
                        parent=group_tag,  # Add to the group
                    )

        # Delete parameters that are no longer declared in ROS, but are in GUI
        for topic_to_delete in param_topic_in_gui:
            dpg.delete_item(topic_to_delete)

    def get_gui_param_names(self) -> List[str]:
        """ Returns List of parameter names. """
        ret = []
        for it in dpg.get_all_items():
            if dpg.get_item_configuration(it)['user_data'] is not None:
                user_data = dpg.get_item_configuration(it)['user_data']
                ret.append(user_data['server']+DIV_CHAR+user_data['name'])
        return ret

    def set_ros_param_callback(self, parameter_name, parameter_value, user_data):
        self.set_ros_param(user_data["name"], parameter_value, user_data["type"], user_data["server"])

    def set_ros_param(self, name: str, value: Any, type: Parameter.Type, server: str):
        """ Value type is based on type (Parameter.Type) """        
        try:
            print(f"SET REMOTE PARAM, {name} {value} {type} {server}")
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
    time.sleep(10)
    
    # rosnode.set_ros_param(name = "/test", value = 10, type = Parameter.Type.INTEGER, server="parameter_tester")
    # rosnode.set_ros_param(name = "/parint1", value = 2024, type = Parameter.Type.INTEGER, server="dynamic_reconfigure")
    # rosnode.set_ros_param(name = "/test2", value = 2025.2, type = Parameter.Type.DOUBLE , server="dynamic_reconfigure")

def main():
    rclpy.init()

    
    node = DynamicReconfigure()
    

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