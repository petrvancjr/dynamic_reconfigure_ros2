
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import dearpygui.dearpygui as dpg


class DynamicReconfigure(Node):
    def __init__(self):
        super().__init__("dynamic_reconfigure")

        # For testing
        self.declare_parameter("par1", 1)
        self.declare_parameter("par2", 2)

        self.parameter_set_requests = []

        dpg.create_context()
        dpg.create_viewport(title="Dynamic Reconfigure", width=300, height=300)
        dpg.setup_dearpygui()

        with dpg.window(label="Dynamic Reconfigure", width=300, height=300):
            for p_name, p_value, p_type in self.get_all_parameters():
                if p_type == Parameter.Type.BOOL:
                    dpg.add_checkbox(
                        label=p_name, 
                        tag=p_name, 
                        callback=self.setparam_callback, 
                        user_data=[p_name, p_type], 
                        default_value=p_value,
                    )
                elif p_type == Parameter.Type.DOUBLE:
                    dpg.add_slider_float(
                        label=p_name, 
                        tag=p_name, 
                        callback=self.setparam_callback, 
                        user_data=[p_name, p_type], 
                        default_value=p_value,
                    )
                elif p_type == Parameter.Type.INTEGER:              
                    dpg.add_slider_int(
                        label=p_name, 
                        tag=p_name, 
                        callback=self.setparam_callback, 
                        user_data=[p_name, p_type], 
                        default_value=p_value,
                    )

        dpg.show_viewport()
        dpg.start_dearpygui()
        dpg.destroy_context()

    def setparam_callback(self, parameter_name, parameter_value, parameter_type):

        self.set_parameters([Parameter(parameter_name, parameter_type[1], parameter_value)])
        print(f"Parameter {parameter_name} set as {parameter_value}")


    def get_all_parameters(self):
        # Retrieve all parameters for this node
        parameters = self.get_parameters_by_prefix('')
        parameters_list = []
        for param_name, param_value in parameters.items():
            parameters_list.append([param_name, param_value.value, param_value.type_])
        return parameters_list

    # self.has_parameter("<parameter name>") # bool
    # self.declare_parameter("<parameter name>", "foo")
    # par = self.get_parameter("<parameter name>")

def main():
    rclpy.init()
    node = DynamicReconfigure()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly shutdown the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()