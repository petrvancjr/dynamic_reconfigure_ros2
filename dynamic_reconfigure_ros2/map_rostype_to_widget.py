
from rclpy.parameter import Parameter
import dearpygui.dearpygui as dpg

mapping_rostype_to_widget = {
    Parameter.Type.BOOL: dpg.add_checkbox,
    Parameter.Type.DOUBLE: dpg.add_slider_float,
    Parameter.Type.INTEGER: dpg.add_slider_int,

}

def get_widget(p_type):
    return mapping_rostype_to_widget[p_type]
