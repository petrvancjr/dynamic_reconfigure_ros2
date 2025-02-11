
from rclpy.parameter import Parameter
import dearpygui.dearpygui as dpg

SLIDER_MIN_VALUE = 0
SLIDER_MAX_VALUE = 100
SLIDER_WIDTH = 100 # px

mapping_rostype_to_widget = {
    Parameter.Type.BOOL: dpg.add_checkbox,
    Parameter.Type.DOUBLE: dpg.add_slider_float,
    Parameter.Type.INTEGER: dpg.add_slider_int,
}

def get_widget(p_type):
    return mapping_rostype_to_widget[p_type]

def create_widget(
        type,
        label,
        tag,
        callback,
        user_data,
        default_value,
        parent,
    ):
    widget = get_widget(type)

    if type == Parameter.Type.BOOL: # BOOL = Checkbox: Don't specify width
        widget(
            label = label,
            tag = tag,
            callback = callback,
            user_data = user_data,
            default_value = default_value,
            parent = parent
        )
    else:
        widget(
            label = label,
            tag = tag,
            callback = callback,
            user_data = user_data,
            default_value = default_value,
            parent = parent,
            width = SLIDER_WIDTH,
            min_value = SLIDER_MIN_VALUE,
            max_value = SLIDER_MAX_VALUE,
        )