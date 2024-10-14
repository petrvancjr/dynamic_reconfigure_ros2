import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

def list_remote_parameters(node):
    # Create a client for the 'list_parameters' service
    client = node.create_client(ListParameters, '/parameter_tester/list_parameters')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for list parameters service...')

    request = ListParameters.Request()
    # Optionally, specify a prefix, this list all parameters by default
    request.prefixes = []
    request.depth = 0  # Depth=0 means all parameters

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()

def get_remote_parameters(node, param_names):
    # Create a client for the 'get_parameters' service
    client = node.create_client(GetParameters, '/parameter_tester/get_parameters')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for get parameters service...')

    request = GetParameters.Request()
    request.names = param_names  # Pass the list of parameter names

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()

def extract_param_value(pvalue):
    param_type = pvalue.type

    # Check the type and extract the corresponding value
    if param_type == ParameterType.PARAMETER_BOOL:
        return pvalue.bool_value, Parameter.Type.BOOL
    elif param_type == ParameterType.PARAMETER_INTEGER:
        return pvalue.integer_value, Parameter.Type.INTEGER
    elif param_type == ParameterType.PARAMETER_DOUBLE:
        return pvalue.double_value, Parameter.Type.DOUBLE
    elif param_type == ParameterType.PARAMETER_STRING:
        return pvalue.string_value, Parameter.Type.STRING
    elif param_type == ParameterType.PARAMETER_BYTE_ARRAY:
        return pvalue.byte_array_value, Parameter.Type.BYTE_ARRAY
    elif param_type == ParameterType.PARAMETER_BOOL_ARRAY:
        return pvalue.bool_array_value, Parameter.Type.BOOL_ARRAY
    elif param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return pvalue.integer_array_value, Parameter.Type.INTEGER_ARRAY
    elif param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return pvalue.double_array_value, Parameter.Type.DOUBLE_ARRAY
    elif param_type == ParameterType.PARAMETER_STRING_ARRAY:
        return pvalue.string_array_value, Parameter.Type.STRING_ARRAY
    else:
        return None  # Unknown type or unsupported type

def get_parameters(node):
    param_list = []
    list_result = list_remote_parameters(node)
    if list_result:
        param_names = list_result.result.names
        if param_names:
            get_result = get_remote_parameters(node, param_names)
            if get_result:
                for i in range(len(get_result.values)):
                    value, type = extract_param_value(get_result.values[i])
                    param_list.append((param_names[i], value, type))
    return param_list
