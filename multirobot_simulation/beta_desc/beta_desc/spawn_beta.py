import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from rclpy.parameter import Parameter
import xml.etree.ElementTree as ET

node_name = "spawn_beta"
default_name = "beta"


def parse_int_float_to_float(parameter: Parameter):
    parameterValue = parameter.get_parameter_value()
    if(parameterValue.type == 2):
        return float(parameterValue.integer_value)
    else:
        return float(parameterValue.double_value)


def main():
    rclpy.init()
    node = rclpy.create_node(node_name)

    node.declare_parameter('name', default_name)
    node.declare_parameter('namespace', default_name)
    node.declare_parameter('xml')
    node.declare_parameter('x', 0)
    node.declare_parameter('y', 0)
    node.declare_parameter('z', 0.1)

    name = node.get_parameter('name').get_parameter_value().string_value
    namespace = node.get_parameter(
        'namespace').get_parameter_value().string_value
    x = parse_int_float_to_float(node.get_parameter('x'))
    y = parse_int_float_to_float(node.get_parameter('y'))
    z = parse_int_float_to_float(node.get_parameter('z'))
    xml = node.get_parameter('xml').get_parameter_value().string_value

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    print('Spawning robot named ' + name + ' with namespace ' +
          namespace + ' at ' + str(x) + ', ' + str(y) + ', ' + str(z))
    request = SpawnEntity.Request()
    request.name = name
    request.xml = xml
    request.robot_namespace = namespace
    request.initial_pose.position.x = x
    request.initial_pose.position.y = y
    request.initial_pose.position.z = z

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
