import rclpy
from rclpy import Node
from rclpy.client import Client


def create_service_client(node: Node, srv_type, srv_name:str):
    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(srv_name + ' service not available, waiting to try again...')
    node.get_logger().info(srv_name + ' service found.')
    return client

def call_and_wait_for_service_response(node: Node, client: Client, data, timeout_sec=1.0, timeout_exception=exceptions.SpinFutureTimeoutException):
    future = client.call_async(data)
    future.set_exception(timeout_exception)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result()
