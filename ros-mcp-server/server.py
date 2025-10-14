from typing import Optional
from mcp.server.fastmcp import FastMCP

from utils.services import list_services, list_services_detail, service_type, service_detail, service_nodes, call_service
from utils.network import ping_robot, connect_to_robot
from utils.websocket_manager import WebSocketManager
from utils.topics import list_topics, topic_type, topic_message, topic_publishers, topic_subscribers, publish_once, subscribe_once, \
      publish_for_duration, subscribe_for_duration, subscribe_for_trigger, echo_topic_once

from config import ROSBRIDGE_IP, ROSBRIDGE_PORT 


# Initialize MCP server and WebSocket manager
mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(
    ROSBRIDGE_IP, ROSBRIDGE_PORT, default_timeout=5.0)  # Increased default timeout for ROS operations


## ################################################################################################
##
##                       ROS TOPICS
##
## ################################################################################################

@mcp.tool(description=("Fetch available topics from the ROS bridge.     Example:get_topics()"))
def get_list_topics() -> dict:
    return list_topics(ws_manager)


@mcp.tool(
    description=("Get the type for a specific topic.    Example: get_topic_type('/cmd_vel')"))
def get_topic_type(topic: str) -> dict:
    return topic_type(ws_manager, topic)

@mcp.tool(
    description=(
        "Get the complete structure/definition of a message type. "
        "Example:get_message_details('geometry_msgs/Twist')"
    )
)
def get_topic_message(message_type: str) -> dict:
    return topic_message(ws_manager, message_type)


@mcp.tool(
    description=(
        "Get list of nodes that are publishing to a specific topic.\n"
        "Example: get_publishers_for_topic('/cmd_vel')"
    )
)
def get_topic_publishers(topic: str) -> dict:
    return topic_publishers(ws_manager, topic)


@mcp.tool(
    description=(
        "Get list of nodes that are subscribed to a specific topic.\n"
        "Example: get_subscribers_for_topic('/cmd_vel')"
    )
)
def get_topic_subscribers(topic: str) -> dict:
    return topic_subscribers(ws_manager, topic)


@mcp.tool(
    description=(
        "Subscribe to a ROS topic and return the first message received."
        "Example:"
        "subscribe_once(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped')"
        "subscribe_once(topic='/slow_topic', msg_type='my_package/SlowMsg', timeout=10.0)  # Specify timeout only if topic publishes infrequently"
        "subscribe_once(topic='/high_rate_topic', msg_type='sensor_msgs/Image', queue_length=5, throttle_rate_ms=100)  # Control message buffering and rate"
    )
)
def get_subscribe_once(
    topic: str = "",
    msg_type: str = "",
    timeout: Optional[float] = None,
    queue_length: Optional[int] = None,
    throttle_rate_ms: Optional[int] = None,
) -> dict:
    return subscribe_once(ws_manager, topic, msg_type, timeout, queue_length, throttle_rate_ms)


@mcp.tool(
    description=(
        "Publish a single message to a ROS topic."
        "Example:"
        "publish_once(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', msg={'linear': {'x': 1.0}})"
    )
)
def get_publish_once(topic: str = "", msg_type: str = "", msg: dict = {}) -> dict:
    return publish_once(ws_manager, topic, msg_type, msg)

@mcp.tool(
    description=(
        "Subscribe to a topic for a duration and collect messages.\n"
        "Example:"
        "subscribe_for_duration(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', duration=5, max_messages=10)"
        "subscribe_for_duration(topic='/high_rate_topic', msg_type='sensor_msgs/Image', duration=10, queue_length=5, throttle_rate_ms=100)  # Control message buffering and rate"
    )
)
def get_subscribe_for_duration(
    topic: str = "",
    msg_type: str = "",
    duration: float = 5.0,
    max_messages: int = 100,
    queue_length: Optional[int] = None,
    throttle_rate_ms: Optional[int] = None,
) -> dict:
    
    return subscribe_for_duration(ws_manager, topic, msg_type, duration,  max_messages, queue_length, throttle_rate_ms)


@mcp.tool(
    description=(
        "Publish a sequence of messages with delays."
        "Example:"
        "publish_for_durations(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', messages=[{'linear': {'x': 1.0}}, {'linear': {'x': 0.0}}], durations=[1, 2])"
    )
)
def get_publish_for_duration(
    topic: str = "", msg_type: str = "", messages: list = [], durations: list = []) -> dict:
    return publish_for_duration (ws_manager, topic, msg_type, messages, durations)


@mcp.tool(
    description=(
        "Echo a single message from a ROS topic once."
        "Example:"
        "echo_topic_once_tool(topic='/odom', msg_type='nav_msgs/msg/Odometry')"
    )
)
def get_echo_topic_once(topic: str = "", msg_type: str = "") -> dict:
    return echo_topic_once(ws_manager, topic, msg_type)



## ############################################################################################## ##
##
##                       ROS SERVICES
##
## ############################################################################################## ##


@mcp.tool(description=("Get list of all available ROS services.\nExample:\nget_services()"))
def get_list_services() -> dict:
    return list_services(ws_manager)

@mcp.tool(
    description=(
        "Get comprehensive information about all services including types and providers."
        "Example: inspect_all_services()"
    )
)
def get_list_services_detail() -> dict:
    return list_services_detail(ws_manager)

@mcp.tool(
    description=(
        "Get the service type for a specific service.\nExample:\nget_service_type('/rosapi/topics')"
    )
)
def get_service_type(service: str) -> dict:
    return service_type(ws_manager, service)


@mcp.tool(
    description=(
        "Get complete service details including request and response structures.\n"
        "Example: get_service_details('my_package/CustomService')"
    )
)
def get_service_detail(service_type: str) -> dict:
    return service_detail(ws_manager, service_type)

@mcp.tool(
    description=(
        "Get list of nodes that provide a specific service.\n"
        "Example: get_service_providers('/rosapi/topics')"
    )
)
def get_service_nodes(service: str) -> dict:
    return service_nodes(ws_manager, service)



@mcp.tool(
    description=(
        "Call a ROS service with specified request data.\n"
        "Example:"
        "call_service('/rosapi/topics', 'rosapi/Topics', {})\n"
        "call_service('/slow_service', 'my_package/SlowService', {}, timeout=10.0)  # Specify timeout only for slow services"
    )
)
def get_call_service(
    service_name: str, service_type: str, request: dict, timeout: Optional[float] = None
) -> dict:
    return call_service(
        ws_manager, service_name, service_type, request, timeout=timeout )


## ############################################################################################## ##
##
##                       NETWORK DIAGNOSTICS
##
## ############################################################################################## ##

@mcp.tool(
    description=(
        "Ping a robot's IP address and check if a specific port is open.\n"
        "A successful ping to the IP but not the port can indicate that ROSbridge is not running.\n"
        "Example: ping_robot(ip='192.168.1.100', port=9090)"
    )
)
def get_ping_robot(ip: str, port: int, ping_timeout: float = 2.0, port_timeout: float = 2.0) -> dict:
    return ping_robot(ip, port, ping_timeout, port_timeout)


@mcp.tool(description=("Connect to a robot by setting IP/port and testing connectivity."))
def get_connect_to_robot(ip: Optional[str] = None, port: Optional[int] = None, ping_timeout: float = 2.0, port_timeout: float = 2.0) -> dict:
    
    return connect_to_robot(
        ws_manager, ip, port, ping_timeout, port_timeout)


@mcp.tool(
    description=(
        "Wait for a trigger on a ROS topic. Returns when a message's field meets the trigger condition.\n"
        "Example:"
        "wait_for_trigger(topic='/turtle1/odom', msg_type='nav_msgs/Odometry', trigger_field='pose.pose.position.x',"
        "trigger_value=5.0, comparison='ge', timeout=60.0)"
    )
)
def get_subscribe_for_trigger(topic: str, msg_type: str, trigger_field: str, trigger_value, comparison: str = "eq", 
                         wait_time: float = 30.0, num_tries: int = 0, max_tries: int = 5, 
                         queue_length: int = None, throttle_rate_ms: int = None) -> dict:
    
    return subscribe_for_trigger(ws_manager, topic, msg_type, trigger_field, trigger_value, 
                            comparison, wait_time, num_tries, max_tries, queue_length, throttle_rate_ms)


if __name__ == "__main__":
    mcp.run(transport="stdio")