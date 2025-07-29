from mcp.server.fastmcp import FastMCP
from typing import List, Any
from pathlib import Path
import argparse
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState


# -------------------- ROS Config --------------------
NUM_ENVS = 2  
LOCAL_IP = "10.56.234.213"      #set your local IP address here
ROSBRIDGE_IP = "10.56.234.213"  #IP of device running rosbridge
# If you are running this on the same machine use local IP
ROSBRIDGE_PORT = 9090

#-------------------- MCP Init --------------------
mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)

# --------------------Env Setup --------------------
twists = []
images = []
jointstates = []

for i in range(NUM_ENVS):
    prefix = f"unitree_go2_{i}"   
    twists.append(Twist(ws_manager, topic=f"/{prefix}/cmd_vel"))
    images.append(Image(ws_manager, topic=f"/{prefix}/front_cam/color_image"))
    jointstates.append(JointState(ws_manager, topic=f"/{prefix}/joint_states"))

# Use first robot for default tools
twist = twists[0]
image = images[0]
jointstate = jointstates[0]

# -------------------- Tools --------------------
@mcp.tool()
def get_topics():
    topic_info = ws_manager.get_topics()
    if topic_info:
        topics, types = zip(*topic_info)
        return {
            "topics": list(topics),
            "types": list(types)
        }
    else:
        return "No topics found"

@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any], robot_id: int = 0):
    """Publish twist message to specific robot
    Args:
        linear: Linear velocity [x, y, z]
        angular: Angular velocity [x, y, z]
        robot_id: Robot ID (0 to NUM_ENVS-1)
    """
    if not 0 <= robot_id < NUM_ENVS:
        return f"Invalid robot_id"
    msg = twists[robot_id].publish(linear, angular)
    return f"Twist msg published to robot {robot_id}" if msg else "No msg published"

@mcp.tool()
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any], robot_id: int = 0):
    if not 0 <= robot_id < NUM_ENVS:
        return f"Invalid robot_id"
    msg = twists[robot_id].publish_sequence(linear, angular, duration)
    return f"Twist sequence msg published to robot {robot_id}" if msg else "No msg published"

@mcp.tool()
def sub_image(robot_id: int = 0):
    if not 0 <= robot_id < NUM_ENVS:
        return f"Invalid robot_id"
    msg = images[robot_id].subscribe()
    return f"Image received from robot {robot_id}" if msg else "No image received"

@mcp.tool()
def pub_jointstate(name: list[str], position: list[float], velocity: list[float], effort: list[float], robot_id: int = 0):
    if not 0 <= robot_id < NUM_ENVS:
        return f"Invalid robot_id"
    msg = jointstates[robot_id].publish(name, position, velocity, effort)
    return f"JointState msg published to robot {robot_id}" if msg else "No msg published"

@mcp.tool()
def sub_jointstate(robot_id: int = 0):
    if not 0 <= robot_id < NUM_ENVS:
        return f"Invalid robot_id"
    msg = jointstates[robot_id].subscribe()
    return msg if msg else f"No JointState data received from robot {robot_id}"

# -------------------- Run MCP --------------------
if __name__ == "__main__":
    mcp.run(transport="stdio")
