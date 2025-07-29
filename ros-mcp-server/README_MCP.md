## Overview
![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

![flow diagram](<img/framework.png>)


The ROS MCP Server is designed to support robots in performing complex tasks and adapting effectively to various environments by providing a set of functions that transform natural language commands, entered by a user through an LLM, into ROS commands for robot control. Furthermore, by utilizing ``rosbridge``, it is configured to operate with both ``ROS`` and ``ROS2`` systems, and its WebSocket-based communication enables broad applicability across diverse platforms.

## Supported Types

- geometry_msgs/Twist
- sensor_msgs/Image
- sensor_msgs/JointState



## Installation

### `uv` Installation
- To install `uv`, you can use the following command:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
or
```bash
pip install uv
```

- Create virtual environment and activate it (Optional)
```bash
uv venv
source .venv/bin/activate
```

### MCP Server Configuration


Install Claude Desktop (For Linux installation follow [claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)) and set MCP setting to mcp.json.

```bash
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/ABSOLUTE/PATH/TO/Go2_Isaac_ros2/ros-mcp-server",,
        "run",
        "server.py"
      ]
    }
  }
}
```

You can find mcp.json using the following command:

- MacOS
```bash
code ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

- Linux(Ubuntu)
  
```bash
code ~/.config/Claude/claude_desktop_config.json
```

- Windows
```bash
code $env:AppData\Claude\claude_desktop_config.json
```

## MCP Functions

You can find the list of functions in the [MCPFUNCTIONS.md](MCPFUNCTIONS.md).

## How To Use
### 1. Set IP and Port to connect rosbridge.
- Open `server.py` and change your `LOCAL_IP`, `ROSBRIDGE_IP` and `ROSBRIDGE_PORT`. (`ROSBRIDGE_PORT`'s default value is `9090`)

### 2. Run rosbridge server.
ROS 1
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
ROS 2
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

![rosbridge server launch](<img/rosbridge.png>)

### 3. Run ```main.py``` 
Before running, make sure ```num_envs``` in [sim.yaml](<../cfg/sim.yaml>) has same value as ```NUM_ENVS``` in [server.py](<server.py>)


### 4. Launch Claude Desktop

Confirm mcp server is running correctly


![Claude desktop mcp server](<img/mcp_server.png>)


### 5. Type your instructions in chat

![How to use](<img/run_command.png>)


