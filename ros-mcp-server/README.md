# ROS MCP Server

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/img/framework.png"/>
</p>

The **ROS MCP Server** bridges large language models (LLMs) with robot control, allowing users to issue natural language commands that are translated into ROS/ROS2 instructions. It is compatible with any language model that supports MCP, can run with both **ROS** and **ROS2**, and communicatess via WebSocket for cross-platform compatibility. 

It does not require changes to existing robot code, since it is built on `rosbridge` — making it fast and easy to integrate into any robotic stack.

---
## Highlights

- **Universal compatibility**: Works with both ROS and ROS2 via rosbridge
- **Cross-platform support**: Compatible with Linux, Windows, and macOS
- **AI-ready**: Seamlessly integrates with LLMs and AI systems to drive robot behavior via natural language
- **No ROS node modification required**: Fully interoperable with existing systems

---
## Supported ROS Commands

- **View all running ROS topics and types**
- **View message type details**, including custom messages — no code changes needed
- **Publish and subscribe to any topic**, including custom message types
- **View all active publishers, subscribers, and nodes**
- **View all running ROS services**
- **Call any ROS service**, including custom service types
- **Get and set ROS parameters**
- *(Coming soon)*: Full ROS Action support
- *(Coming soon)*: Permission management to restrict write access to selected topics and services

---
# Examples in action:
**Controlling the MOCA mobile manipulator in the NVIDIA Isaac Sim environment.** In this example, the user inputs commands directly into Claude desktop, which can now use the MCP server to directly control a robot simulated in Isaac Sim.

<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/result.gif" />
</p>


---

# Getting Started
Below are instructions to install and run the MCP server with your robot. The MCP server is agnostic to the version of ROS/ROS2 that you are running and works with any LLM that supports MCP. (Our examples use Claude desktop)

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/img/MCP_topology.png"/>
</p>

## Installation

Follow the [installation guide](docs/installation.md) for full setup instructions on:
- Cloning the ROS-MCP-server repository
- Installing `uv` and `rosbridge`
- Installing `Claude desktop`
- Configuring Claude to connect to the ROS-MCP-server.

---

## Usage
This section walks you through launching rosbridge, connecting to your AI system, and issuing your first natural language command.
### 1. Launch rosbridge server
Execute the below command on the same machine that is running your ROS processes. Source your ROS workspace first in order to have the MCP server access custom message and service types. 
#### ROS 1:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

#### ROS 2:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

### 2. Run any AI system with the `ros-mcp-server` installed

This could be your LLM wrapper, chatbot, or any custom client. We’ve used Claude Desktop (an LLM interface) for the examples shown below, but any AI system that supports MCP will work.

---


### 3. You're ready to go!
You can test out your server with any robot that you have running.

✅ **Tip:** If you don't currently have any robots running, turtlesim is the easiest ROS robot to experiment. It does not have any simulation depenencies such as Gazebo or IsaacSim. You can launch turtlesim in ROS with the below command
```
rosrun turtlesim turtlesim_node

```
You can spawn additional turtles with
```
rosservice call /spawn 5.0 5.0 0.0 "turtle2"
```

## Example Commands

### - Natural language commands

Example:
```plaintext
Make the robot move forward.
```

<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_1.png" width="500"/>
</p>

---

### - Query your ROS system
Example:  
```plaintext
Do you see any current velocity commands to the robot?
```
<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_3.png" />
</p>


<!-- ### rosbridge server logs:
If you watch the logs on rosbridge, you should see the MCP server connect every time you give it a command through the LLM.
<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_2.png" />
</p>
--- -->


## Contributing

Contributions are welcome!  
Whether you're fixing a typo, adding a feature, or suggesting an improvement — thank you.  

Please see the [contributing guidelines](docs/contributing.md) to get started.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).
