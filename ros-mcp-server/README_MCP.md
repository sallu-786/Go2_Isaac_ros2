## Overview
![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-MIT-blue)

![flow diagram](<img/framework.png>)


The ROS MCP Server with Web Portal is designed to support robots in performing complex tasks and adapting effectively to various environments by providing a set of functions that transform natural language commands, entered by a user through an LLM, into ROS commands for robot control. Furthermore, by utilizing ``rosbridge``, it is configured to operate with both ``ROS`` and ``ROS2`` systems, and its WebSocket-based communication enables broad applicability across diverse platforms. not only that using this MCP Server I have created a web portal that makes it super easy to go through hundreds of topics and services without typing a single command and visualzie it for easy intuition, see live camera stream wth description from LLM and ability to control robots with Web-UI.

## Supported Types

- geometry_msgs/Twist
- sensor_msgs/Image
- sensor_msgs/JointState



## 1. Installation

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

### 2. MCP Server Configuration


#### Option A. Install Cursor 
In cursor open settings and go to ```Tools & Integrations``` and then click ```Add Custom MCP```. Then paste the code in section 2.1


![cursor](<img/mcp_cursor.jpg>)



#### Option B: 
Install Claude Desktop (For Linux installation follow [claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)). 
Run claude desktop and go to ```developer``` settings and click ```Add MCP Server```. Then paste the code given in section 2.1


![claude](<img/mcp_server.png>)


### 2.1 MCP Config File
Paste the code in mcp.json file
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


## 2.2 MCP Functions

You can find the list of functions in the [MCPFUNCTIONS.md](MCPFUNCTIONS.md).

## 3. Launch Rosbridge Server
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

### 4. Run ```main.py``` as per [README.MD](<https://github.com/sallu-786/Go2_Isaac_ros2/blob/main/README.md>) 
Before running, make sure ```num_envs``` in [sim.yaml](<../cfg/sim.yaml>) has same value as ```NUM_ENVS``` in [server.py](<server.py>)


### 5. Type your instructions in chat

![How to use](<img/run_command.png>)


## 6. Launch Web Portal 
To run web portal ol prerequisite is step 3 (launch robsridge server). Other steps can be ignored
### 6.1 Activate the evironment 
First of all activate the same environment being used for running ```main.py``` program (isaaclab_env) as per [README.MD](<https://github.com/sallu-786/Go2_Isaac_ros2/blob/main/README.md>) 


**Note:** If you want to create a new seperate environment for web portal then you need to install torch and numpy as well otherwise make sure you dont update/change them in existing environment or it will cause problems. 
```bash
conda activate env_isaaclab #or activate your custom env
cd ~/Go2_Isaac_ros2/ros-mcp-server
pip install -r requirements_web_mcp.txt
python web_portal.py
```
![Web_Portal](<img/portal.png>)

### 6.1 Changing Portal Settings 
- You can choose which services to run in portal by setting parameters to True/False in [config.py](<config.py>)
- Button location might be different depending on screen size. You can adjust it in [web_portal.py](<web_portal.py>) from line 90 onwards
- To run the LLM description dont forget to add your API key in [config.py](<config.py>) or use [Ollama](https://ollama.com/download) for local hosting. To change prompt go to [llm_describer.py](<web_utils/llm_describer.py>)
- To add a new service make a new code such as lidar.py and initialize it in [data_stream.py](<web_utils/data_stream.py>). From here all data needed for web_portal is acquired. All data is is gathered in data_stream class whose object is then called by web_portal.
- By default an image is sent to LLM every 5 seconds for description update. You can change it in [data_stream.py](<web_utils/data_stream.py>) 
- If you want to use camera and controller for a different robot make sure you change the name of topics in [config.py](<config.py>)
- To share Web-portal with many users across network, Checkout [Gradio-Mcp-Server](https://www.gradio.app/guides/building-mcp-server-with-gradio)



