# Installation Guide

Follow these steps to set up the ROS MCP Server and connect it with a supported language model client (e.g., Claude Desktop).

---

## 1. Clone the Repository

```bash
git clone https://github.com/r-johnv/ros-mcp-server.git
```

Note the **absolute path** to the cloned directory — you’ll need this later when configuring your language model client.

---

## 2. Install UV (Python Virtual Environment Manager)

You can install [`uv`](https://github.com/astral-sh/uv) using one of the following methods:

### Option A: Shell installer
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Option B: Using pip
```bash
pip install uv
```

---

## 3. Install `rosbridge_server`

This package is required for MCP to interface with ROS or ROS 2 via WebSocket. It needs to be installed on the same machine that is running ROS.

### For ROS 1 (Noetic, Melodic, etc.)
```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
```

### For ROS 2 (Foxy, Humble, etc.)
```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
```

Test by launching the rosbridge server in your ROS environment:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> ⚠️ Don’t forget to `source` your ROS workspace before launching, especially if you're using custom messages or services.

---

## 4. Install a Language Model Client

Any LLM client that supports MCP can be used. We use **Claude Desktop** for testing and development.

- **MacOS / Windows**: Download from [claude.ai](https://claude.ai/download)
- **Linux (Unofficial)**: Use the community-supported [claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)

---

## 5. Configure the Language Model Client (Specific to Claude Desktop)

Locate and edit the `claude_desktop_config.json` file:

- **MacOS**
```bash
code ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

- **Linux (Ubuntu)**
```bash
code ~/.config/Claude/claude_desktop_config.json
```

- **Windows (PowerShell)**
```powershell
code $env:AppData\Claude\claude_desktop_config.json
```

Add the following to the `"mcpServers"` section of the JSON file, replacing `<ABSOLUTE_PATH>` with the path to your `ros-mcp-server` folder:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/<ABSOLUTE_PATH>/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```

> ✅ **Tip:** This configuration also works on **Windows using WSL**, with Claude on Windows and the server on WSL. Make sure to: 
> - Set the **full WSL path** to your `uv` installation (e.g., `/home/youruser/.local/bin/uv`)
> - Use the correct **WSL distribution name** (e.g., `"Ubuntu-22.04"`)
```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "wsl",
      "args": [
        "-d", "Ubuntu-22.04",
        "`/home/youruser/.local/bin/uv",
        "--directory",
        "/<ABSOLUTE_PATH>/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```
---
## 6. Confifure IP (Only not running locally)

- If the MCP server and rosbridge are running on the same machine, skip this step.
- If rosbridge is on another machine, first test IP at runtime using the MCP servers tool to dynamically set the target IP.

Example:
```plaintext
My IP is 100.xx.xx.xx. Connect the ROS MCP server to 100.xx.xx.xx port 9090.
```

You can then edit `server.py` and update the following default values to have the server connected to the right IP at launch:

- `LOCAL_IP` (default: `'localhost'`)
- `ROSBRIDGE_IP` (default: `'localhost'`)
- `ROSBRIDGE_PORT` (default: `9090`)
---
