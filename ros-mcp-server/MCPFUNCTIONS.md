# MCP Functions

This is a list of functions that can be used in the ROS MCP Server.

## get_topics
- **Purpose**: Retrieves the list of available topics from the robot's ROS system.
- **Returns**: List of topics (List[Any])

## pub_twist
- **Purpose**: Sends movement commands to the robot by setting linear and angular velocities.
- **Parameters**:
  - `linear`: Linear velocity (List[Any])
  - `angular`: Angular velocity (List[Any])

## pub_twist_seq
- **Purpose**: Sends a sequence of movement commands to the robot, allowing for multi-step motion control.
- **Parameters**:
  - `linear`: List of linear velocities (List[Any])
  - `angular`: List of angular velocities (List[Any])
  - `duration`: List of durations for each step (List[Any])
 
## sub_image
- **Purpose**: Receive images from the robot's point of view or of the surrounding environment.
- **Parameters**:
  - `save_path`: By default, the image is saved to the ``Downloads`` folder.

## pub_jointstate
- **Purpose**: Publishes a custom JointState message to the `/joint_states` topic.
- **Parameters**:
  - `name`: List of joint names (list[str])
  - `position`: List of joint positions (list[float])
  - `velocity`: List of joint velocities (list[float])
  - `effort`: List of joint efforts (list[float])

## sub_jointstate
- **Purpose**: Subscribes to the `/joint_states` topic and returns the latest JointState message as a formatted JSON string.
- **Returns**: JointState message (str)
