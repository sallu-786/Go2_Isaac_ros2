AZURE_API_BASE = ""
AZURE_OPENAI_DEPLOYMENT = ""  
AZURE_API_KEY = ""
AZURE_API_VERSION=""



SHOW_CAMERA=True
SHOW_TOPICS=True
SHOW_SERVICES=True
SHOW_CONTROLLER=True
SHOW_DESCRIPTION=True

# ROS bridge connection settings
ROSBRIDGE_IP = "127.0.0.1"  # Default is localhost. Replace with your IP of device hosting rosbridge service if different.
ROSBRIDGE_PORT = 9090  # Rosbridge default is 9090. 


CAMERA_TOPIC_NAME="/spot_0/front_cam/color_image"    #for go2 replace spot_0 wit unitree_go2_0 where 0 represent index of robot
CMD_VEL_PUB_TOPIC_NAME="/spot_0/cmd_vel"


#Topics and Services Graph
TOPIC_COLOR="orange"
PUBLISHER_COLOR="lightblue"
SUBSCRIBER_COLOR="lightgreen"
NODE_SIZE=2000
TOPIC_SIZE=1000
PLOT_WIDTH=16
PLOT_HEIGHT=12
