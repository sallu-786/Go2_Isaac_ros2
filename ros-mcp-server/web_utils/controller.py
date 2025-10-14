
import torch
from rclpy.node import Node
class Controller(Node,):
    def __init__(self, cmd_vel):
        super().__init__('Movement_Controller')
    # --- Velocity Command Tensor ---
        self.base_vel_cmd_input = cmd_vel

    def move_forward(self):
        """Set forward velocity command"""
        self.base_vel_cmd_input[0] = torch.tensor([1.0, 0.0, 0.0])  
        return "Moving Forward"
    
    def move_backward(self):
        """Set backward velocity command"""
        self.base_vel_cmd_input[0] = torch.tensor([-1.0, 0.0, 0.0])  
        return "Moving Backward"

    def move_left(self):
        """Set left velocity command"""
        self.base_vel_cmd_input[0] = torch.tensor([0.0, 1.0, 0.0]) 
        return "Moving Left"
    
    def move_right(self):
        """Set right velocity command"""
        self.base_vel_cmd_input[0] = torch.tensor([0.0, -1.0, 0.0])  
        return "Moving Right"
    
    def rotate_left(self):
        """Set left angular velocity command"""
        self.base_vel_cmd_input[0] = torch.tensor([0.0, 0.0, 1.0])  
        return "Rotating Left"
    
    def rotate_right(self):
        """Set right angular velocity command"""

        self.base_vel_cmd_input[0] = torch.tensor([0.0, 0.0, -1.0])  
        return "Rotating Right"
    
    def stop_robot(self):
        """Stop robot"""
        self.base_vel_cmd_input.zero_()
        return "Stopped"