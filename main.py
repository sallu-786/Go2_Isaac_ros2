import os
import hydra
import rclpy
import torch
import time
import math
import argparse
import omni
from isaaclab.app import AppLauncher

# add CLI arguments
parser = argparse.ArgumentParser(description="Running the Quadruped RL environment.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest follows."""

import carb    # must come after app launcher
import quadruped.go2.go2_ctrl as go2_ctrl
import quadruped.spot.spot_ctrl as spot_ctrl


import quadruped.go2.go2_env as go2_env
import quadruped.spot.spot_env as spot_env


import quadruped.go2.go2_sensors as go2_sensors
import quadruped.spot.spot_sensors as spot_sensors


import ros2.go2_ros2_bridge as go2_ros2_bridge
import ros2.spot_ros2_bridge as spot_ros2_bridge



from env.sim_env import create_hospital_env, create_obstacle_env, \
    create_warehouse_env, create_office_env, create_rivermark_env, create_terrain_env 


FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)

def main(cfg):

    if (cfg.robot == "go2"):
    # Robot Environment setup
        env_cfg = go2_env.Go2RSLEnvCfg()
        env_cfg.scene.num_envs = cfg.num_envs
        env_cfg.decimation = math.ceil(1./env_cfg.sim.dt/cfg.freq)
        env_cfg.sim.render_interval = env_cfg.decimation
        go2_ctrl.init_base_vel_cmd(cfg.num_envs)
        env, policy = go2_ctrl.get_rsl_flat_policy(env_cfg) 
            # Simulation environment
        if (cfg.env_name == "obstacle"):
            create_obstacle_env() # obstacles

        elif (cfg.env_name == "warehouse"):
            create_warehouse_env() # warehouse

        elif (cfg.env_name == "office"):
            create_office_env() # office
        
        elif (cfg.env_name == "hospital"):
            create_hospital_env() # hospital
        
        elif (cfg.env_name == "rivermark"):
            create_rivermark_env() # rivermark

        elif (cfg.env_name == "terrain"):
            create_terrain_env() # terrain

                # Sensor setup
        sm = go2_sensors.SensorManager(cfg.num_envs)
        lidar_annotators = sm.add_rtx_lidar()
        cameras = sm.add_camera(cfg.freq)

        # Keyboard control
        system_input = carb.input.acquire_input_interface()
        system_input.subscribe_to_keyboard_events(
            omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)
        
        # ROS2 Bridge and YOLO setup
        rclpy.init()
        dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
        # yolo_detector = YOLODetectorNode(cfg.num_envs)  # Create YOLO detector


        # Run simulation
        sim_step_dt = float(env_cfg.sim.dt * env_cfg.decimation)
        obs, _ = env.reset()


        while simulation_app.is_running():
            start_time = time.time()
            with torch.inference_mode():            
                # control joints
                actions = policy(obs)

                # step the environment
                obs, _, _, _ = env.step(actions)

                # ROS2 data publish
                dm.pub_ros2_data()
                rclpy.spin_once(dm,timeout_sec=0.001)
            
                # Camera follow
                if (cfg.camera_follow):
                    go2_env.camera_follow(env)

                # limit loop time
                elapsed_time = time.time() - start_time
                if elapsed_time < sim_step_dt:
                    sleep_duration = sim_step_dt - elapsed_time
                    time.sleep(sleep_duration)





    elif (cfg.robot == "spot"):
        env_cfg = spot_env.SpotRSLEnvCfg()
        env_cfg.scene.num_envs = cfg.num_envs
        env_cfg.decimation = math.ceil(1./env_cfg.sim.dt/cfg.freq)
        env_cfg.sim.render_interval = env_cfg.decimation
        spot_ctrl.init_base_vel_cmd(cfg.num_envs)
        env, policy = spot_ctrl.get_rsl_flat_policy(env_cfg)
    
    # # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    # env, policy = get_rsl_rough_policy(go2_env_cfg)      #-->Run rough terrain policy

        # Simulation environment
        if (cfg.env_name == "obstacle"):
            create_obstacle_env() # obstacles

        elif (cfg.env_name == "warehouse"):
            create_warehouse_env() # warehouse

        elif (cfg.env_name == "office"):
            create_office_env() # office
        
        elif (cfg.env_name == "hospital"):
            create_hospital_env() # hospital
        
        elif (cfg.env_name == "rivermark"):
            create_rivermark_env() # rivermark

        elif (cfg.env_name == "terrain"):
            create_terrain_env() # terrain

    

        # Sensor setup
        sm = spot_sensors.SensorManager(cfg.num_envs)
        lidar_annotators = sm.add_rtx_lidar()
        cameras = sm.add_camera(cfg.freq)

        # Keyboard control
        system_input = carb.input.acquire_input_interface()
        system_input.subscribe_to_keyboard_events(
            omni.appwindow.get_default_app_window().get_keyboard(), spot_ctrl.sub_keyboard_event)
        
        # ROS2 Bridge and YOLO setup
        rclpy.init()
        dm = spot_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
        # yolo_detector = YOLODetectorNode(cfg.num_envs)  # Create YOLO detector


        # Run simulation
        sim_step_dt = float(env_cfg.sim.dt * env_cfg.decimation)
        obs, _ = env.reset()



        while simulation_app.is_running():
            start_time = time.time()
            with torch.inference_mode():            
                # control joints
                actions = policy(obs)

                # step the environment
                obs, _, _, _ = env.step(actions)

                # ROS2 data publish
                dm.pub_ros2_data()
                rclpy.spin_once(dm,timeout_sec=0.001)
            
                # Camera follow
                if (cfg.camera_follow):
                    spot_env.camera_follow(env)

                # limit loop time
                elapsed_time = time.time() - start_time
                if elapsed_time < sim_step_dt:
                    sleep_duration = sim_step_dt - elapsed_time
                    time.sleep(sleep_duration)
                # actual_loop_time = time.time() - start_time
                # rtf = min(1.0, sim_step_dt/elapsed_time)
                # print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f} | YOLO active", end='', flush=True)


    else:
        print("Please select a valid robot in cfg/sim.yaml: go2, spot")
    
    # Cleanup
    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    main()

