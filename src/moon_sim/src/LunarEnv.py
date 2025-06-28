#!/home/tanay/anaconda3/envs/LunarRL/bin/python 
import gym
import gym.spaces.box
import rclpy
from rclpy.node import Node

import gym
import numpy as np
import time
import subprocess, os

from gz.transport13 import Node as GNode
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_reset_pb2 import WorldReset
class LunarEnv(gym.Env):
    def __init__(self):
        super().__init__()

        #OBSPACE = [Height, Velocity, C1, C2, C3, C4]

        self.observation_space = gym.spaces.Box(low=np.array([-2, -9999, 0, 0, 0, 0]), high=np.array([9999, 9999, 1, 1, 1, 1]), dtype=np.float64) 
        self.action_space = gym.spaces.Box(low=np.array([0.0]), high=np.array([1.0]), dtype=np.float64)

        self.gui_controller = GNode()
        self.service_name = "/world/moon_flat_world/control"
        self.request = WorldControl()



        self.create_env()
        
    def create_env(self):
        #process = subprocess.Popen(["ros2", "launch", "moon_sim", "moon_world.launch.py"], text=True)
        process = subprocess.Popen(["gz", "sim", "/home/tanay/lander_simulator/src/moon_sim/worlds/moon_flat_world.sdf"], text=True)
        time.sleep(10)
        self.unpause()

    def unpause(self):
        timeout = 5000  # milliseconds
        self.request.pause = False
        # Perform the service request
        success, response = self.gui_controller.request(
            self.service_name, self.request, WorldControl, Boolean, timeout
        )
        if success:
            print("Success Sent:", success)
            print("Gazebo Paused:", response.data)   

    def pause(self):
        timeout = 5000  # milliseconds
        self.request.pause = True
        # Perform the service request
        success, response = self.gui_controller.request(
            self.service_name, self.request, WorldControl, Boolean, timeout
        )
        if success:
            print("Success Sent:", success)
            print("Gazebo Paused:", response.data) 

    def reset(self):
        reset_msg = WorldReset()
        reset_msg.all = True  # Reset everything (poses, velocities, etc.)

            # Set it into the WorldControl message
        self.request.reset.CopyFrom(reset_msg)

        timeout = 5000  # milliseconds

            # Call the service
        success, response = self.gui_controller.request(
            self.service_name, self.request, WorldControl, Boolean, timeout
        )



env = LunarEnv()

time.sleep(40)

print("Resetting the environment...")
env.reset()