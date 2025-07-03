#!/home/tanay/anaconda3/envs/LunarRL/bin/python 
import gym
import gym.spaces.box


import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


from alti_reader import AltimeterReader
from imu_reader import ImuReader
from contact_sensors import ContactSensor
from WorldManager import WorldManager

import numpy as np
import time
import threading
import subprocess, os

from gz.transport13 import Node as GNode
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.pose_pb2 import Pose
class LunarEnv(gym.Env):
    def __init__(self):
        super().__init__()

        #OBSPACE = [Height, Velocity, no. of contacts, Orientation.X, Orientation.Y, Orientation.Z]

        self.observation_space = gym.spaces.Box(low=np.array([-2, -9999, 0, -9999, -9999, -9999]), 
                                                high=np.array([9999, 9999, 4, 9999, 9999, 9999]), 
                                                dtype=np.float64) 
        
        self.action_space = gym.spaces.Box(low=np.array([0.0]), high=np.array([1.0]), dtype=np.float64)

        self.gui_controller = GNode()
        self.control_service = "/world/moon_flat_world/control"
        self.request = WorldControl()

        self.thrust_publisher = GNode()
        self.thrust_service = "/cmd_thrust"
        self.publisher = self.thrust_publisher.advertise(self.thrust_service, Actuators)

        rclpy.init()
        self.altimeter_reader = AltimeterReader()
        self.imu_reader = ImuReader()
        self.control_sensor = ContactSensor()
        self.world_manager = WorldManager()
        
        self.step_node = GNode()

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.altimeter_reader)
        self.executor.add_node(self.imu_reader)
        # self.executor.add_node(self.control_sensor)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        print("[INFO] ROS 2 nodes spinning in background")


        self.create_env()
        
    def create_env(self):
        process = subprocess.Popen(["ros2", "launch", "moon_sim", "moon_world.launch.py"], text=True)
        #process = subprocess.Popen(["gz", "sim", "/home/tanay/lander_simulator/src/moon_sim/worlds/moon_flat_world.sdf"], text=True)
        time.sleep(10)
        self.world_manager.spawn_model()
        print("Environment Created")
        # self.unpause()

    def unpause(self):
        timeout = 5000  
        self.request.pause = False
        success, response = self.gui_controller.request(
            self.control_service, self.request, WorldControl, Boolean, timeout
        )
        if success:
            print("Success Sent:", success)
            print("Gazebo Paused:", response.data)   

    def pause(self):
        timeout = 5000  
        self.request.pause = True
        success, response = self.gui_controller.request(
            self.control_service, self.request, WorldControl, Boolean, timeout
        )
        if success:
            print("Success Sent:", success)
            print("Gazebo Paused:", response.data) 

    def reset(self):
        # reset_msg = WorldReset()
        # reset_msg.all = True  
        # self.request.reset.CopyFrom(reset_msg)
        # timeout = 5000   
        # success, response = self.gui_controller.request(
        #     self.control_service, self.request, WorldControl, Boolean, timeout
        # )

        self.world_manager.destroy_model()
        time.sleep(2)
        self.world_manager.spawn_model()
        time.sleep(1)

    def apply_thrust(self):
        msg = Actuators()
        msg.normalized.append(1.0)  
        time.sleep(1)
        self.publisher.publish(msg)
        print("Published:", msg)

    def get_observation(self):
        # print("getting imu")
        # rclpy.spin_once(self.imu_reader, timeout_sec=0.1)
        # print("altimeter reading")
        # rclpy.spin_once(self.altimeter_reader, timeout_sec=0.1)
        height = self.altimeter_reader.vertical_position + 50
        velocity = self.altimeter_reader.vertical_velocity
        orientation_z = self.imu_reader.orientation_z
        orientation_y = self.imu_reader.orientation_y
        orientation_x = self.imu_reader.orientation_x
        contacts = self.control_sensor.get_total_contacts()
        observation = np.array([height, velocity, contacts, orientation_x, orientation_y, orientation_z], dtype=np.float64)
        self.observation_space.contains(observation)
        return observation

    def step(self):
        observation = self.get_observation()

        if observation[0] < 2:
            print("Resetting . . .")
            self.pause()
            time.sleep(0.1)
            self.reset()

          
            for _ in range(5): 
                ctrl_msg = WorldControl()
                ctrl_msg.step = True
                self.step_node.request(self.control_service, ctrl_msg, WorldControl, Boolean, 1000)
                time.sleep(0.05)  
          
            observation = self.get_observation()

            if observation[0] < 2:
                print("still in contact")
                return

        ctrl_msg = WorldControl()
        ctrl_msg.step = True
        # ctrl_msg.pause = True
        self.step_node.request(
            self.control_service,
            ctrl_msg,
            WorldControl,
            Boolean,
            1000
        )





env = LunarEnv()

try:
    while True:
        print("Printing ...")
        env.step()
        print(env.get_observation())
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[INFO] KeyboardInterrupt received. Shutting down cleanly...")
    env.executor.shutdown()
    rclpy.shutdown()
    print("[INFO] Shutdown complete.")
