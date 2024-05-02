from robot import Robot
from dynamixel import Dynamixel
from vr_controller import VRController
import numpy as np
import threading
import time

follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM0').instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5])
        
leader = VRController()
leader.start()
while True:
    target_pos = ((np.array(leader.read_position())/3.14)*2048 + 2048).astype(int)
    target_pos[1] = 2*2048-target_pos[1]
    target_pos[3] = 2*2048-target_pos[3]
    target_pos[4] = 2*2048-target_pos[4]
    
    follower.set_goal_pos(target_pos)