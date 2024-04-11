from robot import Robot
from config.config import ROBOT_PORTS

# init robots
leader = Robot(device_name=ROBOT_PORTS['leader'])
follower = Robot(device_name=ROBOT_PORTS['follower'])
# activate the leader gripper torque
leader.set_trigger_torque()

while True:
    follower.set_goal_pos(leader.read_position())