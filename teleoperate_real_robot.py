from robot import Robot
from dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbmodem57380045631').instantiate()
follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbserial-FT8ISNO8').instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
leader.set_trigger_torque()


while True:
    follower.set_goal_pos(leader.read_position())