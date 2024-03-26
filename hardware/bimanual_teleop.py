from robot import Robot
from dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbmodem57380045631').instantiate()
follower1_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbserial-FT8ISNO8').instantiate()
follower2_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbserial-FT8J0SDW').instantiate()

leader1 = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
leader2 = Robot(leader_dynamixel, servo_ids=[10, 11, 12, 13, 15, 16])
follower1 = Robot(follower1_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
follower2 = Robot(follower2_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])

leader1.set_trigger_torque()
leader2.set_trigger_torque()


while True:
    leader1_pos = leader1.read_position()
    follower1.set_goal_pos(leader1_pos)
    follower2.set_goal_pos(leader2.read_position())
