from robot import Robot

leader = Robot(device_name='/dev/tty.usbmodem57380045631')
follower = Robot(device_name='/dev/tty.usbmodem57380047071')
leader.set_trigger_torque()


while True:
    follower.set_goal_pos(leader.read_position())