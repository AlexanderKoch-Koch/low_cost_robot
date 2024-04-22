from vr_controller import VRController
import numpy as np
import time
import mujoco.viewer
from simulation.interface import SimulatedRobot
import threading

def read_leader_position():
    global target_pos
    while True:
        target_pos = np.array(leader.read_position())
        
leader = VRController()
leader.start()

m = mujoco.MjModel.from_xml_path('simulation/low_cost_robot/scene.xml')
d = mujoco.MjData(m)

r = SimulatedRobot(m, d)

target_pos = np.zeros(5)

# Start the thread for reading leader position
leader_thread = threading.Thread(target=read_leader_position)
leader_thread.daemon = True
leader_thread.start()

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        # Use the latest target_pos
        step_start = time.time()
        target_pos_local = target_pos.copy()
        # print(f'target pos copy {time.time() - step_start}')
        r.set_target_pos(target_pos_local)
        # print(f'set targtee pos copy {time.time() - step_start}')
        mujoco.mj_step(m, d)
        # print(f'mjstep {time.time() - step_start}')
        viewer.sync()
        # print(f'viewer sync {time.time() - step_start}')

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        # print(f'time until next step {time_until_next_step}')
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
