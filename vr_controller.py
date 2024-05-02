import openvr
import numpy as np
import math
import threading
import time

class VRController:
    def __init__(self, L1=0.0239, L2=0.1144, L3=0.0469+0.0207):
        self.scale = 5
        self.L1 = L1*self.scale  # Length of the first arm segment
        self.L2 = L2*self.scale  # Length of the second arm segment
        self.L3 = L3*self.scale  # Length of the third arm segment
        self.x_pos = 0.0  # Initial x position
        self.y_pos = 0.0  # Initial y position
        self.z_pos = 0.0  # Initial z position
        self.base_angle = 0.0  # Initial base angle
        self.shoulder_angle = 0.0  # Initial shoulder angle
        self.elbow_angle = 0.0  # Initial elbow angle
        self.wrist_angle = 0.0  # Initial wrist angle
        self.gripper_angle = 0.0  # Initial gripper angle
        self.min_angle = -np.pi/2
        self.max_angle = np.pi/2

    def start(self):
        openvr.init(openvr.VRApplication_Other)
        print("OpenVR initialized")
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        openvr.shutdown()
        print("OpenVR shutdown")

    def get_controller_poses(self):
        poses = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0.0, openvr.k_unMaxTrackedDeviceCount)
        return poses

    # Function to calculate joint angles
    def calculate_joint_angles(self, x, y, z):
        q1 = np.arctan2(y, x)
        x_prime = np.sqrt(x**2 + y**2)
        z_prime = -z + self.L1
        r = np.sqrt(x_prime**2 + z_prime**2)
        # Check if the point is reachable
        if r > (self.L1 + self.L3) or r < abs(self.L1 - self.L3):
            return None  # Target is not reachable, return None
        cos_q3 = (r**2 - self.L1**2 - self.L3**2) / (2 * self.L1 * self.L3)
        q3 = np.arccos(np.clip(cos_q3, -1.0, 1.0))
        sin_q2 = ((self.L1 + self.L3 * np.cos(q3)) * z_prime - self.L3 * np.sin(q3) * x_prime) / r**2
        cos_q2 = ((self.L1 + self.L3 * np.cos(q3)) * x_prime + self.L3 * np.sin(q3) * z_prime) / r**2
        q2 = np.arctan2(sin_q2, cos_q2)
        return q1, q2, q3

    def get_trigger_value(self, controller_index):
        # Get the state of the controller
        _, result = openvr.VRSystem().getControllerState(controller_index)
        # Check if the call was successful
        if result:
            # The trigger is typically on the first axis (index 0)
            trigger_value = result.rAxis[1].x  # Adjust the index if necessary
            return trigger_value
        else:
            return None

    def get_pose_matrix(self, pose: openvr.TrackedDevicePose_t):
        """
        Extract the 3x4 pose matrix from the TrackedDevicePose_t object
        Parameters:
            pose: openvr.TrackedDevicePose_t object
        Returns:
            pose_matrix: 3x4 numpy array
        """
        matrix = pose.mDeviceToAbsoluteTracking
        pose_matrix = np.zeros((3, 4))
        for i in range(3):
            for j in range(4):
                pose_matrix[i, j] = matrix[i][j]
        return pose_matrix

    def rotation_matrix_to_euler_angles(self, R):
        # Ensure the input matrix is 3x3
        assert(R.shape == (3, 3))
        
        sy = np.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return x, y, z

    def run(self):
        """
        Continuously read the position of the VR controllers
        """
        while True:
            poses = self.get_controller_poses()
            # Update positions
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                pose = poses[i]
                if pose.bDeviceIsConnected and pose.bPoseIsValid:
                    device_class = openvr.VRSystem().getTrackedDeviceClass(i)
                    if device_class == openvr.TrackedDeviceClass_Controller:
                        pose_matrix = self.get_pose_matrix(pose)
                        trigger_value = self.get_trigger_value(i)
                        self.gripper_angle = -trigger_value * np.pi*0.5
                        position = pose_matrix[:3, 3]
                        rotation = pose_matrix[:3, :3]
                        position[1], position[2] = position[2], position[1]
                        position[1] *= -1.0
                        position[2] -= 1.0
                        self.x_pos, self.y_pos, self.z_pos  = position
                        joint_angles = self.calculate_joint_angles(self.x_pos, self.y_pos, self.z_pos)
                        if joint_angles:
                            self.base_angle = joint_angles[0]-np.pi/2
                            self.shoulder_angle = -joint_angles[1]-np.pi/2
                            self.elbow_angle = -joint_angles[2]+np.pi/2
                        _, _, self.wrist_angle = self.rotation_matrix_to_euler_angles(rotation)

    def read_position(self):
        # make sure each angle is in the range [self.min_angle, self.max_angle] for safety
        self.base_angle = np.clip(self.base_angle, self.min_angle, self.max_angle)
        self.shoulder_angle = np.clip(self.shoulder_angle, self.min_angle, self.max_angle)
        self.elbow_angle = np.clip(self.elbow_angle, self.min_angle, self.max_angle)
        self.wrist_angle = np.clip(self.wrist_angle, self.min_angle, self.max_angle)
        self.gripper_angle = np.clip(self.gripper_angle, self.min_angle, self.max_angle)
        # print all angles
        # print(f'Base angle: {self.base_angle}')
        # print(f'Shoulder angle: {self.shoulder_angle}')
        # print(f'Elbow angle: {self.elbow_angle}')
        # print(f'Wrist angle: {self.wrist_angle}')
        # print(f'Gripper angle: {self.gripper_angle}')
        
        return self.base_angle, self.shoulder_angle, self.elbow_angle, self.wrist_angle, self.gripper_angle

if __name__ == "__main__":
    vr_controller = VRController(1.0, 1.0, 1.0)
    vr_controller.start()
    vr_controller.stop()
