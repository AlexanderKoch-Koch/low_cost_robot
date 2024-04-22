import openvr
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def init_openvr():
    openvr.init(openvr.VRApplication_Other)
    print("OpenVR initialized")

def get_controller_poses():
    poses = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(
        openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    return poses

def close_plot(event):
    plt.close()  # Close the matplotlib window
    openvr.shutdown()  # Properly shutdown OpenVR
    exit()  # Exit the Python script


def draw_cube(ax, position, rotation, size=0.1):
    # Define the cube's eight vertices
    half_size = size / 2.0
    vertices = np.array([
        [-half_size, -half_size, -half_size],
        [+half_size, -half_size, -half_size],
        [+half_size, +half_size, -half_size],
        [-half_size, +half_size, -half_size],
        [-half_size, -half_size, +half_size],
        [+half_size, -half_size, +half_size],
        [+half_size, +half_size, +half_size],
        [-half_size, +half_size, +half_size]
    ])
    
    # Apply rotation and translation to the vertices
    vertices = vertices @ rotation.T + position
    
    # Define the six faces of the cube
    # Each face uses vertices that are listed in a clockwise or counter-clockwise order to maintain correct normals
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left face
        [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right face
    ]
    
    # Create a 3D polygon collection
    cube = Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.25)
    
    # Add the cube to the axes
    ax.add_collection3d(cube)

def rotation_matrix_to_euler_angles(R):
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

    return np.degrees(x), np.degrees(y), np.degrees(z)  # Convert from radians to degrees

# Function to calculate joint angles
def calculate_joint_angles(x, y, z, L1, L2, L3):
    q1 = np.arctan2(y, x)
    x_prime = np.sqrt(x**2 + y**2)
    z_prime = -z + L1
    r = np.sqrt(x_prime**2 + z_prime**2)
    # Check if the point is reachable
    if r > (L2 + L3) or r < abs(L2 - L3):
        return None  # Target is not reachable, return None
    cos_q3 = (r**2 - L2**2 - L3**2) / (2 * L2 * L3)
    q3 = np.arccos(np.clip(cos_q3, -1.0, 1.0))
    sin_q2 = ((L2 + L3 * np.cos(q3)) * z_prime - L3 * np.sin(q3) * x_prime) / r**2
    cos_q2 = ((L2 + L3 * np.cos(q3)) * x_prime + L3 * np.sin(q3) * z_prime) / r**2
    q2 = np.arctan2(sin_q2, cos_q2)
    return q1, q2, q3


def plot_real_time():
    # Initialize OpenVR
    init_openvr()
    
    # Set up the matplotlib figure and axes, using 3D projection
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Z Position')  # Swapping label to Z
    ax.set_zlabel('Y Position')  # Swapping label to Y
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    # Button
    ax_button = plt.axes([0.8, 0.0, 0.1, 0.05])  # Adjust the rectangle for button placement
    button = Button(ax_button, 'Exit')
    button.on_clicked(close_plot)

    # Continuously update the plot
    try:
        while True:
            ax.cla()  # Clear the axis for fresh plot
            ax.set_xlabel('X Position')
            ax.set_ylabel('Y Position')
            ax.set_zlabel('Z Position')
            ax.set_xlim([-0.5, 0.5])
            ax.set_ylim([-0.5, 0.5])
            ax.set_zlim([-0.25, 0.5])

            poses = get_controller_poses()
            # Update positions
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                pose = poses[i]
                if pose.bDeviceIsConnected and pose.bPoseIsValid:
                    device_class = openvr.VRSystem().getTrackedDeviceClass(i)
                    if device_class == openvr.TrackedDeviceClass_Controller:
                        matrix = pose.mDeviceToAbsoluteTracking
                        pose_matrix = np.zeros((3, 4))
                        for i in range(3):
                            for j in range(4):
                                pose_matrix[i, j] = matrix[i][j]
                        position = pose_matrix[:3, 3]
                        position[1], position[2] = position[2], position[1]
                        position[1] *= -1.0
                        position[2] -= 1.0
                        rotation = pose_matrix[:3, :3]
                        x, y, z = position
                        scale = 3
                        L1, L2, L3 = 0.0239*scale, 0.1144*scale, (0.0469+ 0.0207)*scale
                        angles = calculate_joint_angles(x, y, z, L1, L2, L3)
                        if angles is not None:
                            q1, q2, q3 = angles
                            base = np.array([0, 0, 0.0])
                            joint1 = base + np.array([0, 0, L1])
                            joint2 = joint1 + np.array([L2 * np.cos(q1) * np.cos(q2), L2 * np.sin(q1) * np.cos(q2), -L2 * np.sin(q2)])
                            end_effector = joint2 + np.array([L3 * np.cos(q1) * np.cos(q2 + q3), L3 * np.sin(q1) * np.cos(q2 + q3), -L3 * np.sin(q2 + q3)])
                            ax.plot([base[0], joint1[0], joint2[0], end_effector[0]], [base[1], joint1[1], joint2[1], end_effector[1]], [base[2], joint1[2], joint2[2], end_effector[2]], 'o-')
                            
                        ax.scatter(x, y, z, label=f"Controller {i}")
                        draw_cube(ax, position, rotation)
                        pitch, yaw, roll = rotation_matrix_to_euler_angles(rotation)
                        print(f"Controller {i}  pitch {pitch:.2f} yaw {yaw:.2f} roll {roll:.2f}")
                        print(f"Controller {i} position: {x:.2f}, {y:.2f}, {z:.2f}")
                        # print angles:
                        if angles is not None:
                            print(f"Base angle: {q1-np.pi/2:.2f}")
                            print(f"Shoulder angle: {-q2-np.pi/2:.2f}")
                            print(f"Elbow angle: {-q3+np.pi/2:.2f}")

            plt.draw()
            plt.pause(0.01)  # Pause briefly to allow for updates
            
    finally:
        plt.close()
        openvr.shutdown()

if __name__ == "__main__":
    plot_real_time()
