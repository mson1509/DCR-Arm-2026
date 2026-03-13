import numpy as np
import ikpy.chain
from geometry_msgs.msg import Point

class IKSolver:
    def __init__(self):
        # Define the robot chain
        # pkg_path = get_package_share_directory('motor_node')
        # urdf_path = os.path.join(pkg_path, 'urdf', 'arm.urdf')
        self.chain = ikpy.chain.Chain.from_urdf_file('/home/me/arm_ws/src/motor_node/urdf/arm.urdf',active_links_mask=[False, True, True, True, False])
        self.old_joints = [0.0] * len(self.chain.links)

    def solve(self, target_pos):
        joints_rad = self.chain.inverse_kinematics(target_position=[target_pos.x, target_pos.y, target_pos.z],
            initial_position=self.old_joints,
            target_orientation=None,
            orientation_mode=None)
        self.old_joints = joints_rad
        joints_deg = np.degrees(joints_rad)
        joints_deg_round = np.round(joints_deg / 0.02) * 0.02
        return joints_deg_round[1:4]

    def desolve(self, joints):
        joints_rad = np.radians(joints)

        full_joints = [0, joints_rad[0], joints_rad[1], joints_rad[2], 0]

        T = self.chain.forward_kinematics(full_joints)

        pos = Point()
        pos.x = float(T[0, 3])
        pos.y = float(T[1, 3])
        pos.z = float(T[2, 3])

        return pos

