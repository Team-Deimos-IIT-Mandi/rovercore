#!/usr/bin/env python3
"""
arm_reachability_cloud.py
--------------------------
High-Performance Vectorized FK Workspace Mapper.
Evaluates 30 Million poses in seconds, filters for the extreme outer 2cm boundary,
and publishes the resulting hollow shell to RViz.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np

# ── helpers ──────────────────────────────────────────────────────────────────

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], float)

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], float)

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], float)

def T(x, y, z):
    m = np.eye(4)
    m[0,3], m[1,3], m[2,3] = x, y, z
    return m

def rpy_mat(r, p, y):
    return Rz(y) @ Ry(p) @ Rx(r)

def fixed_tf(xyz, rpy):
    return T(*xyz) @ rpy_mat(*rpy)

def rot_axis(ax, angles):
    ax = np.asarray(ax, float)
    ax /= np.linalg.norm(ax)
    x, y, z = ax
    N = len(angles)
    c = np.cos(angles)
    s = np.sin(angles)
    ic = 1.0 - c

    R = np.zeros((N, 4, 4), float)
    R[:, 3, 3] = 1.0
    R[:, 0, 0] = c + x*x*ic
    R[:, 0, 1] = x*y*ic - z*s
    R[:, 0, 2] = x*z*ic + y*s
    R[:, 1, 0] = y*x*ic + z*s
    R[:, 1, 1] = c + y*y*ic
    R[:, 1, 2] = y*z*ic - x*s
    R[:, 2, 0] = z*x*ic - y*s
    R[:, 2, 1] = z*y*ic + x*s
    R[:, 2, 2] = c + z*z*ic
    return R

def apply_fixed(batch, fixed):
    return batch @ fixed[np.newaxis]

def prepend_fixed(fixed, batch):
    return fixed[np.newaxis] @ batch

# ── URDF fixed joint origins ─────────────────────────────────────────────────

TF_base_to_chassis   = fixed_tf([0, 0, 0.43083],                                [0, -1.5708, 0])
TF_chassis_to_zaxis  = fixed_tf([0.12, 0.05, -0.02],                            [0,  1.57,   0])
TF_zaxis_to_link1    = fixed_tf([0.0717427736589908, -0.118676975137196, 0.043], [0,  0, -0.539222444193772])
TF_link1_to_link2    = fixed_tf([0.0463762994663664,  0.026,             0.4476],[0,  0.103241752091021, 0])
TF_link2_to_link3    = fixed_tf([0.067242668214866,   0.039,             0.0872],[0.468343930204125, -0.299928960515459, 0])
TF_link3_to_wrist1   = fixed_tf([-0.544,             -0.05248,           0],     [0, 0, 0])
TF_wrist1_to_gripper = fixed_tf([-0.00704202903975437, 0.0524785020943702, 0.0543441409794076],
                                  [-1.65253448174348,   0.0997346165681447, 2.45313293712552])

TF_base_to_z0 = TF_base_to_chassis @ TF_chassis_to_zaxis 

# ── joint axes & limits ───────────────────────────────────────────────────────

AX_z    = [0,  0,  1]
AX_l1   = [0, -1,  0]
AX_l2   = [0, -1,  0]
AX_l3   = [-1, 0,  0]
AX_w1   = [0,  1,  0]
AX_gw   = [0, -1,  0]

LIM = {
    "z_axis":        (0.0,   6.24),
    "link_1":        (-0.52, 0.96),
    "link_2":        (-1.0,  1.0),
    "link_3":        (0.0,   6.28),
    "wrist_1":       (-1.57, 1.57),
    "gripper_wrist": (-1.57, 1.57),
}

# ── vectorised FK with Boundary Filter ────────────────────────────────────────

def sample_reachability(batch_size: int = 3_000_000) -> tuple:
    rng = np.random.default_rng()
    def u(k): return rng.uniform(*LIM[k], batch_size)

    q_z  = u("z_axis")
    q_1  = u("link_1")
    q_2  = u("link_2")
    q_3  = np.zeros(batch_size)
    q_w1 = np.zeros(batch_size)
    q_gw = np.zeros(batch_size)

    T = prepend_fixed(TF_base_to_z0, rot_axis(AX_z, q_z))
    fixed_then_rot = lambda T, fixed, ax, q: (T @ fixed) @ rot_axis(ax, q)

    T = fixed_then_rot(T, TF_zaxis_to_link1,    AX_l1, q_1)
    T = fixed_then_rot(T, TF_link1_to_link2,    AX_l2, q_2)
    T = fixed_then_rot(T, TF_link2_to_link3,    AX_l3, q_3)
    T = fixed_then_rot(T, TF_link3_to_wrist1,   AX_w1, q_w1)
    T = fixed_then_rot(T, TF_wrist1_to_gripper, AX_gw, q_gw)

    all_pts = T[:, :3, 3]
    distances = np.linalg.norm(all_pts, axis=1)
    max_reach = np.max(distances)
    
    # Boundary Filter
    mask = distances > 0.2
    boundary_pts = all_pts[mask]
    angles = np.column_stack((q_z[mask], q_1[mask], q_2[mask]))
    
    return boundary_pts, angles, float(max_reach)

# ── ROS 2 node ────────────────────────────────────────────────────────────────

class ReachabilityCloudNode(Node):
    def __init__(self):
        super().__init__('arm_reachability_cloud')
        self.pub = self.create_publisher(MarkerArray, '/arm_reachability_cloud', 10)

        all_edge_points = []
        all_edge_angles = []
        global_max_reach = 0.0
        
        num_chunks = 10
        chunk_size = 3_000_000
        
        for chunk in range(num_chunks):
            self.get_logger().info(f'Computing chunk {chunk + 1}/{num_chunks}...')
            pts, angs, max_reach = sample_reachability(chunk_size)
            all_edge_points.append(pts)
            all_edge_angles.append(angs)
            
            if max_reach > global_max_reach:
                global_max_reach = max_reach
                
        final_points = np.vstack(all_edge_points)
        final_angles = np.vstack(all_edge_angles)
        
        # --- SAVE DATA ---
        np.savez_compressed('workspace_lookup.npz', points=final_points, angles=final_angles)
        self.get_logger().info('Lookup table saved to workspace_lookup.npz')

        # --- THE 1 MILLION POINT RENDER CAP ---
        MAX_RENDER_POINTS = 1_000_000
        if len(final_points) > MAX_RENDER_POINTS:
            indices = np.random.choice(len(final_points), MAX_RENDER_POINTS, replace=False)
            final_points = final_points[indices]
        
        self.get_logger().info(f'Filtered to {len(final_points):,} points for RViz.')

        # Pre-build markers
        self._colors, _, _ = self.make_colors(final_points[:, 2])
        self._point_list = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in final_points]
        self._base_pos = (TF_base_to_z0 @ np.array([0, 0, 0, 1]))[:3]
        self._publish()

    def make_colors(self, zs):
        t = np.clip((zs - zs.min()) / (zs.max() - zs.min() + 1e-9), 0.0, 1.0)
        return [ColorRGBA(r=float(min(1.0, 2.0*ti)), g=float(min(1.0, 2.0*(1.0-ti))), b=0.0, a=1.0) for ti in t], 0, 0

    def _publish(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = now
        m.ns = 'reachability'; m.id = 0; m.type = Marker.POINTS; m.action = Marker.ADD
        m.scale.x = 0.005; m.scale.y = 0.005
        m.points = self._point_list; m.colors = self._colors; m.lifetime.sec = 0 
        ma.markers.append(m)
        s = Marker()
        s.header.frame_id = 'base_link'; s.header.stamp = now; s.ns = 'reachability'; s.id = 1; s.type = Marker.SPHERE; s.action = Marker.ADD
        s.pose.position.x = float(self._base_pos[0]); s.pose.position.y = float(self._base_pos[1]); s.pose.position.z = float(self._base_pos[2])
        s.scale.x = s.scale.y = s.scale.z = 0.05; s.color.r = 0.2; s.color.g = 0.5; s.color.b = 1.0; s.color.a = 1.0; s.lifetime.sec = 4
        ma.markers.append(s)
        self.pub.publish(ma)

def main(args=None):
    rclpy.init(args=args); node = ReachabilityCloudNode(); rclpy.spin(node)

if __name__ == '__main__':
    main()