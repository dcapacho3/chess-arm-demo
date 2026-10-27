"""UR5e driver built on MoveIt 2, with a gripper driven straight through Gazebo.
"""
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints,
                             OrientationConstraint, PlanningScene, PositionConstraint, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64

from . import layout as L

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FINGERS = ['finger_left_joint', 'finger_right_joint']
GROUP, TIP = 'ur_manipulator', 'tool0'
HAND = ['gripper_base', 'finger_left', 'finger_right']
SPEED_PER_VMAX = 1.0 / 1.5     # MoveIt velocity scale per m/s of pad speed (UR5e: about 1.5 m/s at scale 1)
CLOSED = 0.024                 # finger command that squeezes (the force cap in the URDF limits the grip)


def tool_pose(x, y, z):
    """Flange pose that puts the pad centre at (x, y, z), fingers pointing down, closing along world y."""
    T = np.diag([1.0, -1.0, -1.0, 1.0])
    T[:3, 3] = [x, y, z + L.TCP_OFFSET]
    return T


def pose_msg(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def _at(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def box(name, size, centre):
    o = CollisionObject()
    o.header.frame_id, o.id, o.operation = 'world', name, CollisionObject.ADD
    o.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size]))
    o.primitive_poses.append(pose_msg(_at(*centre)))
    return o


def piece_cylinder(name, ptype, x, y, frame='world', z_centre=None):
    """Collision cylinder with the piece's base radius and full height (the safe outer shape)."""
    h, r = L.PIECE_SIZE[ptype]
    o = CollisionObject()
    o.header.frame_id, o.id, o.operation = frame, name, CollisionObject.ADD
    o.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[h, r]))
    o.primitive_poses.append(pose_msg(_at(x, y, L.BOARD_TOP + h / 2 if z_centre is None else z_centre)))
    return o


class TrajectoryAborted(RuntimeError):
    """The controller stopped a trajectory because a joint left its tolerance band."""


class MoveItArm(Node):
    def __init__(self):
        super().__init__('chess_arm')
        self.q = None            # arm joint positions
        self.finger = None       # finger joint positions
        self.retries = []        # one message per move that was planned again after an abort
        self.plan_time = 0.0     # seconds spent in planning services
        self.fallbacks = 0       # moves that needed OMPL
        self.types = {}          # piece model name -> piece type
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.create_subscription(JointState, '/gripper/joint_states', self._js, 10)
        self.grip_pubs = [self.create_publisher(Float64, f'/gripper/{s}_cmd', 10) for s in ('left', 'right')]
        self.scene_srv = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.cart_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_act = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.move_act = ActionClient(self, MoveGroup, '/move_action')
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            self.q = np.array([d[j] for j in ARM])
        if all(j in d for j in FINGERS):
            self.finger = np.array([d[j] for j in FINGERS])

    def wait_ready(self, timeout=90.0):
        t0 = time.time()
        while (self.q is None or self.finger is None or not self.scene_srv.service_is_ready()
               or not self.cart_srv.service_is_ready() or not self.exec_act.server_is_ready()
               or not self.move_act.server_is_ready()):
            if time.time() - t0 > timeout:
                raise TimeoutError('arm, controllers or move_group not up (is moveit.launch.py running?)')
            time.sleep(0.2)

    # --- gripper -----------------------------------------------------------------
    def grip(self, target, wait=0.9):
        for pub in self.grip_pubs:
            pub.publish(Float64(data=float(target)))
        time.sleep(wait)

    def open(self):
        self.grip(0.0)

    def close(self):
        self.grip(CLOSED, wait=1.0)

    # --- planning scene ------------------------------------------------------------
    def _apply(self, world=(), attached=()):
        sc = PlanningScene(is_diff=True)
        sc.world.collision_objects = list(world)
        sc.robot_state = RobotState(is_diff=True)
        sc.robot_state.attached_collision_objects = list(attached)
        fut = self.scene_srv.call_async(ApplyPlanningScene.Request(scene=sc))
        while not fut.done():
            time.sleep(0.01)
        if not fut.result().success:
            raise RuntimeError('planning scene update refused')

    def scene_init(self, items):
        """items: (model name, piece type, x, y) for every piece on the table, spares included."""
        table = box('table', (1.5, 1.3, 0.04), (0.38, 0.0, -0.02 - 0.002))
        board = box('board', (8 * L.SQ + 0.03, 8 * L.SQ + 0.03, L.BOARD_TOP), (L.BOARD_CX, 0.0, L.BOARD_TOP / 2))
        objs = [table, board]
        for name, ptype, x, y in items:
            self.types[name] = ptype
            objs.append(piece_cylinder(name, ptype, x, y))
        self._apply(world=objs)

    def attach(self, name):
        """The gripper has closed on `name` at the pick height: move it from the world onto the hand."""
        rm = CollisionObject()
        rm.id, rm.operation = name, CollisionObject.REMOVE
        h = L.PIECE_SIZE[self.types[name]][0]
        # piece centre in the tool0 frame: straight below the flange, tool z points down
        z_tool = (L.GRASP_Z + L.TCP_OFFSET) - (L.BOARD_TOP + h / 2)
        obj = piece_cylinder(name, self.types[name], 0.0, 0.0, frame=TIP, z_centre=z_tool)
        att = AttachedCollisionObject(link_name=TIP, object=obj, touch_links=HAND + ['tool0', 'wrist_3_link'])
        self._apply(world=[rm])          # one step at a time: MoveIt refuses a diff that removes and attaches together
        self._apply(attached=[att])

    def detach(self, name, x, y):
        """The gripper has opened over (x, y): put `name` back into the world there."""
        det = AttachedCollisionObject(link_name=TIP)
        det.object.id, det.object.operation = name, CollisionObject.REMOVE
        self._apply(attached=[det])      # detaching drops the object into the world where the hand holds it
        self._apply(world=[piece_cylinder(name, self.types[name], x, y)])   # then set its exact resting pose

    # --- motion --------------------------------------------------------------------
    def line(self, target, vmax=0.25, attempts=3):
        """Straight line of the pad centre to target (x, y, z), orientation fixed.

        After a tolerance abort the path is planned again from wherever the arm stopped.
        """
        for attempt in range(attempts):
            try:
                return self._line_once(target, vmax)
            except TrajectoryAborted as e:
                self.retries.append(f'{e} (target {np.round(target, 3).tolist()}, attempt {attempt + 1})')
                self.get_logger().warn(self.retries[-1])
                if attempt == attempts - 1:
                    raise
                time.sleep(0.5)

    def _line_once(self, target, vmax):
        t0 = time.time()
        target = np.array(target, float)
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = RobotState(is_diff=True)
        req.start_state.joint_state = JointState(name=ARM, position=self.q.tolist())
        req.group_name, req.link_name = GROUP, TIP
        req.waypoints = [pose_msg(tool_pose(*target))]
        req.max_step, req.jump_threshold = 0.004, 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = float(min(1.0, vmax * SPEED_PER_VMAX))
        req.max_acceleration_scaling_factor = 0.3
        fut = self.cart_srv.call_async(req)
        while not fut.done():
            time.sleep(0.01)
        res = fut.result()
        self.plan_time += time.time() - t0
        traj = res.solution.joint_trajectory
        if res.fraction >= 0.999 and len(traj.points) < 2:
            return                       # the pad is already at the target: nothing to plan
        if res.fraction < 0.999:
            self.fallbacks += 1
            self.get_logger().warn(f'cartesian fraction {res.fraction:.2f} to {target.round(3)}: falling back to OMPL')
            return self._ompl(target)
        jumps = np.abs(np.diff([p.positions for p in traj.points], axis=0)).max()
        if jumps > 0.4:
            raise RuntimeError(f'cartesian path jumps {jumps:.2f} rad between points')
        self._execute(res.solution)

    def _execute(self, solution):
        fut = self.exec_act.send_goal_async(ExecuteTrajectory.Goal(trajectory=solution))
        while not fut.done():
            time.sleep(0.01)
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError('execute_trajectory rejected')
        rf = handle.get_result_async()
        while not rf.done():
            time.sleep(0.01)
        code = rf.result().result.error_code.val
        if code in (-4, -5, 99999, -1):  # path or goal tolerance, control failed, timed out
            raise TrajectoryAborted(f'execute_trajectory error {code}')
        if code != 1:
            raise RuntimeError(f'execute_trajectory error {code}')

    def _ompl(self, target):
        t0 = time.time()
        pose = pose_msg(tool_pose(*target))
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name, r.num_planning_attempts, r.allowed_planning_time = GROUP, 5, 4.0
        r.max_velocity_scaling_factor, r.max_acceleration_scaling_factor = 0.15, 0.3
        pc = PositionConstraint(link_name=TIP, weight=1.0)
        pc.header.frame_id = 'world'
        pc.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])],
                                              primitive_poses=[pose])
        oc = OrientationConstraint(link_name=TIP, orientation=pose.orientation, weight=1.0,
                                   absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02,
                                   absolute_z_axis_tolerance=0.02)
        oc.header.frame_id = 'world'
        r.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        goal.planning_options.plan_only = False
        fut = self.move_act.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.01)
        rf = fut.result().get_result_async()
        while not rf.done():
            time.sleep(0.01)
        self.plan_time += time.time() - t0
        code = rf.result().result.error_code.val
        if code != 1:
            raise RuntimeError(f'OMPL fallback failed, error {code}')
