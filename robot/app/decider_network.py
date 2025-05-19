import time
from collections import OrderedDict

import numpy as np
from numpy.typing import NDArray
import omni.isaac.cortex.math_util as math_util
from omni.isaac.cortex.cortex_object import CortexObject
from omni.isaac.cortex.df import *
from omni.isaac.cortex.dfb import DfApproachGrasp, DfCloseGripper, DfOpenGripper, DfRobotApiContext, make_go_home
from omni.isaac.cortex.motion_commander import MotionCommand, PosePq
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.cortex.cortex_world import CommandableArticulation, CortexWorld
from omni.isaac.cortex.robot import add_franka_to_stage, CortexRobot, MotionCommandedRobot, CortexUr10, CortexGripper
from scipy.spatial.transform import Rotation

import carb.events
import omni.kit.app

# global variable. Set to True to enable the robot to run.
can_run_robot = True

def make_grasp_T(t, ay):
    """
    Create a grasp transformation matrix.

    Args:
        t: Translation vector.
        ay: Y-axis vector.

    Returns:
        Transformation matrix.
    """
    az = math_util.normalized(-t)
    ax = np.cross(ay, az)

    T = np.eye(4)
    T[:3, 0] = ax
    T[:3, 1] = ay
    T[:3, 2] = az
    T[:3, 3] = t

    return T


def make_block_grasp_Ts(block_pick_height):
    """
    Generate grasp transformation matrices for a block.

    Args:
        block_pick_height: Height at which to pick the block.

    Returns:
        List of transformation matrices.
    """
    R = np.eye(3)

    Ts = []
    for i in range(3):
        t = block_pick_height * R[:, i]
        for j in range(2):
            ay = R[:, (i + j + 1) % 3]
            for s1 in [1, -1]:
                for s2 in [1, -1]:
                    grasp_T = make_grasp_T(s1 * t, s2 * ay)
                    Ts.append(grasp_T)

    return Ts


def get_world_block_grasp_Ts(
    obj_T,
    obj_grasp_Ts,
    axis_x_filter=None,
    axis_x_filter_thresh=0.1,
    axis_y_filter=None,
    axis_y_filter_thresh=0.1,
    axis_z_filter=None,
    axis_z_filter_thresh=0.1,
):
    """
    Filter and transform grasp matrices to world coordinates.

    Args:
        obj_T: Object transformation matrix.
        obj_grasp_Ts: List of grasp transformation matrices.
        axis_x_filter: Optional filter for x-axis.
        axis_x_filter_thresh: Threshold for x-axis filter.
        axis_y_filter: Optional filter for y-axis.
        axis_y_filter_thresh: Threshold for y-axis filter.
        axis_z_filter: Optional filter for z-axis.
        axis_z_filter_thresh: Threshold for z-axis filter.

    Returns:
        List of filtered world grasp transformation matrices.
    """
    world_grasp_Ts = []
    for gT in obj_grasp_Ts:
        world_gT = obj_T.dot(gT)
        if axis_x_filter is not None and (
            1.0 - world_gT[:3, 0].dot(math_util.normalized(axis_x_filter)) > axis_x_filter_thresh
        ):
            continue
        if axis_y_filter is not None and (
            1.0 - world_gT[:3, 1].dot(math_util.normalized(axis_y_filter)) > axis_y_filter_thresh
        ):
            continue
        if axis_z_filter is not None and (
            1.0 - world_gT[:3, 2].dot(math_util.normalized(axis_z_filter)) > axis_z_filter_thresh
        ):
            continue

        world_grasp_Ts.append(world_gT)
    return world_grasp_Ts


def get_best_obj_grasp(obj_T, obj_grasp_Ts, eff_T, other_obj_Ts):
    """
    Uses a manually defined score-based classifier for choosing which grasp to use on a given block.

    It chooses a grasp that's simultaneously natural for the arm and avoids any nearby blocks.

    Args:
        obj_T: The block object being grasped.
        obj_grasp_Ts: The grasp transforms in coordinates local to the block.
        eff_T: The current end-effector transform.
        other_obj_Ts: The transforms of all other surrounding blocks we want to consider.

    Returns:
        Best grasp transformation matrix.
    """
    Ts = get_world_block_grasp_Ts(obj_T, obj_grasp_Ts, axis_z_filter=np.array([0.0, 0.0, -1.0]))

    # This could happen if all the grasps are filtered out.
    if len(Ts) == 0:
        return None

    # Score each grasp based on how well the gripper's x-axis will correlate with the direction to
    # the robot (most natural configuration).
    obj_p = obj_T[:3, 3]
    v = math_util.normalized(-obj_p)

    # Score each of the candidate grasps based on how their world transform is oriented relative to
    # the robot and relative to nearby other blocks.
    scores = np.zeros(len(Ts))
    for i, grasp_T in enumerate(Ts):
        # The base score is a function of how the end-effector would be oriented relative to the
        # base of the robot.
        score = grasp_T[:3, 0].dot(v)

        # For all surrounding objects, if the object is closer than 15cm away, add a proximity cost
        # (negative score) for the grasp based on whether the finger might clip it.
        for obj_T in other_obj_Ts:
            other_obj_p = obj_T[:3, 3]
            toward_other = other_obj_p - obj_p
            dist = np.linalg.norm(toward_other)
            if dist < 0.25:
                # Care about closer blocks more.
                w = np.exp(-0.5 * (dist / 0.15) ** 2)
                prox_score = -10.0 * w * (grasp_T[:3, 1].dot(math_util.normalized(toward_other))) ** 2
                score += prox_score

        scores[i] += score

    # Return the highest scoring transform.
    scored_Ts = zip(scores, Ts)
    T = max(scored_Ts, key=lambda v: v[0])[1]
    return T

def calc_grasp_for_top_of_tower(context, target_p):
    """
    Calculate the grasp transformation matrix for the top of the tower.

    Args:
        context: BuildTowerContext object.
        target_p: Target position.

    Returns:
        Grasp transformation matrix.
    """
    p = target_p

    R = np.eye(3)

    T = math_util.pack_Rp(R, p)

    ct = context
    block_target_T = T
    candidate_Ts = get_world_block_grasp_Ts(
        block_target_T, ct.active_block.grasp_Ts, axis_z_filter=np.array([0.0, 0.0, -1.0])
    )
    if len(candidate_Ts) == 0:
        return None

    desired_ax = np.array([0.0, -1.0, 0.0])
    scored_candidate_Ts = [(np.dot(desired_ax, T[:3, 0]), T) for T in candidate_Ts]

    grasp_T = max(scored_candidate_Ts, key=lambda v: v[0])[1]
    return grasp_T


class BuildTowerContext(DfRobotApiContext):
    """
    Context for building a tower with blocks.

    Attributes:
        robot: MotionCommandedRobot object.
        dropPointAPos: Position of drop point A.
        block_height: Height of a block.
        block_pick_height: Height at which to pick a block.
        block_grasp_Ts: List of grasp transformation matrices for blocks.
        diagnostics_message: Diagnostics message.
    """
    class Block:
        """
        Represents a block in the tower building context.

        Attributes:
            i: Index of the block.
            obj: Object representing the block.
            is_aligned: Whether the block is aligned.
            grasp_Ts: List of grasp transformation matrices.
            chosen_grasp: Chosen grasp transformation matrix.
            collision_avoidance_enabled: Whether collision avoidance is enabled.
            drop_point: Drop point for the block.
        """
        def __init__(self, i, obj, grasp_Ts, drop_point):
            self.i = i
            self.obj = obj
            self.is_aligned = None
            self.grasp_Ts = grasp_Ts
            self.chosen_grasp = None
            self.collision_avoidance_enabled = True

            self.drop_point = drop_point

        @property
        def has_chosen_grasp(self):
            return self.chosen_grasp is not None

        @property
        def name(self):
            return self.obj.name

        def get_best_grasp(self, eff_T, other_obj_Ts):
            return get_best_obj_grasp(self.obj.get_transform(), self.grasp_Ts, eff_T, other_obj_Ts)

        def set_aligned(self):
            self.is_aligned = True

    def __init__(self, robot: MotionCommandedRobot, dropPointAPos: NDArray):
        super().__init__(robot)

        self.robot = robot
        self.dropPointAPos = dropPointAPos

        self.block_height = 0.0515
        self.block_pick_height = 0.02
        self.block_grasp_Ts = make_block_grasp_Ts(self.block_pick_height)
        self.diagnostics_message = ""
        self.reset()

        self.add_monitors(
            [
                BuildTowerContext.monitor_perception,
                BuildTowerContext.monitor_gripper_has_block,
                BuildTowerContext.monitor_suppression_requirements,
                BuildTowerContext.monitor_diagnostics,
            ]
        )

    def reset(self):
        """
        Reset the context.
        """
        self.blocks = OrderedDict()

        self.active_block = None
        self.in_gripper = None
        self.placement_target_eff_T = None

        self.print_dt = 0.25
        self.next_print_time = None
        self.start_time = None

    @property
    def has_active_block(self):
        return self.active_block is not None

    def activate_block(self, name):
        self.active_block = self.blocks[name]

    def reset_active_block(self):
        if self.active_block is None:
            return

        self.active_block.chosen_grasp = None
        self.active_block = None

    @property
    def block_names(self):
        block_names = [name for name in self.blocks.keys()]
        return block_names

    @property
    def num_blocks(self):
        return len(self.blocks)

    def mark_block_in_gripper(self):
        """
        Mark the block currently in the gripper.
        """
        eff_p = self.robot.arm.get_fk_p()
        blocks_with_dists = []
        for _, block in self.blocks.items():
            block_p, _ = block.obj.get_world_pose()
            blocks_with_dists.append((block, np.linalg.norm(eff_p - block_p)))

        closest_block, _ = min(blocks_with_dists, key=lambda v: v[1])
        self.in_gripper = closest_block

    def clear_gripper(self):
        """
        Clear the gripper.
        """
        self.in_gripper = None

    @property
    def is_gripper_clear(self):
        return self.in_gripper is None

    @property
    def gripper_has_block(self):
        return not self.is_gripper_clear

    @property
    def has_placement_target_eff_T(self):
        return self.placement_target_eff_T is not None

    def monitor_perception(self):
        """
        Monitor the perception of blocks.
        """
        for _, block in self.blocks.items():
            obj = block.obj
            if not obj.has_measured_pose():
                continue

            measured_T = obj.get_measured_T()
            belief_T = obj.get_T()

            not_in_gripper = block != self.in_gripper

            eff_p = self.robot.arm.get_fk_p()
            sync_performed = False
            if not_in_gripper and np.linalg.norm(belief_T[:3, 3] - eff_p) > 0.05:
                sync_performed = True
                obj.sync_to_measured_pose()
            elif np.linalg.norm(belief_T[:3, 3] - measured_T[:3, 3]) > 0.15:
                sync_performed = True
                obj.sync_to_measured_pose()

    def monitor_gripper_has_block(self):
        """
        Monitor whether the gripper has a block.
        """
        if self.gripper_has_block:
            block = self.in_gripper
            _, block_p = math_util.unpack_T(block.obj.get_transform())
            eff_p = self.robot.arm.get_fk_p()
            if np.linalg.norm(block_p - eff_p) > 0.1:
                self.diagnostics_message = "Block lost. Clearing gripper."
                self.clear_gripper()

    def monitor_suppression_requirements(self):
        """
        Monitor suppression requirements for collision avoidance.
        """
        arm = self.robot.arm
        eff_T = arm.get_fk_T()
        eff_R, eff_p = math_util.unpack_T(eff_T)
        ax, ay, az = math_util.unpack_R(eff_R)

        target_p, _ = arm.target_prim.get_world_pose()

        toward_target = target_p - eff_p
        dist_to_target = np.linalg.norm(toward_target)

        blocks_to_suppress = []
        if self.gripper_has_block:
            blocks_to_suppress.append(self.in_gripper)

        for name, block in self.blocks.items():
            block_T = block.obj.get_transform()
            block_R, block_p = math_util.unpack_T(block_T)

            # If the block is close to the target and the end-effector is above the block (in z), then
            # suppress it.
            target_dist_to_block = np.linalg.norm(block_p - target_p)
            xy_dist = np.linalg.norm(block_p[:2] - target_p[:2])
            margin = 0.05
            # Add the block if either we're descending on the block, or they're neighboring blocks
            # during the descent.
            if (
                target_dist_to_block < 0.1
                and (xy_dist < 0.02 or eff_p[2] > block_p[2] + margin)
                or target_dist_to_block < 0.15
                and target_dist_to_block > 0.07
                and eff_p[2] > block_p[2] + margin
            ):
                if block not in blocks_to_suppress:
                    blocks_to_suppress.append(block)

        for block in blocks_to_suppress:
            if block.collision_avoidance_enabled:
                try:
                    arm.disable_obstacle(block.obj)
                    block.collision_avoidance_enabled = False
                except Exception as e:
                    print("error disabling obstacle")
                    import traceback

                    traceback.print_exc()

        for name, block in self.blocks.items():
            if block not in blocks_to_suppress:
                if not block.collision_avoidance_enabled:
                    arm.enable_obstacle(block.obj)
                    block.collision_avoidance_enabled = True

    def monitor_diagnostics(self):
        """
        Monitor diagnostics and print messages.
        """
        now = time.time()
        if self.start_time is None:
            self.start_time = now
            self.next_print_time = now + self.print_dt

        if now >= self.next_print_time:
            out = ("time since start: %f sec" % (now - self.start_time)) + "\n"
            self.next_print_time += self.print_dt

            if self.has_active_block:
                out += f"active block:{self.active_block.name}"
            else:
                out += "no active block"
            self.diagnostics_message = out

class MyDfOpenGripper(DfAction):
    """
    Custom action to open the gripper.
    """
    def enter(self) -> None:
        self.context.robot.gripper.open(speed=5)
        pass

class MyDfCloseGripper(DfAction):
    """
    Custom action to close the gripper.
    """
    def enter(self):
        self.context.robot.gripper.close(speed=5)
        pass


class ReachToBlockRd(DfRldsNode):
    """
    RLDS node to reach to a block.

    Attributes:
        child_name: Name of the child node.
    """
    def __init__(self):
        super().__init__()
        self.child_name = None

    def link_to(self, name, decider):
        self.child_name = name
        self.add_child(name, decider)

    def is_runnable(self):
        return self.context.is_gripper_clear

    def decide(self):
        self.context.robot.gripper.open(speed=5)
        return DfDecision(self.child_name)


class GoHome(DfDecider):
    """
    Decider to go home.

    Attributes:
        None
    """
    def __init__(self):
        super().__init__()
        self.add_child("go_home", make_go_home())

    def enter(self):
        self.context.robot.gripper.close(speed=5)
        pass

    def decide(self):
        return DfDecision("go_home")


class ChooseNextBlockForTowerBuildUp(DfDecider):
    """
    Decider to choose the next block for tower build-up.

    Attributes:
        child_name: Name of the child node.
        world: CortexWorld instance.
    """
    def __init__(self):
        super().__init__()
        # If conditions aren't good, we'll just go home.
        self.add_child("go_home", GoHome())
        self.child_name = None

        self.world: CortexWorld = CortexWorld.instance()

    def link_to(self, name, decider):
        self.child_name = name
        self.add_child(name, decider)

    def decide(self):
        ct = self.context

        if ct.active_block is None:
            try:
                cortex_obj: XFormPrim = self.world.scene.get_object("object")
                if cortex_obj is not None:
                    # Cycle through all bins and find the bin in the active region with smallest y value.
                    p, _ = cortex_obj.get_world_pose()

                    # Check whether it's on the conveyor in the active region.
                    x, y, z = p
                    # if 0.3 < y and y < 1 and 0.2 < x and x < 0.7:
                    global can_run_robot
                    print(f"new block found: {can_run_robot}")

                    if not can_run_robot:
                        return DfDecision("go_home")

                    # This behavior might be run either with CortexObjects (e.g. when synchronizing with a
                    # sim/real world via ROS) or standard core API objects. If it's the latter, add the
                    # CortexObject API.
                    if not isinstance(cortex_obj, CortexObject):
                        cortex_obj = CortexObject(cortex_obj)

                    name = cortex_obj.name
                    i = len(ct.blocks)

                    cortex_obj.sync_throttle_dt = 0.25

                    drop_point = ct.dropPointAPos

                    ct.blocks[name] = BuildTowerContext.Block(i, cortex_obj, ct.block_grasp_Ts, drop_point)

                    ct.active_block = ct.blocks[name]
                else:
                    print("no object found")
            except Exception as e:
                pass
        if ct.active_block is None:
            # print("no active block")
            return DfDecision("go_home")

        # Check exceptions
        block_p, _ = ct.active_block.obj.get_world_pose()
        if np.linalg.norm(block_p) < 0.25:
            print("block too close to robot base: {}".format(ct.active_block.name))
            return DfDecision("go_home")
        elif np.linalg.norm(block_p) > 1:
            # block is too far away, go home
            return DfDecision("go_home")

        other_obj_Ts = []
        ct.active_block.chosen_grasp = ct.active_block.get_best_grasp(ct.robot.arm.get_fk_T(), other_obj_Ts)
        return DfDecision(self.child_name, ct.active_block.chosen_grasp)

    def exit(self):
        if self.context.active_block is not None:
            self.context.active_block.chosen_grasp = None


class LiftState(DfState):
    """A simple state which sends a target a distance command_delta_z above the current
    end-effector position until the end-effector has moved success_delta_z meters up.

    Args:
        command_delta_z: The delta offset up to shift the command away from the current end-effector
            position every cycle.
        success_delta_z: The delta offset up from the original end-effector position measured on
            entry required for exiting the state.
    """

    def __init__(self, command_delta_z, success_delta_z, cautious_command_delta_z=None):
        self.command_delta_z = command_delta_z
        self.cautious_command_delta_z = cautious_command_delta_z
        self.success_delta_z = success_delta_z

    def enter(self):
        # On entry, set the posture config to the current config so the movement is minimal.
        posture_config = self.context.robot.arm.articulation_subset.get_joints_state().positions.astype(float)
        self.context.robot.arm.set_posture_config(posture_config)

        self.success_z = self.context.robot.arm.get_fk_p()[2] + self.success_delta_z

    def closest_non_grasped_block_dist(self, eff_p):
        """
        Calculate the distance to the closest non-grasped block.

        Args:
            eff_p: End-effector position.

        Returns:
            Distance to the closest non-grasped block.
        """
        blocks_with_dists = []
        for name, block in self.context.blocks.items():
            block_p, _ = block.obj.get_world_pose()
            dist = np.linalg.norm(eff_p[:2] - block_p[:2])
            if dist > 0.03:
                # Only consider it if it's not grapsed (i.e. not too close to the gripper).
                blocks_with_dists.append((block, dist))

        if len(blocks_with_dists) == 0:
            return 1000.0

        closest_block, closest_dist = min(blocks_with_dists, key=lambda v: v[1])
        return closest_dist

    def step(self):
        """
        Step the state.

        Returns:
            DfState object.
        """
        pose = self.context.robot.arm.get_fk_pq()
        if pose.p[2] >= self.success_z:
            return None

        if self.cautious_command_delta_z is not None and self.closest_non_grasped_block_dist(pose.p) < 0.1:
            # Use the cautious command delta-z if it's specified and we're close to another block.
            pose.p[2] += self.cautious_command_delta_z
        else:
            pose.p[2] += self.command_delta_z

        self.context.robot.arm.send_end_effector(target_pose=pose)
        return self

    def exit(self):
        # On exit, reset the posture config back to the default value.
        self.context.robot.arm.set_posture_config_to_default()


class PickBlockRd(DfStateMachineDecider, DfRldsNode):
    def __init__(self):
        # This behavior uses the locking feature of the decision framework to run a state machine
        # sequence as an atomic unit.
        super().__init__(
            DfStateSequence(
                [
                    DfSetLockState(set_locked_to=True, decider=self),
                    DfTimedDeciderState(MyDfCloseGripper(), activity_duration=1),
                    LiftState(command_delta_z=0.3, cautious_command_delta_z=0.03, success_delta_z=0.075),
                    DfWriteContextState(lambda ctx: ctx.mark_block_in_gripper()),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )
        self.is_locked = False

    def is_runnable(self):
        # Check if the block is in the gripper.
        ct = self.context
        if ct.has_active_block and ct.active_block.has_chosen_grasp:
            grasp_T = ct.active_block.chosen_grasp
            eff_T = self.context.robot.arm.get_fk_T()

            thresh_met = math_util.transforms_are_close(grasp_T, eff_T, p_thresh=0.005, R_thresh=0.005)
            return thresh_met

        return False


def make_pick_rlds():
    """
    Make the RLDS for picking a block.

    Returns:
        DfRldsDecider object.
    """
    rlds = DfRldsDecider()

    reach_to_block_rd = ReachToBlockRd()
    choose_block = ChooseNextBlockForTowerBuildUp()
    approach_grasp = DfApproachGrasp()

    reach_to_block_rd.link_to("choose_block", choose_block)
    choose_block.link_to("approach_grasp", approach_grasp)

    rlds.append_rlds_node("reach_to_block", reach_to_block_rd)
    rlds.append_rlds_node("pick_block", PickBlockRd())

    return rlds

class ReachToPlaceOnTower(DfDecider):
    """
    Decider to reach to a place on the tower.

    Attributes:
        None
    """
    def __init__(self):
        """
        Initialize the decider.
        """
        super().__init__()
        self.add_child("approach_grasp", DfApproachGrasp())

    def decide(self):
        """
        Decide the next action.

        Returns:
            DfDecision object.
        """
        ct = self.context

        drop_point = ct.active_block.drop_point

        ct.placement_target_eff_T = calc_grasp_for_top_of_tower(ct, drop_point)
        return DfDecision("approach_grasp", ct.placement_target_eff_T)

    def exit(self):
        self.context.placement_target_eff_T = None


class ReachToPlacementRd(DfRldsNode):
    """
    RLDS node to reach to a placement.

    Attributes:
        None
    """
    def __init__(self):
        """
        Initialize the RLDS node.
        """
        super().__init__()
        self.add_child("reach_to_place_on_tower", ReachToPlaceOnTower())

    def is_runnable(self):
        """
        Check if the behavior is runnable.

        Returns:
            Boolean value.
        """
        return self.context.gripper_has_block

    def enter(self):
        """
        Enter the behavior.
        """
        self.context.placement_target_eff_T = None

    def decide(self):
        """
        Decide the next action.

        Returns:
            DfDecision object.
        """
        ct = self.context

        return DfDecision("reach_to_place_on_tower")


class PlaceBlockRd(DfStateMachineDecider, DfRldsNode):
    """
    RLDS node to place a block.

    Attributes:
        is_locked: Whether the behavior is locked.
    """
    def __init__(self):
        """
        Initialize the RLDS node.
        """
        # This behavior uses the locking feature of the decision framework to run a state machine
        # sequence as an atomic unit.
        super().__init__(
            DfStateSequence(
                [
                    DfSetLockState(set_locked_to=True, decider=self),
                    DfTimedDeciderState(MyDfOpenGripper(), activity_duration=1),
                    LiftState(command_delta_z=0.1, success_delta_z=0.03),
                    DfWriteContextState(lambda ctx: ctx.clear_gripper()),
                    DfSetLockState(set_locked_to=False, decider=self),
                ]
            )
        )
        self.is_locked = False

    def is_runnable(self):
        """
        Check if the behavior is runnable.

        Returns:
            Boolean value.
        """
        ct = self.context
        if ct.gripper_has_block and ct.has_placement_target_eff_T:
            eff_T = ct.robot.arm.get_fk_T()

            thresh_met = math_util.transforms_are_close(
                ct.placement_target_eff_T, eff_T, p_thresh=0.05, R_thresh=0.05
            )

            if thresh_met:
                print("<placing block>")
            return thresh_met

        return False

    def exit(self):
        """
        Exit the behavior.
        """
        self.context.reset_active_block()
        self.context.placement_target_eff_T = None


def make_place_rlds():
    """
    Make the RLDS for placing a block.

    Returns:
        DfRldsDecider object.
    """
    rlds = DfRldsDecider()
    rlds.append_rlds_node("reach_to_placement", ReachToPlacementRd())
    rlds.append_rlds_node("place_block", PlaceBlockRd())
    return rlds

class DoNothing(DfState):
    """
    State to do nothing.
    """
    def enter(self):
        """
        Enter the state.
        """
        self.context.robot.arm.clear()

    def step(self):
        """
        Step the state.
        """
        return self

class BlockPickAndPlaceDispatch(DfDecider):
    """
    Decider to dispatch block pick and place.

    Attributes:
        None
    """
    def __init__(self):
        """
        Initialize the decider.
        """
        super().__init__()
        self.add_child("pick", make_pick_rlds())
        self.add_child("place", make_place_rlds())
        self.add_child("do_nothing", DfStateMachineDecider(DoNothing()))

    def decide(self):
        """
        Decide the next action.

        Returns:
            DfDecision object.
        """
        ct: BuildTowerContext = self.context

        global can_run_robot

        if can_run_robot:
            if ct.is_gripper_clear:
                return DfDecision("pick")
            else:
                return DfDecision("place")
        else:
            return DfDecision("do_nothing")


def make_decider_network(robot, dropPointAPos):
    """
    Make the decider network.

    Args:
        robot: MotionCommandedRobot object.
        dropPointAPos: Position of drop point A.

    Returns:
        DfNetwork object.
    """
    return DfNetwork(
        BlockPickAndPlaceDispatch(),
        context=BuildTowerContext(robot, dropPointAPos)
    )
