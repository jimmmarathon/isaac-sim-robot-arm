import os
import asyncio
import json
from dataclasses import dataclass
from typing import Callable, Optional

import carb.settings
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .constants import (
    EXTENSION_ID,
    GRIP1_CLOSE_L,
    GRIP1_CLOSE_R,
    GRIP1_OPEN_L,
    GRIP1_OPEN_R,
    GRIP2_CLOSE_L,
    GRIP2_CLOSE_R,
    GRIP2_OPEN_L,
    GRIP2_OPEN_R,
    GRIP_STEPS,
    HOLD_FRAMES,
    LEFT_GRIPPER_IDX,
    MOVE_STEPS,
    RELEASE_AFTER_FRAMES,
    RELEASE_SETTLE_FRAMES,
    RIGHT_GRIPPER_IDX,
    SAFE_WAIT_HOLD_FRAMES,
    STATUS_CLOSING_LID,
    STATUS_COMPLETED,
    STATUS_ERROR,
    STATUS_IDLE,
    STATUS_OPENING_LID,
    STATUS_PLACING_OBJECT,
    STATUS_PREPARING,
)


@dataclass
class RunnerConfig:
    json_path: str
    arm1_prim_path: str
    arm1_ee_path: str
    arm2_prim_path: str
    arm2_ee_path: str
    goods_path: str
    lid_path: str

    @staticmethod
    def from_settings() -> "RunnerConfig":
        settings = carb.settings.get_settings()
        base = f"/exts/{EXTENSION_ID}"
        return RunnerConfig(
            json_path=settings.get(f"{base}/json_path"),
            arm1_prim_path=settings.get(f"{base}/arm1_prim_path"),
            arm1_ee_path=settings.get(f"{base}/arm1_ee_path"),
            arm2_prim_path=settings.get(f"{base}/arm2_prim_path"),
            arm2_ee_path=settings.get(f"{base}/arm2_ee_path"),
            goods_path=settings.get(f"{base}/goods_path"),
            lid_path=settings.get(f"{base}/lid_path"),
        )


class DualArmTaskController:
    def __init__(self):
        self.config = RunnerConfig.from_settings()

        self.status = STATUS_IDLE
        self.current_step = "-"
        self.last_error = "None"
        self.is_running = False
        self.log_text = ""

        self._on_status: Optional[Callable[[str], None]] = None
        self._on_step: Optional[Callable[[str], None]] = None
        self._on_error: Optional[Callable[[str], None]] = None
        self._on_running_changed: Optional[Callable[[bool], None]] = None
        self._on_log: Optional[Callable[[str], None]] = None

        self._stop_requested = False
        self._lid_attached = False
        self._goods_attached = False
        self._initial_object_poses = {}
        self._initial_physics_state = {}

        self._reset_runtime_state()

    def _reset_runtime_state(self):
        self.box_zone_lock = asyncio.Lock()
        self.lid_opened_event = asyncio.Event()
        self.goods_ready_event = asyncio.Event()
        self.goods_placed_event = asyncio.Event()

    def reload_config(self):
        self.config = RunnerConfig.from_settings()

    def clear_log(self):
        self.log_text = ""
        if self._on_log:
            self._on_log(self.log_text)

    def log(self, msg: str):
        print(msg)
        self.log_text = f"{self.log_text}\n{msg}".strip() if self.log_text else msg
        if self._on_log:
            self._on_log(self.log_text)

    def append_log(self, msg: str):
        self.log(msg)

    def set_ui_hooks(
        self,
        on_status: Optional[Callable[[str], None]] = None,
        on_step: Optional[Callable[[str], None]] = None,
        on_error: Optional[Callable[[str], None]] = None,
        on_running_changed: Optional[Callable[[bool], None]] = None,
        on_log: Optional[Callable[[str], None]] = None,
    ):
        self._on_status = on_status
        self._on_step = on_step
        self._on_error = on_error
        self._on_running_changed = on_running_changed
        self._on_log = on_log

    def set_status(self, value: str):
        self.status = value
        if self._on_status:
            self._on_status(value)

    def set_step(self, value: str):
        self.current_step = value
        if self._on_step:
            self._on_step(value)

    def set_error(self, value: str):
        self.last_error = value
        if self._on_error:
            self._on_error(value)

    def _set_running(self, value: bool):
        self.is_running = value
        if self._on_running_changed:
            self._on_running_changed(value)

    def stop(self):
        self._stop_requested = True
        self._set_running(False)
        self.log("[task] stop requested")

    def ensure_playing(self):
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()
            self.log("[timeline] play")

    def get_stage(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("沒拿到 USD stage")
        return stage

    def get_prim(self, path: str):
        prim = self.get_stage().GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Prim 不存在: {path}")
        return prim

    def load_points(self, path: str):
        if not os.path.isabs(path):
            base_dir = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "../../..")  # 這邊先用相對路徑讀 JSON
            )
            path = os.path.join(base_dir, path)

        path = os.path.normpath(path)

        if not os.path.exists(path):
            raise FileNotFoundError(f"taught_points.json not found: {path}")

        print("[DEBUG] taught_points path =", path)

        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def get_robot(self, prim_path: str, name: str):
        robot = SingleArticulation(prim_path=prim_path, name=name)
        robot.initialize()
        return robot

    def read_q(self, robot):
        q = robot.get_joint_positions()
        if q is None:
            raise RuntimeError("讀不到 joint positions")
        return np.array(q, dtype=np.float32)

    def point6(self, points: dict, key: str):
        if key not in points:
            raise KeyError(f"教導點位不存在: {key}")
        return np.array(points[key]["arm_joints_rad"], dtype=np.float32)

    def build_target_q(self, robot, points, key, grip_open_r, grip_open_l, grip_close_r, grip_close_l, gripper_open=None):
        current = self.read_q(robot)
        target = np.array(current, dtype=np.float32)

        if len(target) < 8:
            raise RuntimeError(f"目前 DOF={len(target)}，程式預期至少 8 軸")

        target[:6] = self.point6(points, key)

        if gripper_open is not None:
            if gripper_open:
                target[RIGHT_GRIPPER_IDX] = grip_open_r
                target[LEFT_GRIPPER_IDX] = grip_open_l
            else:
                target[RIGHT_GRIPPER_IDX] = grip_close_r
                target[LEFT_GRIPPER_IDX] = grip_close_l
        return target

    def _check_stop_requested(self):
        if self._stop_requested:
            raise asyncio.CancelledError("Task stopped by user")

    async def wait_one_frame(self):
        self._check_stop_requested()
        await omni.kit.app.get_app().next_update_async()
        self._check_stop_requested()

    async def hold_frames(self, n=HOLD_FRAMES):
        for _ in range(n):
            await self.wait_one_frame()

    def gf_quat_to_np(self, q):
        imag = q.GetImaginary()
        return np.array([q.GetReal(), imag[0], imag[1], imag[2]], dtype=np.float64)

    def np_to_gf_quatd(self, arr):
        return Gf.Quatd(float(arr[0]), Gf.Vec3d(float(arr[1]), float(arr[2]), float(arr[3])))

    def get_world_matrix(self, path: str):
        prim = self.get_prim(path)
        xformable = UsdGeom.Xformable(prim)
        return xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def get_world_pose(self, path: str):
        mat = self.get_world_matrix(path)
        pos = mat.ExtractTranslation()
        quat = mat.ExtractRotationQuat()
        pos_np = np.array([pos[0], pos[1], pos[2]], dtype=np.float64)
        quat_np = self.gf_quat_to_np(quat)
        return pos_np, quat_np

    def print_pose(self, path: str, label: str):
        pos, quat = self.get_world_pose(path)
        self.log(f"[pose] {label} pos={pos.tolist()}, quat={quat.tolist()}")

    def get_or_add_translate_op(self, xformable):
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op
        return xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)

    def get_or_add_orient_op(self, xformable):
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                return op
        return xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

    def set_world_pose(self, path: str, world_pos, world_quat):
        prim = self.get_prim(path)
        xformable = UsdGeom.Xformable(prim)

        parent = prim.GetParent()
        if parent and parent.IsValid() and str(parent.GetPath()) != "/":
            parent_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            parent_world_inv = parent_world.GetInverse()
        else:
            parent_world_inv = Gf.Matrix4d(1.0)

        world_mat = Gf.Matrix4d(1.0)
        world_mat.SetRotate(self.np_to_gf_quatd(world_quat))
        world_mat.SetTranslateOnly(Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2])))

        local_mat = parent_world_inv * world_mat
        local_pos = local_mat.ExtractTranslation()
        local_quat = local_mat.ExtractRotationQuat()

        t_op = self.get_or_add_translate_op(xformable)
        r_op = self.get_or_add_orient_op(xformable)

        if t_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            t_op.Set(Gf.Vec3f(float(local_pos[0]), float(local_pos[1]), float(local_pos[2])))
        else:
            t_op.Set(Gf.Vec3d(float(local_pos[0]), float(local_pos[1]), float(local_pos[2])))

        imag = local_quat.GetImaginary()
        real = local_quat.GetReal()
        if r_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            r_op.Set(Gf.Quatf(float(real), Gf.Vec3f(float(imag[0]), float(imag[1]), float(imag[2]))))
        else:
            r_op.Set(Gf.Quatd(float(real), Gf.Vec3d(float(imag[0]), float(imag[1]), float(imag[2]))))

    def compute_attach_offset(self, obj_path: str, ee_path: str):
        obj_pos, obj_quat = self.get_world_pose(obj_path)
        ee_pos, _ = self.get_world_pose(ee_path)
        offset_pos = obj_pos - ee_pos
        return offset_pos, obj_quat

    def follow_object(self, obj_path: str, ee_path: str, offset_pos, obj_quat):
        ee_pos, _ = self.get_world_pose(ee_path)
        new_pos = ee_pos + np.array(offset_pos, dtype=np.float64)
        self.set_world_pose(obj_path, new_pos, obj_quat)

    def _get_or_create_attr(self, api_obj, getter_name: str, creator_name: str):
        getter = getattr(api_obj, getter_name, None)
        creator = getattr(api_obj, creator_name, None)
        attr = getter() if getter else None
        if attr is None or not attr.IsValid():
            attr = creator() if creator else None
        return attr

    def _get_rigidbody_attrs(self, obj_path: str):
        prim = self.get_prim(obj_path)
        stage = self.get_stage()

        rb_api = UsdPhysics.RigidBodyAPI.Get(stage, prim.GetPath())
        if not rb_api:
            rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)

        physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Get(stage, prim.GetPath())
        if not physx_rb_api:
            physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)

        rb_enabled_attr = self._get_or_create_attr(rb_api, "GetRigidBodyEnabledAttr", "CreateRigidBodyEnabledAttr")
        kinematic_attr = self._get_or_create_attr(rb_api, "GetKinematicEnabledAttr", "CreateKinematicEnabledAttr")
        disable_gravity_attr = self._get_or_create_attr(physx_rb_api, "GetDisableGravityAttr", "CreateDisableGravityAttr")

        return prim, rb_enabled_attr, kinematic_attr, disable_gravity_attr

    def _capture_initial_object_states_if_needed(self):
        for obj_path in [self.config.lid_path, self.config.goods_path]:
            if not obj_path:
                continue

            if obj_path not in self._initial_object_poses:
                pos, quat = self.get_world_pose(obj_path)
                self._initial_object_poses[obj_path] = {
                    "position": np.array(pos, dtype=np.float64),
                    "orientation": np.array(quat, dtype=np.float64),
                }
                self.log(f"[reset] captured initial pose: {obj_path}")

            if obj_path not in self._initial_physics_state:
                _, rb_enabled_attr, kinematic_attr, disable_gravity_attr = self._get_rigidbody_attrs(obj_path)
                self._initial_physics_state[obj_path] = {
                    "rigid_enabled": rb_enabled_attr.Get() if rb_enabled_attr and rb_enabled_attr.IsValid() else True,
                    "kinematic_enabled": kinematic_attr.Get() if kinematic_attr and kinematic_attr.IsValid() else False,
                    "disable_gravity": disable_gravity_attr.Get() if disable_gravity_attr and disable_gravity_attr.IsValid() else False,
                }
                self.log(f"[reset] captured initial physics: {obj_path}")

    def set_attached_physics(self, obj_path: str, attached: bool):
        _, rb_enabled_attr, kinematic_attr, disable_gravity_attr = self._get_rigidbody_attrs(obj_path)

        if attached:
            self.log(f"[physics] ATTACH {obj_path} -> rigid=True, kinematic=True, gravity=False")
            if rb_enabled_attr and rb_enabled_attr.IsValid():
                rb_enabled_attr.Set(True)
            if kinematic_attr and kinematic_attr.IsValid():
                kinematic_attr.Set(True)
            if disable_gravity_attr and disable_gravity_attr.IsValid():
                disable_gravity_attr.Set(True)
        else:
            self.log(f"[physics] RELEASE {obj_path} -> rigid=True, kinematic=False, gravity=True")
            if rb_enabled_attr and rb_enabled_attr.IsValid():
                rb_enabled_attr.Set(True)
            if kinematic_attr and kinematic_attr.IsValid():
                kinematic_attr.Set(False)
            if disable_gravity_attr and disable_gravity_attr.IsValid():
                disable_gravity_attr.Set(False)

        rb_val = rb_enabled_attr.Get() if rb_enabled_attr and rb_enabled_attr.IsValid() else None
        k_val = kinematic_attr.Get() if kinematic_attr and kinematic_attr.IsValid() else None
        g_val = disable_gravity_attr.Get() if disable_gravity_attr and disable_gravity_attr.IsValid() else None
        self.log(f"[physics-check] rigid={rb_val}, kinematic={k_val}, disable_gravity={g_val}")

    def restore_object_physics(self, obj_path: str):
        self._capture_initial_object_states_if_needed()
        state = self._initial_physics_state.get(obj_path)
        if state is None:
            return

        _, rb_enabled_attr, kinematic_attr, disable_gravity_attr = self._get_rigidbody_attrs(obj_path)

        if rb_enabled_attr and rb_enabled_attr.IsValid():
            rb_enabled_attr.Set(bool(state["rigid_enabled"]))
        if kinematic_attr and kinematic_attr.IsValid():
            kinematic_attr.Set(bool(state["kinematic_enabled"]))
        if disable_gravity_attr and disable_gravity_attr.IsValid():
            disable_gravity_attr.Set(bool(state["disable_gravity"]))

        self.log(
            f"[physics] restore {obj_path} -> "
            f"rigid={state['rigid_enabled']}, kinematic={state['kinematic_enabled']}, disable_gravity={state['disable_gravity']}"
        )

    def restore_lid_physics_if_needed(self):
        self.restore_object_physics(self.config.lid_path)
        self._lid_attached = False

    def restore_goods_physics_if_needed(self):
        self.restore_object_physics(self.config.goods_path)
        self._goods_attached = False

    def detach_lid_if_needed(self):
        if self._lid_attached:
            self.log("[reset] detach lid")
        self._lid_attached = False

    def detach_goods_if_needed(self):
        if self._goods_attached:
            self.log("[reset] detach goods")
        self._goods_attached = False

    def zero_rigidbody_velocity(self, obj_path: str):
        prim = self.get_prim(obj_path)

        vel_attr = prim.GetAttribute("physics:velocity")
        ang_attr = prim.GetAttribute("physics:angularVelocity")

        if vel_attr and vel_attr.IsValid():
            vel_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        if ang_attr and ang_attr.IsValid():
            ang_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

        physx_lin = prim.GetAttribute("physxRigidBody:linearVelocity")
        physx_ang = prim.GetAttribute("physxRigidBody:angularVelocity")
        if physx_lin and physx_lin.IsValid():
            physx_lin.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        if physx_ang and physx_ang.IsValid():
            physx_ang.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def zero_all_object_velocities(self):
        for obj_path in [self.config.lid_path, self.config.goods_path]:
            try:
                self.zero_rigidbody_velocity(obj_path)
                self.log(f"[reset] zero velocity: {obj_path}")
            except Exception as exc:
                self.log(f"[warn] zero velocity failed for {obj_path}: {exc}")

    def reset_objects(self):
        self._capture_initial_object_states_if_needed()

        for obj_path, pose in self._initial_object_poses.items():
            self.set_world_pose(obj_path, pose["position"], pose["orientation"])
            self.log(f"[reset] object pose restored: {obj_path}")

    def move_robots_home(self):
        points = self.load_points(self.config.json_path)

        arm1 = self.get_robot(self.config.arm1_prim_path, "arm1_reset_ctrl")
        arm2 = self.get_robot(self.config.arm2_prim_path, "arm2_reset_ctrl")

        q1 = self.build_target_q(
            arm1, points, "arm1_home",
            GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L, True
        )
        q2 = self.build_target_q(
            arm2, points, "arm2_home",
            GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, True
        )

        arm1.set_joint_positions(q1)
        arm2.set_joint_positions(q2)
        self.log("[reset] robots moved to home")

    def open_grippers(self):
        points = self.load_points(self.config.json_path)

        arm1 = self.get_robot(self.config.arm1_prim_path, "arm1_grip_reset")
        arm2 = self.get_robot(self.config.arm2_prim_path, "arm2_grip_reset")

        q1 = self.build_target_q(
            arm1, points, "arm1_home",
            GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L, True
        )
        q2 = self.build_target_q(
            arm2, points, "arm2_home",
            GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, True
        )

        arm1.set_joint_positions(q1)
        arm2.set_joint_positions(q2)
        self.log("[reset] grippers opened")

    async def release_with_delay(self, obj_path: str):
        await self.hold_frames(RELEASE_SETTLE_FRAMES)
        self.set_attached_physics(obj_path, attached=False)
        if obj_path == self.config.lid_path:
            self._lid_attached = False
        if obj_path == self.config.goods_path:
            self._goods_attached = False
        await self.hold_frames(max(RELEASE_AFTER_FRAMES, 20))

    async def move_joint_smooth_async(
        self,
        robot,
        target_q,
        steps=MOVE_STEPS,
        attached_obj_path=None,
        ee_path=None,
        offset_pos=None,
        obj_quat=None,
    ):
        self._check_stop_requested()
        current = self.read_q(robot)
        target_q = np.array(target_q, dtype=np.float32)

        for i in range(1, steps + 1):
            self._check_stop_requested()
            a = i / steps
            q = (1.0 - a) * current + a * target_q
            robot.set_joint_positions(q)

            if attached_obj_path is not None and ee_path is not None and offset_pos is not None and obj_quat is not None:
                self.follow_object(attached_obj_path, ee_path, offset_pos, obj_quat)

            await self.wait_one_frame()

    async def set_gripper_async(self, robot, is_open: bool, grip_open_r, grip_open_l, grip_close_r, grip_close_l, steps=GRIP_STEPS):
        self._check_stop_requested()
        q = self.read_q(robot)
        q = np.array(q, dtype=np.float32)
        if is_open:
            q[RIGHT_GRIPPER_IDX] = grip_open_r
            q[LEFT_GRIPPER_IDX] = grip_open_l
        else:
            q[RIGHT_GRIPPER_IDX] = grip_close_r
            q[LEFT_GRIPPER_IDX] = grip_close_l
        await self.move_joint_smooth_async(robot, q, steps=steps)

    async def arm2_open_lid_and_move_aside(self, arm2, points):
        cfg = self.config
        self.set_status(STATUS_OPENING_LID)
        self.set_step("Arm2 opening lid")
        self.log("[task] arm2 open lid start")

        for key in ["arm2_home", "arm2_lid_above", "arm2_lid_pick"]:
            self.log(f"[arm2] move -> {key}")
            await self.move_joint_smooth_async(
                arm2,
                self.build_target_q(arm2, points, key, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, True),
            )
            await self.hold_frames()

        self.log("[arm2] close gripper on lid")
        await self.set_gripper_async(arm2, False, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L)
        await self.hold_frames()

        self.set_attached_physics(cfg.lid_path, attached=True)
        self._lid_attached = True
        await self.hold_frames(5)
        lid_offset_pos, lid_quat = self.compute_attach_offset(cfg.lid_path, cfg.arm2_ee_path)

        for key in ["arm2_lid_above", "arm2_lid_open_place_above", "arm2_lid_open_place"]:
            self.log(f"[arm2] move with lid -> {key}")
            await self.move_joint_smooth_async(
                arm2,
                self.build_target_q(arm2, points, key, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, False),
                attached_obj_path=cfg.lid_path,
                ee_path=cfg.arm2_ee_path,
                offset_pos=lid_offset_pos,
                obj_quat=lid_quat,
            )
            await self.hold_frames()

        self.log("[arm2] open gripper release lid aside")
        await self.set_gripper_async(arm2, True, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L)
        await self.hold_frames(3)
        await self.release_with_delay(cfg.lid_path)

        for key in ["arm2_lid_open_place_above", "arm2_safe_wait"]:
            self.log(f"[arm2] retreat -> {key}")
            await self.move_joint_smooth_async(
                arm2,
                self.build_target_q(arm2, points, key, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, True),
            )
            await self.hold_frames()

        self.log("[task] arm2 lid opened and moved aside")
        self.lid_opened_event.set()

    async def arm1_prepare_goods(self, arm1, points):
        cfg = self.config
        self.set_step("Arm1 preparing goods")
        self.log("[task] arm1 prepare goods start")

        for key in ["arm1_home", "arm1_goods_above", "arm1_goods_pick"]:
            self.log(f"[arm1] move -> {key}")
            await self.move_joint_smooth_async(
                arm1,
                self.build_target_q(arm1, points, key, GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L, True),
            )
            await self.hold_frames()

        self.log("[arm1] close gripper on goods")
        await self.set_gripper_async(arm1, False, GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L)
        await self.hold_frames()

        self.set_attached_physics(cfg.goods_path, attached=True)
        self._goods_attached = True
        await self.hold_frames(5)
        goods_offset_pos, goods_quat = self.compute_attach_offset(cfg.goods_path, cfg.arm1_ee_path)

        for key in ["arm1_goods_above", "arm1_safe_wait"]:
            self.log(f"[arm1] move with goods -> {key}")
            await self.move_joint_smooth_async(
                arm1,
                self.build_target_q(arm1, points, key, GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L, False),
                attached_obj_path=cfg.goods_path,
                ee_path=cfg.arm1_ee_path,
                offset_pos=goods_offset_pos,
                obj_quat=goods_quat,
            )
            await self.hold_frames()

        self.log("[task] arm1 goods ready")
        self.goods_ready_event.set()

    async def arm1_place_goods_into_box(self, arm1, points):
        cfg = self.config
        await self.lid_opened_event.wait()
        await self.goods_ready_event.wait()

        async with self.box_zone_lock:
            self.set_status(STATUS_PLACING_OBJECT)
            self.set_step("Arm1 placing object into box")
            self.log("[task] arm1 place goods start")

            goods_offset_pos, goods_quat = self.compute_attach_offset(cfg.goods_path, cfg.arm1_ee_path)

            for key in ["arm1_box_above", "arm1_box_place"]:
                self.log(f"[arm1] move with goods -> {key}")
                await self.move_joint_smooth_async(
                    arm1,
                    self.build_target_q(arm1, points, key, GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L, False),
                    attached_obj_path=cfg.goods_path,
                    ee_path=cfg.arm1_ee_path,
                    offset_pos=goods_offset_pos,
                    obj_quat=goods_quat,
                )
                await self.hold_frames()

            self.log("[arm1] open gripper release goods")
            await self.set_gripper_async(arm1, True, GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L)
            await self.hold_frames(3)
            await self.release_with_delay(cfg.goods_path)

            for key in ["arm1_box_above", "arm1_safe_wait", "arm1_home"]:
                self.log(f"[arm1] retreat -> {key}")
                await self.move_joint_smooth_async(
                    arm1,
                    self.build_target_q(arm1, points, key, GRIP1_OPEN_R, GRIP1_OPEN_L, GRIP1_CLOSE_R, GRIP1_CLOSE_L, True),
                )
                await self.hold_frames()

            self.log("[task] arm1 goods placed")
            self.goods_placed_event.set()

    async def arm2_pick_and_close_lid(self, arm2, points):
        cfg = self.config
        await self.goods_placed_event.wait()

        self.set_status(STATUS_CLOSING_LID)
        self.set_step("Arm2 picking lid from side")
        self.log("[task] arm2 close lid start")

        for key in ["arm2_lid_open_place_above", "arm2_lid_open_place"]:
            self.log(f"[arm2] move -> {key}")
            await self.move_joint_smooth_async(
                arm2,
                self.build_target_q(arm2, points, key, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, True),
            )
            await self.hold_frames()

        self.log("[arm2] close gripper pick lid again")
        await self.set_gripper_async(arm2, False, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L)
        await self.hold_frames()

        self.set_attached_physics(cfg.lid_path, attached=True)
        self._lid_attached = True
        await self.hold_frames(5)
        lid_offset_pos, lid_quat = self.compute_attach_offset(cfg.lid_path, cfg.arm2_ee_path)

        async with self.box_zone_lock:
            self.set_step("Arm2 closing lid on box")
            for key in ["arm2_lid_open_place_above", "arm2_box_cover_above", "arm2_box_cover_place"]:
                self.log(f"[arm2] move with lid -> {key}")
                await self.move_joint_smooth_async(
                    arm2,
                    self.build_target_q(arm2, points, key, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, False),
                    attached_obj_path=cfg.lid_path,
                    ee_path=cfg.arm2_ee_path,
                    offset_pos=lid_offset_pos,
                    obj_quat=lid_quat,
                )
                await self.hold_frames()

            self.log("[arm2] open gripper place lid back")
            await self.set_gripper_async(arm2, True, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L)
            await self.hold_frames(3)

            self.print_pose(cfg.lid_path, "Lid before final release")
            self.log("[lid] keep lid at cover place before release")
            await self.hold_frames(10)

            self.log("[lid] restoring physics for final cover")
            self.set_attached_physics(cfg.lid_path, attached=False)
            self._lid_attached = False
            await self.hold_frames(1)
            self.print_pose(cfg.lid_path, "Lid just after release")

            self.log("[lid] waiting for lid to settle onto box")
            await self.hold_frames(20)
            self.print_pose(cfg.lid_path, "Lid after settle")

        for key in ["arm2_box_cover_above", "arm2_safe_wait", "arm2_home"]:
            self.log(f"[arm2] retreat -> {key}")
            await self.move_joint_smooth_async(
                arm2,
                self.build_target_q(arm2, points, key, GRIP2_OPEN_R, GRIP2_OPEN_L, GRIP2_CLOSE_R, GRIP2_CLOSE_L, True),
            )
            await self.hold_frames()

        self.log("[task] arm2 lid closed")

    async def run_full_task(self):
        if self.is_running:
            return

        self.clear_log()
        self._set_running(True)
        self._stop_requested = False
        self._reset_runtime_state()
        self.set_error("None")

        try:
            self.reload_config()
            self.set_status(STATUS_PREPARING)
            self.set_step("Validate timeline, stage, prims, and point file")
            self.log("[task] run_full_task start")

            self.ensure_playing()
            await self.hold_frames(2)

            for path in [
                self.config.arm1_prim_path,
                self.config.arm1_ee_path,
                self.config.arm2_prim_path,
                self.config.arm2_ee_path,
                self.config.goods_path,
                self.config.lid_path,
            ]:
                self.get_prim(path)
                self.log(f"[check] ok: {path}")

            self._capture_initial_object_states_if_needed()

            points = self.load_points(self.config.json_path)
            self.log(f"[check] points loaded: {self.config.json_path}")
            arm1 = self.get_robot(self.config.arm1_prim_path, "arm1_ctrl")
            arm2 = self.get_robot(self.config.arm2_prim_path, "arm2_ctrl")
            self.log("[check] robot controllers initialized")

            await asyncio.gather(
                self.arm2_open_lid_and_move_aside(arm2, points),
                self.arm1_prepare_goods(arm1, points),
            )

            await asyncio.gather(
                self.arm1_place_goods_into_box(arm1, points),
                self.arm2_pick_and_close_lid(arm2, points),
            )

            self.set_status(STATUS_COMPLETED)
            self.set_step("Task finished")
            self.log("[task] finished")
            await self.hold_frames(SAFE_WAIT_HOLD_FRAMES)
            self.set_status(STATUS_IDLE)
            self.set_step("-")

        except asyncio.CancelledError:
            self.set_status(STATUS_IDLE)
            self.set_step("Task stopped")
            self.log("[task] cancelled by user")
        except Exception as exc:
            self.set_status(STATUS_ERROR)
            self.set_error(str(exc))
            self.set_step("Task aborted")
            self.log(f"[error] {exc}")
            raise
        finally:
            self._set_running(False)
