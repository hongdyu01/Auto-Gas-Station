from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import sys
import carb

from pxr import Usd, UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.api.objects import DynamicCuboid

import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.prims import SingleArticulation

class RMPFlowController(mg.MotionPolicyController):
    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        attach_gripper: bool = False,
    ) -> None:

        if attach_gripper:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config(
                "UR10", "RMPflowSuction"
            )
        else:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )

class Move_UR10(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        self._goal_points = [np.array([2.5, 0, 0]),
                             np.array([1.4, -2.0, 1.0]),
                             np.array([1.4, -1.0, 1.0])]

        return
    
    def setup_scene(self):
        ########################################################################
        # World Loading
        world = World(stage_units_in_meters=1.0)
        add_reference_to_stage(
            usd_path="/home/rokey/auto_oil_project/assets/default_world.usd",
            prim_path="/World")
        
        ########################################################################
        # Light
        stage = world.stage
        light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
        light_prim.CreateIntensityAttr(1000)
        light_prim.CreateAngleAttr(0.53)

        ########################################################################
        # Assets Root Path
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()

        left_diesel_gun = XFormPrim("/World/left_diesel_gun")

        ####################################################################
        # Add UR10 Robots
        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"

        left_ur10 = add_reference_to_stage(usd_path=asset_path, prim_path="/World/left_ur10")
        left_ur10.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")

        left_gripper = SurfaceGripper(
            end_effector_prim_path="/World/left_ur10/ee_link",
            surface_gripper_path="/World/left_ur10/ee_link/SurfaceGripper"
        )
        my_left_ur10 = world.scene.add(
            SingleManipulator(
                prim_path="/World/left_ur10",
                name="left_ur10",
                end_effector_prim_path="/World/left_ur10/ee_link",
                gripper=left_gripper,
            )
        )

        my_left_ur10.gripper.set_default_state(opened=True)
        world.reset()
    
        my_left_ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))
        return

    def move_point(self, goal_position: np.ndarray, end_effector_orientation: np.ndarray=np.array([0, np.pi/2, 0])) -> bool:

        end_effector_orientation = euler_angles_to_quat(end_effector_orientation)
        target_joint_positions = self.cspace_controller.forward(
            target_end_effector_position=goal_position, 
            target_end_effector_orientation=end_effector_orientation
        )
        self.robots.apply_action(target_joint_positions)
        current_joint_positions = self.robots.get_joint_positions()
        is_reached = np.all(np.abs(current_joint_positions[:7] - target_joint_positions.joint_positions) < 0.001)
        return is_reached


    async def setup_post_load(self):
        self._world = self.get_world()
        self.robots = self._world.scene.get_object("my_ur10")
        self.cspace_controller=RMPFlowController(name="my_ur10_cspace_controller", robot_articulation=self.robots, attach_gripper=True)
        self.task_phase = 0
        self._goal_reached = False
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        if self.task_phase == 0:
            current_goal = self._goal_points[self.task_phase]
            self._goal_reached = self.move_point(current_goal)
            
            if self._goal_reached:
                print("Phase 0 (Pre-grasp) ë„ë‹¬.")
                self.cspace_controller.reset()
                self.task_phase = 1
        elif self.task_phase == 1:
            current_goal = self._goal_points[self.task_phase]
            orientation = np.array([np.pi/2, np.pi, 0])
            self._goal_reached = self.move_point(current_goal, end_effector_orientation=orientation)
            
            if self._goal_reached:
                print("Phase 1 (Grasp) ë„ë‹¬.")
                self.cspace_controller.reset()
                self.task_phase = 2
        elif self.task_phase == 2:
            pass
        return