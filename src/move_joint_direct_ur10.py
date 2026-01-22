import numpy as np
import sys
import carb

from isaacsim.examples.interactive.base_sample import BaseSample

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction


class Move_Joint_Direct_UR10(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        
        self._target_joint_positions = None

    
    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()    

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()
        
        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )
        ur10 = world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
            )
        )
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

    async def setup_post_load(self):
        self._world = self.get_world()
        self._ur10 = self._world.scene.get_object("my_ur10")
        
        self.task_phase = 1

        arm_target_deg_1 = np.array([-265, -51, 93.0, -31.0, 20.0, 26.0])
        arm_target_deg_2 = np.array([0.0, -20.0, 10.0, 20.0, 0.0, 0.0])

        self._target_joint_positions_1 = np.deg2rad(arm_target_deg_1)
        self._target_joint_positions_2 = np.deg2rad(arm_target_deg_2)
        
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()

    def move_joint_direct(self, target_joint_positions: np.ndarray) -> bool:
        action = ArticulationAction(joint_positions=target_joint_positions)
        self._ur10.apply_action(action)

        current_joint_positions = self._ur10.get_joint_positions()

        is_reached = np.all(
            np.abs(current_joint_positions[:6] - target_joint_positions[:6]) < 0.01
        )

        return is_reached

    def physics_step(self, step_size):
        if self.task_phase == 1:
            if self.move_joint_direct(self._target_joint_positions_1):
                self.task_phase = 2
        elif self.task_phase == 2:
            if self.move_joint_direct(self._target_joint_positions_2):
                self.task_phase = 3 
        elif self.task_phase == 3:
            pass
        return