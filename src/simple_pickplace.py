from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import sys
import carb

from isaacsim.core.api import World

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


class Gripper_UR10:
    def __init__(self) -> None:
        super().__init__()

        self.BROWN = np.array([0.5, 0.2, 0.1])
        self._brown_cube_position = np.array([0.40, 0.0, 0.025])
        self.task_phase = 1
        return

    
    def setup_scene(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()    

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
        ur10 = self.world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
            )
        )
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))
        
        self.world.scene.add(DynamicCuboid(
            prim_path="/World/BrownCube", 
            name="brown_cube",
            position=self._brown_cube_position, 
            scale=np.array([0.05, 0.05, 0.05]), 
            color=self.BROWN
        ))

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
        self.robots = self.world.scene.get_object("my_ur10")
        self._cube = self.world.scene.get_object("brown_cube")
        self.cspace_controller=RMPFlowController(name="my_ur10_cspace_controller", robot_articulation=self.robots, attach_gripper=True)
        self._goal_points = [
            np.array([0.3, 0.3, 0.3]),
            np.array([0.3, -0.3, 0.3]) 
        ]
        self.task_phase = 1
        self._goal_reached = False
        self.world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self.world.play_async()
        return

    def physics_step(self, step_size):
        if self.task_phase == 1:
            cube_position, _ = self._cube.get_world_pose()
            cube_position[2] = 0.045

            self._goal_reached = self.move_point(cube_position)
            if self._goal_reached:
                self.cspace_controller.reset()
                self.task_phase = 2

        elif self.task_phase == 2:
            self.robots.gripper.close()
            self.task_phase = 3

        elif self.task_phase == 3:
            cube_position, _ = self._cube.get_world_pose()
            cube_position[2] = 0.4

            self._goal_reached = self.move_point(cube_position)
            if self._goal_reached:
                self.cspace_controller.reset()
                self.task_phase = 4

        elif self.task_phase == 4:
            self.robots.gripper.open()
            self.task_phase = 5
        return