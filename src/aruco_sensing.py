from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--disable_output", action="store_true", help="Disable debug output.")
parser.add_argument("--test", action="store_true", help="Enable test mode (fixed frame count).")
args, _ = parser.parse_known_args()

import isaacsim.core.utils.numpy.rotations as rot_utils
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera

from isaacsim import SimulationApp


import sys

import carb
import math
import numpy as np
from pxr import UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)

asset_path = "/home/rokey/auto_oil_project/assets/aruco_sample.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World")

# Light
stage = my_world.stage

# Remove all existing lights (including referenced ones)
for prim in stage.Traverse():
    if prim.GetTypeName() in ["DistantLight", "SphereLight", "RectLight", "CylinderLight", "DiskLight"]:
        # Create an override first (important for referenced USD)
        override_prim = stage.OverridePrim(prim.GetPath())
        stage.RemovePrim(prim.GetPath())

# Add a default light
light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
light_prim.CreateIntensityAttr(1000)
light_prim.CreateAngleAttr(0.53)

from pxr import UsdGeom

stage.DefinePrim("/World/camera", "XFormPrim")

camera = Camera(
    prim_path="/World/camera/cam",
    position=np.array([-6, -0.5, 1.1]),
    frequency=20,
    resolution=(256, 256),
    orientation=np.array([0.5, 0.5, -0.5, -0.5]),
)

my_world.scene.add_default_ground_plane()
my_world.reset()
camera.initialize()

i = 0
camera.add_motion_vectors_to_frame()
reset_needed = False
while simulation_app.is_running() and (not args.test or i < 601):
    my_world.step(render=True)
    if (i + 1) % 100 == 0:
        imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
        plt.draw()
        plt.savefig(f"camera.frame{i:03d}.png")
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
    i += 1


simulation_app.close()