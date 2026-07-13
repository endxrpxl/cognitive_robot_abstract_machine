import os

import rclpy

from ansgar_bt.demo_world import DemoEnvironment
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import Body


def _build_hsrb_world():
    hsrb_urdf = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../resources/robots/hsrb.urdf")
    )
    world = URDFParser.from_file(file_path=hsrb_urdf).parse()
    with world.modify_world():
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        omni_drive = OmniDrive.create_with_dofs(
            parent=odom, child=world.root, world=world
        )
        world.add_connection(omni_drive)
        omni_drive.has_hardware_interface = True
    return world


def demo_setup(with_fridge: bool = True):
    rclpy.init()
    rclpy_node = rclpy.create_node("bt_demo")

    hsrb_world = _build_hsrb_world()
    env_world = DemoEnvironment().get_world(with_fridge=with_fridge)

    env_world.merge_world_at_pose(
        hsrb_world,
        HomogeneousTransformationMatrix(),
    )
    robot_view = HSRB.from_world(env_world)

    viz = VizMarkerPublisher(_world=env_world, node=rclpy_node).with_tf_publisher()

    return Context(
        world=env_world,
        robot=robot_view,
        ros_node=rclpy_node,
    )
