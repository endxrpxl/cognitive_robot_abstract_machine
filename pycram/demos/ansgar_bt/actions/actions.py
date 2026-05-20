from dataclasses import dataclass, field
from typing import Optional

from demos.ansgar_bt.helpers.object_helpers import placement_poses_on_surface
from pycram.datastructures.enums import Arms
from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.composite.transporting import TransportAction
from semantic_digital_twin.reasoning.queries import (
    preferred_surface_for_object,
    get_next_object_using_planar_distance,
)
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.spatial_types import Vector3


@dataclass
class FreeSurfaceAction(ActionDescription):
    """
    Cleans the surface by transporting all objects that are currently on the surface to their preferred storage location.
    """

    surface_to_clean: HasSupportingSurface = field(repr=False)
    """
    Surface to clean
    """

    arm: Optional[Arms]
    """
    Arm that should be used
    """

    def execute(self) -> None:
        objects_on_surface = get_next_object_using_planar_distance(
            main_body=self.robot.root,
            supporting_surface=self.surface_to_clean,
            ignore_dimension=Vector3(z=1.0),
        ).tolist()
        for obj in objects_on_surface:
            target_surfaces = preferred_surface_for_object(obj).tolist()
            place_poses = placement_poses_on_surface(
                surface=target_surfaces[0],
                obj=obj,
            )

            transport_action = TransportAction(
                semantic_annotation=obj,
                target_location=place_poses[0],
                arm=self.arm,
            )
            self.add_subplan(execute_single(transport_action)).perform()
