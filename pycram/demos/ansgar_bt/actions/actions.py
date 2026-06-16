from dataclasses import dataclass, field
from typing import Optional, List

from krrood.entity_query_language.factories import variable, underspecified
from pycram.datastructures.enums import Arms
from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.composite.transporting import TransportAction
from semantic_digital_twin.reasoning.queries import (
    get_next_object_using_planar_distance,
    storages_with_environment_for_object,
    sort_surfaces_by_most_similar_objects_to_object,
)
from semantic_digital_twin.semantic_annotations.mixins import (
    HasSupportingSurface,
)
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.spatial_types.spatial_types import Pose


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
            storages_with_environment = storages_with_environment_for_object(obj)
            sorted_storages, empty_storages = (
                sort_surfaces_by_most_similar_objects_to_object(
                    storages_with_environment, obj
                )
            )
            storages_to_try: List[HasSupportingSurface] = (
                sorted_storages.tolist() + empty_storages.tolist()
            )
            poses_to_try = []
            for storage in storages_to_try:
                points = storage.sample_points_from_surface(obj)
                for point in points:
                    poses_to_try.append(
                        Pose(position=point, reference_frame=point.reference_frame)
                    )
            transport_action = underspecified(TransportAction)(
                semantic_annotation=obj,
                target_location=variable(Pose, poses_to_try),
                arm=self.arm,
            )
            self.add_subplan(execute_single(transport_action)).perform()
