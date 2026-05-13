from pycram.datastructures.enums import Arms

import logging
from typing import List

import semantic_digital_twin
from demos.ansgar_bt.helpers.object_helpers import (
    placement_pose_on_surface,
    seed_semantic_annotations_on_surface,
    seed_semantic_annotation_on_surface,
)
from demos.ansgar_bt.helpers.setup import setup_context
from pycram.datastructures.dataclasses import Context
from pycram.motion_executor import ExecutionEnvironment
from pycram.plans.factories import sequential, execute_single
from pycram.robot_plans.actions.composite.transporting import TransportAction
from pycram.robot_plans.actions.core.navigation import NavigateAction
from semantic_digital_twin.reasoning.queries import (
    goal_surface_of_object,
    get_next_object_using_planar_distance,
)
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
    ShelfLayer,
    Table,
    Apple,
    Bottle,
)
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

TABLE_ANNOTATION_NAME = "cooking_table"


class StoringGroceriesDemo:
    def __init__(self, context: Context, execution_type: ExecutionEnvironment):
        self._context = context
        self._execution_type = execution_type
        self._world = context.world
        self._robot_view = context.robot

        self._TABLE_POSE = Pose.from_xyz_quaternion(
            1.28,
            4.35,
            0.0,
            0.0,
            0.0,
            0.728,
            0.684,
            reference_frame=self._world.root,
        )
        self._SHELF_POSE = Pose.from_xyz_quaternion(
            3.655,
            4.639,
            0.0,
            0.0,
            0.0,
            0.027,
            0.999,
            reference_frame=self._world.root,
        )

    def run(self) -> None:
        table: Table = self._world.get_semantic_annotation_by_name(
            TABLE_ANNOTATION_NAME
        )
        shelf_layers = self._world.get_semantic_annotations_by_type(ShelfLayer)

        objects: List[HasRootBody] = get_next_object_using_planar_distance(
            self._robot_view.bodies[0], table, Vector3(z=1)
        ).tolist()

        for obj in objects:
            surface = goal_surface_of_object(obj, shelf_layers)
            place_pose = placement_pose_on_surface(
                surface=surface,
                obj=obj,
            )

            with self._execution_type:
                execute_single(
                    TransportAction(obj.root, place_pose, Arms.LEFT),
                    context=self._context,
                ).perform()

            with self._world.modify_world():
                surface.add_object(obj)


if __name__ == "__main__":
    main_context, main_execution_type = setup_context(simulated=True)
    world = main_context.world

    _table: Table = world.get_semantic_annotation_by_name(TABLE_ANNOTATION_NAME)
    _items = {
        "milk1": (Milk, Scale(0.1, 0.1, 0.2)),
        "bottle1": (Bottle, Scale(0.1, 0.1, 0.2)),
        "apple1": (Apple, Scale(0.05, 0.05, 0.05)),
    }
    seed_semantic_annotations_on_surface(world=world, surface=_table, items=_items)
    shelf_layers = world.get_semantic_annotations_by_type(ShelfLayer)
    seed_semantic_annotation_on_surface(
        world=world,
        surface=shelf_layers[0],
        annotation_class=Milk,
        object_name="milk2",
        scale=Scale(0.1, 0.1, 0.2),
    )

    StoringGroceriesDemo(context=main_context, execution_type=main_execution_type).run()
