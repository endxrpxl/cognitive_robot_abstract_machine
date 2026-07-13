from ansgar_bt.helpers.setup import demo_setup
from demos.ansgar_bt.helpers.object_helpers import (
    seed_semantic_annotations_on_surface,
    seed_semantic_annotation_on_surface,
)
from pycram.datastructures.enums import Arms
from pycram.motion_executor import simulated_robot
from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.composite.cleaning import (
    CleanSurfaceAction,
    PickUpStrategy,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    Milk,
    Orange,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale, Color

main_context = demo_setup()
world: World = main_context.world


_fridge_layer1 = world.get_semantic_annotation_by_name("fridge_layer1")
_fridge_items_dict = {
    "milk11": (Milk, Scale(0.1, 0.1, 0.2)),
    "milk21": (Milk, Scale(0.1, 0.1, 0.2)),
}
_fridge_items = seed_semantic_annotations_on_surface(
    surface=_fridge_layer1, items=_fridge_items_dict, world=world
)

_shelf_2 = world.get_semantic_annotation_by_name("shelf_2")
_orange = seed_semantic_annotation_on_surface(
    world=world,
    surface=_shelf_2,
    annotation_class=Orange,
    scale=Scale(0.1, 0.1, 0.1),
    object_name="orange",
)
for color in _orange.bodies[0].visual.shapes:
    color.color = Color.ORANGE()


_table: Table = world.get_semantic_annotation_by_name("table")

with simulated_robot:
    execute_single(
        CleanSurfaceAction(
            surface_to_clean=_table,
            arm=Arms.LEFT,
            pick_up_strategy=PickUpStrategy.COLD_FIRST,
        ),
        context=main_context,
    ).perform()
