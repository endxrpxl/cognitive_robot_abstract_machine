import logging

from pycram.robot_plans.actions.composite.cleaning import CleanTableAction
from demos.ansgar_bt.helpers.object_helpers import (
    seed_semantic_annotations_on_surface,
    set_color,
    seed_semantic_annotation_on_surface,
)
from demos.ansgar_bt.helpers.setup import setup_context
from pycram.datastructures.enums import Arms
from pycram.plans.factories import execute_single
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    Milk,
    Bottle,
    Apple,
    Orange,
    Lettuce,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale, Color

### Setup
main_context, main_execution_type = setup_context(simulated=True)
world: World = main_context.world


logger = logging.getLogger(__name__)

logger.setLevel(level=logging.DEBUG)

_table: Table = world.get_semantic_annotation_by_name("table")

with world.modify_world():
    milk = Milk.create_with_new_body_in_world(
        name=PrefixedName("milk"),
        world=world,
        scale=Scale(0.1, 0.1, 0.2),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.1, y=1.8, z=0.795, reference_frame=world.root
        ),
    )
    apple = Apple.create_with_new_body_in_world(
        name=PrefixedName("apple"),
        world=world,
        scale=Scale(0.1, 0.1, 0.1),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-0.1, y=1.9, z=0.745, reference_frame=world.root
        ),
    )
    lettuce = Lettuce.create_with_new_body_in_world(
        name=PrefixedName("lettuce"),
        world=world,
        scale=Scale(0.1, 0.1, 0.1),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.15, y=2.1, z=0.745, reference_frame=world.root
        ),
    )


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
spawned_items = [milk, apple, lettuce, _orange] + _fridge_items

for spawned_item in spawned_items:
    name = spawned_item.name.name
    if "milk" in name:
        set_color(semantic_annotation=spawned_item, color=Color.WHITE())
    elif "lettuce" in name:
        set_color(semantic_annotation=spawned_item, color=Color.GREEN())
    elif "apple" in name:
        set_color(semantic_annotation=spawned_item, color=Color.RED())
    elif "orange" in name:
        set_color(semantic_annotation=spawned_item, color=Color.ORANGE())

with main_execution_type:
    execute_single(
        CleanTableAction(surface_to_clean=_table, arm=Arms.LEFT),
        context=main_context,
    ).perform()
