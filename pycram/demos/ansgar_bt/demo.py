from demos.ansgar_bt.actions.actions import FreeSurfaceAction
from demos.ansgar_bt.helpers.object_helpers import (
    seed_semantic_annotations_on_surface,
    set_color,
)
from demos.ansgar_bt.helpers.setup import setup_context
from pycram.datastructures.enums import Arms
from pycram.plans.factories import execute_single
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.reasoning.predicates import InsideOf, contact
from semantic_digital_twin.reasoning.queries import preferred_surface_for_object
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Fridge,
    ShelfLayer,
    CounterTop,
    Table,
    Milk,
    Bottle,
    Apple,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale, Color

### Setup
main_context, main_execution_type = setup_context(simulated=True)
world: World = main_context.world

_table: Table = world.get_semantic_annotation_by_name("cooking_table")
_items = {
    "milk": (Milk, Scale(0.1, 0.1, 0.2)),
    "bottle": (Bottle, Scale(0.1, 0.1, 0.2)),
    "apple": (Apple, Scale(0.05, 0.05, 0.05)),
}
spawned_items = seed_semantic_annotations_on_surface(
    world=world, surface=_table, items=_items
)
for spawned_item in spawned_items:
    name = spawned_item.name.name
    if "milk" in name:
        set_color(semantic_annotation=spawned_item, color=Color.WHITE())
    elif "bottle" in name:
        set_color(semantic_annotation=spawned_item, color=Color.BLUE())
    elif "apple" in name:
        set_color(semantic_annotation=spawned_item, color=Color.RED())

print(spawned_items[0].preferred_storage_location)


for spawned_item in spawned_items:
    pref = preferred_surface_for_object(spawned_item).tolist()
    print(pref)

###
with main_execution_type:
    execute_single(
        FreeSurfaceAction(surface_to_clean=_table, arm=Arms.LEFT), context=main_context
    ).perform()
