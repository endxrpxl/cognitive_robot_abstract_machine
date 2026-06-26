from ansgar_bt.reasoner import StorageReasoner
from demos.ansgar_bt.actions.actions import FreeSurfaceAction
from demos.ansgar_bt.helpers.object_helpers import (
    seed_semantic_annotations_on_surface,
    set_color,
)
from demos.ansgar_bt.helpers.setup import setup_context
from pycram.datastructures.enums import Arms
from pycram.plans.factories import execute_single
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    Milk,
    Bottle,
    Apple,
)
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

_layer1 = world.get_semantic_annotation_by_name("fridge_layer1")
_items1 = {
    "milk11": (Milk, Scale(0.1, 0.1, 0.2)),
    "milk21": (Milk, Scale(0.1, 0.1, 0.2)),
}
spawned_items += seed_semantic_annotations_on_surface(
    world=world, surface=_layer1, items=_items1
)

# _layer3 = world.get_semantic_annotation_by_name("fridge_layer21")
# _items3 = {
#     "milk13": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk23": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk33": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk43": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk53": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk63": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk73": (Milk, Scale(0.1, 0.1, 0.2)),
#     "milk83": (Milk, Scale(0.1, 0.1, 0.2)),
# }
# spawned_items += seed_semantic_annotations_on_surface(
#     world=world, surface=_layer3, items=_items3
# )

for spawned_item in spawned_items:
    name = spawned_item.name.name
    if "milk" in name:
        set_color(semantic_annotation=spawned_item, color=Color.WHITE())
    elif "bottle" in name:
        set_color(semantic_annotation=spawned_item, color=Color.BLUE())
    elif "apple" in name:
        set_color(semantic_annotation=spawned_item, color=Color.RED())

###
# with main_execution_type:
#     execute_single(
#         FreeSurfaceAction(surface_to_clean=_table, arm=Arms.LEFT),
#         context=main_context,
#     ).perform()


reasoner = StorageReasoner(context=main_context)
test = reasoner.reason_for_object(spawned_items[0])
for t in test:
    print(t)
