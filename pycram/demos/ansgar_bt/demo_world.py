from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    Fridge,
    Cupboard,
    ShelfLayer,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale, Color
from semantic_digital_twin.world_description.world_entity import Body


class DemoEnvironment:
    """
    Simple demo environment for demonstration purposes.
    """

    def get_world(self) -> World:
        world = World()
        root = Body(name=PrefixedName("root"))
        with world.modify_world():
            world.add_body(root)

        with world.modify_world():
            cupboard_scale = Scale(0.5, 0.80, 2.0)

            cupboard = Cupboard.create_with_new_body_in_world(
                name=PrefixedName("cupboard"),
                world=world,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2.0, y=1.0, z=1.0
                ),
                scale=cupboard_scale,
                wall_thickness=0.02,
            )

            shelf_1 = ShelfLayer.create_with_new_body_in_world(
                name=PrefixedName("shelf_1"),
                world=world,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2.0, y=1.0, z=0.5
                ),
                scale=Scale(0.45, 0.75, 0.02),
            )
            shelf_1.use_as_storage = True

            shelf_2 = ShelfLayer.create_with_new_body_in_world(
                name=PrefixedName("shelf_2"),
                world=world,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2.0, y=1.0, z=1.0
                ),
                scale=Scale(0.45, 0.75, 0.02),
            )
            shelf_2.use_as_storage = True

            for color in (
                cupboard.bodies[0].visual.shapes
                + shelf_1.bodies[0].visual.shapes
                + shelf_2.bodies[0].visual.shapes
            ):
                color.color = Color.BEIGE()

            refrigerator = Fridge.create_with_new_body_in_world(
                world=world,
                name=PrefixedName("refrigerator"),
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2.0, y=-1.0, z=1.0
                ),
                scale=cupboard_scale,
            )

            refrigerator_layer1 = ShelfLayer.create_with_new_body_in_world(
                world=world,
                name=PrefixedName("fridge_layer1"),
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2.0, y=-1.0, z=1.0
                ),
                scale=Scale(0.45, 0.75, 0.02),
            )
            refrigerator_layer1.use_as_storage = True

            refrigerator_layer2 = ShelfLayer.create_with_new_body_in_world(
                world=world,
                name=PrefixedName("fridge_layer2"),
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2.0, y=-1.0, z=0.5
                ),
                scale=Scale(0.45, 0.75, 0.02),
            )
            refrigerator_layer2.use_as_storage = True

            table = Table.create_with_new_body_in_world(
                world=world,
                name=PrefixedName("table"),
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=0, y=2, z=0.35
                ),
                scale=Scale(0.5, 0.5, 0.7),
            )
            for color in table.bodies[0].visual.shapes:
                color.color = Color.GREY()

        return world
