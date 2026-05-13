from typing import Type, List, Sequence

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale


def move_object_to_new_pose(
    semantic_annotation: HasRootBody, new_transform: HomogeneousTransformationMatrix
):
    world: World = semantic_annotation._world
    new_transform_world = world.transform(new_transform, world.root)
    parent_connection = semantic_annotation.root.parent_connection
    parent_connection_parent = parent_connection.parent
    parent_connection_child = parent_connection.child
    new_transform_world.reference_frame = parent_connection_parent
    new_transform_world.child_frame = parent_connection_child
    new_parent_connection = FixedConnection(
        parent=parent_connection_parent,
        child=parent_connection_child,
        parent_T_connection_expression=new_transform_world,
    )
    world.remove_connection(parent_connection)
    world.add_connection(new_parent_connection)


PLACEMENT_Z_OFFSET = 0.005


def seed_semantic_annotation_on_surface(
    world: World,
    surface: HasSupportingSurface,
    annotation_class: Type[HasRootBody],
    object_name: str,
    scale: Scale,
) -> HasRootBody:

    with world.modify_world():
        obj = annotation_class.create_with_new_body_in_world(
            name=PrefixedName(object_name),
            world=world,
            scale=scale,
        )

    sampled = surface.sample_points_from_surface(body_to_sample_for=obj)
    filtered = filter_points_full_on_surface(sampled, obj, surface)
    candidates = filtered or sampled

    if not candidates:
        raise RuntimeError(
            f"Demo setup: no sample point on {surface!r} for {object_name}"
        )

    point = candidates[0]
    point.z -= PLACEMENT_Z_OFFSET  # Offset so object touches the surface. Needed for queries to work

    with world.modify_world():
        move_object_to_new_pose(
            semantic_annotation=obj,
            new_transform=HomogeneousTransformationMatrix.from_point_rotation_matrix(
                point=point, reference_frame=point.reference_frame
            ),
        )

    with world.modify_world():
        surface.add_object(obj)

    return obj


def seed_semantic_annotations_on_surface(
    world: World,
    surface: HasSupportingSurface,
    items: dict[str, tuple[Type[HasRootBody], Scale]],
) -> List[HasRootBody]:
    objs = []
    for name, t in items.items():
        annotation_type = t[0]
        scale = t[1]
        obj = seed_semantic_annotation_on_surface(
            world, surface, annotation_type, name, scale
        )
        objs.append(obj)

    return objs


def filter_points_full_on_surface(
    points: Sequence[Point3], obj: HasRootBody, surface: HasSupportingSurface
) -> List[Point3]:
    obj_min, obj_max = obj.min_max_points
    surf_min, surf_max = surface.min_max_points

    return [
        point
        for point in points
        if (
            surf_min.x <= point.x + obj_min.x <= surf_max.x
            and surf_min.x <= point.x + obj_max.x <= surf_max.x
            and surf_min.y <= point.y + obj_min.y <= surf_max.y
            and surf_min.y <= point.y + obj_max.y <= surf_max.y
        )
    ]


def placement_pose_on_surface(
    *,
    surface: HasSupportingSurface,
    obj: HasRootBody,
) -> Pose:
    raw = surface.sample_points_from_surface(obj)
    on_surface = filter_points_full_on_surface(raw, obj, surface)
    candidates = on_surface or raw

    if not candidates:
        raise ValueError(
            f"No placement samples for object {obj!r} on surface {surface!r}"
        )

    point = candidates[0]
    point.z -= PLACEMENT_Z_OFFSET

    return Pose(position=point, reference_frame=point.reference_frame)
