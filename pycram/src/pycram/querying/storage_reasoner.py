"""
Reasoning about where to physically store an object in the world.

This module scores candidate storage surfaces (e.g. shelves, fridge layers) for a given
object based on environment preference, free space, reachability, and grouping with
similar objects already stored there.

Note: "storage" here means a physical object-placement location, unrelated to robokudo's
MongoDB/CAS-backed "storage" I/O layer (``robokudo.io.storage``).
"""

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from krrood.entity_query_language.factories import (
    entity,
    variable,
    count,
    type_,
    the,
    an,
)
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.locations.locations import CostmapLocation
from semantic_digital_twin.reasoning.predicates import inheritance_path_length_
from semantic_digital_twin.semantic_annotations.mixins import (
    HasSupportingSurface,
    HasRootBody,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    StorageObject,
)
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose

logger = logging.getLogger(__name__)
logger.setLevel(level=logging.DEBUG)


@dataclass
class StorageReasonerResult:
    """
    Outcome of scoring a single candidate storage surface for a single object.
    """

    surface: HasSupportingSurface
    """
    The candidate storage surface this result is about.
    """

    score: float
    """
    Higher is better. 0.0 means the surface should not be used.
    """

    satisfied_constraints: List[str]
    """
    Names of the constraints that passed.
    """

    violated_constraints: List[str]
    """
    Names of the constraints that failed.
    """

    poses: List[Pose] = field(default=None)
    """
    Reachable candidate poses on the surface for placing the object, if any.
    """

    def __repr__(self):
        result = f"surface={self.surface.name.name}, score={self.score}, satisfied_constraints={self.satisfied_constraints}, violated_constraints={self.violated_constraints}"
        return f"{self.__class__.__name__}({result})"


class StorageReasonerConstraints(Enum):
    """
    Constraints checked by the StorageReasoner when scoring a storage surface for an object.
    """

    STORAGE_ENVIRONMENT = "storage environment"
    FREE_SPACE = "free space"
    REACHABLE = "reachable"
    SIMILAR_OBJECTS = "similar objects"


def _filter_points_full_on_surface(
    points: List[Point3], obj: HasRootBody, surface: HasSupportingSurface
) -> List[Point3]:
    """
    Filters the given points to only include those where the whole object would be on the surface if placed at that point.

    :param points: List of points to filter
    :param obj: Object to filter for
    :param surface: Surface to filter on
    :return: Filtered list of points
    """
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


@dataclass
class StorageReasoner:
    """
    Finds and scores storage surfaces in the world for storing a given object.
    """

    context: Context = field(repr=False)
    """Plan execution context; provides the ``world`` this reasoner queries."""

    def __post_init__(self):
        self.world = self.context.world

        surface_in_world = variable(
            HasSupportingSurface, self.world.semantic_annotations
        )
        self.storages: List[HasSupportingSurface] = (
            entity(surface_in_world).where(surface_in_world.use_as_storage).tolist()
        )

        self.constraints = [c.value for c in StorageReasonerConstraints]

    def reason_for_object(
        self, storage_object: StorageObject, arm: Optional[Arms] = None
    ) -> List[StorageReasonerResult]:
        """
        Scores every known storage surface for the given object.

        :param storage_object: The object to find a storage location for.
        :param arm: Arm to check reachability with, passed through to the reachability check.
        :return: One :class:`StorageReasonerResult` per known storage surface.
        """

        results: List[StorageReasonerResult] = []

        for storage in self.storages:
            results.append(self._reason_storage(storage_object, storage, arm))
        logger.debug(f"Reasoning results for {storage_object.name}: {results}")
        return results

    def _reason_storage(
        self,
        storage_object: StorageObject,
        storage: HasSupportingSurface,
        arm: Optional[Arms] = None,
    ) -> StorageReasonerResult:
        """
        Checks a single storage surface against all :class:`StorageReasonerConstraints` for the given object.
        The first three constraints are gates: failing one immediately returns a zero-score result with only that constraint listed as violated.
        The last constraint only ever adds to the score.

        1. ``STORAGE_ENVIRONMENT``: the surface's storage environment must equal the object's preferred storage environment.
        2. ``FREE_SPACE``: at least one point sampled from the surface must fit the object's whole footprint.
        3. ``REACHABLE``: at least one free-space point (up to 10 checked) must be reachable by
           ``arm``. Each reachable point adds 0.1 to the score.
        4. ``SIMILAR_OBJECTS``: objects already on the surface with an inheritance distance
           <= 1 to ``storage_object`` add their count to the score.

        :param storage_object: The object to check this surface for.
        :param storage: The candidate storage surface.
        :param arm: Arm to check reachability with.
        :return: The calculated :class:`StorageReasonerResult` for this surface.
        """

        satisfied_constraints = []
        score = 0.0

        # check environment
        if storage.storage_environment != storage_object.preferred_storage_environment:
            return StorageReasonerResult(
                surface=storage,
                score=score,
                satisfied_constraints=[],
                violated_constraints=[
                    StorageReasonerConstraints.STORAGE_ENVIRONMENT.value
                ],
            )

        satisfied_constraints.append(
            StorageReasonerConstraints.STORAGE_ENVIRONMENT.value
        )

        # free space
        unfiltered_positions = storage.sample_points_from_surface(
            body_to_sample_for=storage_object
        )
        positions = _filter_points_full_on_surface(
            points=unfiltered_positions, obj=storage_object, surface=storage
        )
        if not positions:
            return StorageReasonerResult(
                surface=storage,
                score=score,
                satisfied_constraints=satisfied_constraints,
                violated_constraints=[StorageReasonerConstraints.FREE_SPACE.value],
            )
        satisfied_constraints.append(StorageReasonerConstraints.FREE_SPACE.value)

        # check reachable
        poses: List[Pose] = []
        for position in positions:
            if len(poses) >= 10:
                break
            pose = Pose(position=position, reference_frame=position.reference_frame)
            place_loc = CostmapLocation(
                target=pose,
                reachable_arm=arm,
                reachable=True,
                context=self.context,
            )
            costmap = place_loc.setup_costmaps(
                target=place_loc.target,
                visible=place_loc.visible,
                reachable=place_loc.reachable,
            )
            place_pose = place_loc.ground()
            if place_pose:
                poses.append(pose)

        if not poses:
            return StorageReasonerResult(
                surface=storage,
                score=score,
                satisfied_constraints=satisfied_constraints,
                violated_constraints=[StorageReasonerConstraints.REACHABLE.value],
            )

        score += len(poses) * 0.1
        satisfied_constraints.append(StorageReasonerConstraints.REACHABLE.value)

        # check similar objects
        object_on_surface = variable(
            StorageObject,
            domain=storage.objects,
        )
        similarity_threshold = 1
        count_similar_objects = the(
            entity(count(object_on_surface)).where(
                inheritance_path_length_(type(storage_object), type_(object_on_surface))
                <= similarity_threshold
            )
        ).first()
        if count_similar_objects > 0:
            score += count_similar_objects
            satisfied_constraints.append(
                StorageReasonerConstraints.SIMILAR_OBJECTS.value
            )

        return StorageReasonerResult(
            surface=storage,
            score=score,
            satisfied_constraints=satisfied_constraints,
            violated_constraints=[
                c for c in self.constraints if c not in satisfied_constraints
            ],
            poses=poses,
        )

    def select_usable_results(
        self, storage_object: StorageObject, arm: Optional[Arms] = None
    ) -> List[StorageReasonerResult]:
        """
        Scores every known storage surface for the given object and keeps only the usable ones.

        :param storage_object: The object to find a storage location for.
        :param arm: Arm to check reachability with.
        :return: Results with ``score > 0.0``, ordered best (highest score) first.
        """
        result = variable(
            type_=StorageReasonerResult,
            domain=self.reason_for_object(storage_object, arm),
        )
        query = an(
            entity(result)
            .where(result.score > 0.0)
            .ordered_by(result.score, descending=True)
        )

        return query.tolist()

    def select_best_result(
        self, storage_object: StorageObject, arm: Optional[Arms] = None
    ) -> StorageReasonerResult | None:
        """
        Convenience wrapper around :meth:`select_usable_results` returning only the top result.

        :param storage_object: The object to find a storage location for.
        :param arm: Arm to check reachability with.
        :return: The highest-scoring usable result, or ``None`` if no surface is usable.
        """
        results = self.select_usable_results(storage_object, arm)
        return results[0] if results else None
