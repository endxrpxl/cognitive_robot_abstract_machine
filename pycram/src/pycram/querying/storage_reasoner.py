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
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    StorageObject,
)
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass
class StorageReasonerResult:
    surface: HasSupportingSurface
    score: float
    satisfied_constraints: List[str]
    violated_constraints: List[str]
    poses: List[Pose] = field(default=None)

    def __repr__(self):
        result = f"surface={self.surface.name.name}, score={self.score}, satisfied_constraints={self.satisfied_constraints}, violated_constraints={self.violated_constraints}"
        return f"{self.__class__.__name__}({result})"


class StorageReasonerConstraints(Enum):
    STORAGE_ENVIRONMENT = "storage environment"
    FREE_SPACE = "free space"
    REACHABLE = "reachable"
    SIMILAR_OBJECTS = "similar objects"


@dataclass
class StorageReasoner:

    context: Context = field(repr=False)

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

        results: List[StorageReasonerResult] = []

        for storage in self.storages:
            results.append(self._reason_storage(storage_object, storage, arm))
        return results

    def _reason_storage(
        self,
        storage_object: StorageObject,
        storage: HasSupportingSurface,
        arm: Optional[Arms] = None,
    ) -> StorageReasonerResult:

        satisfied_constraints = []
        score = 0.0

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
        positions = storage.sample_points_from_surface(
            body_to_sample_for=storage_object,
            amount=20,
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
        results = self.select_usable_results(storage_object, arm)
        return results[0] if results else None
