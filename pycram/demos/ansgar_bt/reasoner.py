from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from krrood.entity_query_language.factories import (
    entity,
    variable,
    an,
    count,
    type_,
    the,
)
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.datastructures.grasp import GraspDescription
from pycram.locations.locations import CostmapLocation
from pycram.plans.failures import BodyUnfetchable
from semantic_digital_twin.reasoning.predicates import inheritance_path_length_
from semantic_digital_twin.reasoning.queries import semantic_annotations_on_surface
from semantic_digital_twin.semantic_annotations.mixins import (
    HasStorageSpace,
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
    pose_grasp: Optional[List[Tuple[Pose, CostmapLocation]]] = field(default=None)

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

    def reason_for_object(
        self, storage_object: StorageObject, arm: Optional[Arms] = None
    ) -> List[StorageReasonerResult]:

        results: List[StorageReasonerResult] = []

        constraints = [c.value for c in StorageReasonerConstraints]

        for storage in self.storages:

            satisfied_constraints = []
            score = 0.0

            if (
                storage.storage_environment
                != storage_object.preferred_storage_environment
            ):
                results.append(
                    StorageReasonerResult(
                        surface=storage,
                        score=score,
                        satisfied_constraints=[],
                        violated_constraints=[
                            StorageReasonerConstraints.STORAGE_ENVIRONMENT.value
                        ],
                    )
                )
                continue
            satisfied_constraints.append(
                StorageReasonerConstraints.STORAGE_ENVIRONMENT.value
            )

            # free space
            positions = storage.sample_points_from_surface(
                body_to_sample_for=storage_object,
                category_of_interest=type(storage_object),
            )
            if not positions:
                results.append(
                    StorageReasonerResult(
                        surface=storage,
                        score=score,
                        satisfied_constraints=satisfied_constraints,
                        violated_constraints=[
                            StorageReasonerConstraints.FREE_SPACE.value
                        ],
                    )
                )
                continue
            satisfied_constraints.append(StorageReasonerConstraints.FREE_SPACE.value)

            # check reachable
            positions_locations: List[Tuple[Point3, CostmapLocation]] = []
            for position in positions:
                if len(positions_locations) >= 10:
                    break

                place_loc = CostmapLocation(
                    target=Pose(
                        position=position, reference_frame=position.reference_frame
                    ),
                    reachable_arm=arm,
                    reachable=True,
                    context=self.context,
                )
                place_pose = place_loc.ground()
                if place_pose:
                    positions_locations.append((position, place_pose))

            if not positions_locations:
                results.append(
                    StorageReasonerResult(
                        surface=storage,
                        score=score,
                        satisfied_constraints=satisfied_constraints,
                        violated_constraints=[
                            StorageReasonerConstraints.REACHABLE.value
                        ],
                    )
                )
                continue

            score += len(positions_locations) * 0.1
            satisfied_constraints.append(StorageReasonerConstraints.REACHABLE.value)

            # check similar objects
            object_on_surface = variable(
                StorageObject,
                domain=storage.objects,
            )
            similarity_threshold = 1
            count_similar_objects = the(
                entity(count(object_on_surface)).where(
                    inheritance_path_length_(
                        type(storage_object), type_(object_on_surface)
                    )
                    <= similarity_threshold
                )
            ).first()
            if count_similar_objects > 0:
                score += count_similar_objects
                satisfied_constraints.append(
                    StorageReasonerConstraints.SIMILAR_OBJECTS.value
                )

            results.append(
                StorageReasonerResult(
                    surface=storage,
                    score=score,
                    satisfied_constraints=satisfied_constraints,
                    violated_constraints=[
                        c for c in constraints if c not in satisfied_constraints
                    ],
                    pose_grasp=positions_locations,
                )
            )

        return results

    # def get_highest_score_result(self) -> StorageReasonerResult:
    #     results = self.reason_for_object()
