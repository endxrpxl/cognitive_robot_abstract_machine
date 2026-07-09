import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List

from pycram.datastructures.enums import Arms
from pycram.plans.factories import execute_single
from pycram.plans.failures import ConfigurationNotReached, BodyUnfetchable
from pycram.querying.storage_reasoner import StorageReasoner, StorageReasonerResult
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.composite.transporting import TransportAction
from semantic_digital_twin.reasoning.queries import (
    get_next_object_using_planar_distance,
)
from semantic_digital_twin.semantic_annotations.mixins import (
    HasSupportingSurface,
    StorageEnvironments,
    HasRootBody,
)
from semantic_digital_twin.spatial_types import Vector3

logger = logging.getLogger(__name__)


class PickUpStrategy(Enum):
    """
    Order in which :class:`CleanSurfaceAction` picks up objects from the surface it is cleaning.
    """

    NEAREST_FIRST = auto()
    """
    Pick up objects in order of planar distance to the robot, closest first.
    """

    COLD_FIRST = auto()
    """
    Pick up objects that prefer cold storage first, then normal; objects with the same preferred storage environment
    keep their nearest-first relative order.
    """


@dataclass
class CleanSurfaceAction(ActionDescription):
    """
    Cleans the surface by transporting all objects that are currently on the surface to their best storage location based on the StorageReasoner.
    """

    surface_to_clean: HasSupportingSurface = field(repr=False)
    """
    Surface to clean
    """

    arm: Optional[Arms]
    """
    Arm that should be used
    """

    pick_up_strategy: PickUpStrategy = PickUpStrategy.NEAREST_FIRST
    """
    Strategy to use for pick up.
    """

    def _order_objects(self) -> List[HasRootBody]:
        """
        Returns the objects currently on :attr:`surface_to_clean`, ordered according to
        :attr:`pick_up_strategy`.

        :return: Objects on the surface, in the order they should be picked up and stored.
        """
        match self.pick_up_strategy:
            case PickUpStrategy.NEAREST_FIRST:
                return get_next_object_using_planar_distance(
                    main_body=self.robot.root,
                    supporting_surface=self.surface_to_clean,
                    ignore_dimension=Vector3(z=1.0),
                ).tolist()
            case PickUpStrategy.COLD_FIRST:
                objs_ordered_by_distance = get_next_object_using_planar_distance(
                    main_body=self.robot.root,
                    supporting_surface=self.surface_to_clean,
                    ignore_dimension=Vector3(z=1.0),
                ).tolist()

                env_priority = {
                    StorageEnvironments.COLD: 0,
                    StorageEnvironments.NORMAL: 1,
                    StorageEnvironments.WARM: 2,
                }

                return sorted(
                    objs_ordered_by_distance,
                    key=lambda obj: env_priority.get(
                        obj.preferred_storage_environment, 3
                    ),
                )

    def execute(self) -> None:
        """
        Orders the objects on :attr:`surface_to_clean` per :attr:`pick_up_strategy`, then
        transports each one in turn to its best storage location, as determined by a fresh
        :class:`~pycram.querying.storage_reasoner.StorageReasoner` built from the current plan
        context.

        :raises RuntimeError: if an object has usable storage candidates but transporting it
            to all of them fails at runtime (see :meth:`_transport_object`).
        """
        objects_on_surface = self._order_objects()
        storage_reasoner = StorageReasoner(self.plan.context)

        for obj in objects_on_surface:
            self._transport_object(obj, storage_reasoner)

    def _transport_object(
        self, obj: HasRootBody, storage_reasoner: StorageReasoner
    ) -> None:
        """
        Stores a single object using the reasoner's ranked storage candidates.

        Queries ``storage_reasoner`` for usable storage results for ``obj``, and tries them
        best-first via :meth:`_try_solution` until one succeeds. If the reasoner finds no
        usable storage at all, logs a warning and leaves the object where it is (this is not
        treated as an error). If usable candidates exist but every one of them fails at
        runtime, raises ``RuntimeError``.

        :param obj: The object to transport off the surface.
        :param storage_reasoner: Reasoner used to find and rank candidate storage locations.
        :raises RuntimeError: if all candidate solutions fail to be reached/placed.
        """
        possible_solutions = storage_reasoner.select_usable_results(
            storage_object=obj, arm=self.arm
        )
        if not possible_solutions:
            logger.warning(f"No storage solutions found for object {obj.name.name}")
            return

        stored = False
        for solution in possible_solutions:
            if self._try_solution(obj, solution):
                stored = True
                break

        if not stored:
            raise RuntimeError(
                f"Could not store object {obj.name.name}: All candidates failed."
            )

    def _try_solution(self, obj: HasRootBody, solution: StorageReasonerResult) -> bool:
        """
        Tries to transport ``obj`` to one candidate storage surface, attempting each of the
        solution's candidate poses in order until one succeeds.

        ``ConfigurationNotReached`` and ``BodyUnfetchable`` are treated as expected, recoverable
        failures for a given pose and trigger trying the next pose. Any other exception is
        logged (with traceback) and also treated as a soft failure, so a single bad pose does
        not abort the whole cleaning run.

        :param obj: The object to transport.
        :param solution: A candidate storage result (surface + reachable poses) to try.
        :return: ``True`` if the object was successfully transported to this surface, ``False``
            if every pose for this surface failed.
        """
        poses = solution.poses
        for pose in poses:
            transport_action = TransportAction(
                object_designator=obj.root, target_location=pose, arm=self.arm
            )

            try:
                self.add_subplan(execute_single(transport_action)).perform()
                logger.debug(
                    f"Successfully transported {obj.name.name} to {solution.surface.name.name}"
                )
                return True
            except (ConfigurationNotReached, BodyUnfetchable) as e:
                logger.debug(
                    f"Pose failed for object {obj.name.name} on surface {solution.surface.name.name}: {e}. Trying next pose."
                )
                continue
            except Exception as e:
                logger.exception(
                    f"Unexpected error while transporting {obj.name.name}: {e}. Trying next pose/surface."
                )
                continue

        logger.debug(
            f"All poses for surface {solution.surface.name.name} failed at runtime for object {obj.name.name}. Trying next candidate."
        )
        return False
