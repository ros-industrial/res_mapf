# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import uuid

import pytest
from res_mapf_planning.traffic_dependencies.models.plan import Plan, PlanId, Waypoint
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_plan_execution.plan_execution.dependency_manager import (
    DependencyManager,
    DispatchedWaypoint,
    ProgressReport,
)


def _uuid_from_str(name, namespace=uuid.UUID("00000000-0000-0000-0000-000000000000")) -> uuid.UUID:
    return uuid.uuid5(namespace, name)


def _alice_plan_id(plan_version: int) -> PlanId:
    return PlanId(_uuid_from_str("alice"), plan_version=plan_version)


def _bob_plan_id(plan_version: int) -> PlanId:
    return PlanId(_uuid_from_str("bob"), plan_version=plan_version)


def _make_alice_plan() -> Plan:
    return Plan(
        plan_id=_alice_plan_id(0),
        waypoints=[
            Waypoint(name="A", position=(0.0, 0.0), progress=0.0),
            Waypoint(name="B", position=(1.0, 0.0), progress=1.0),
            Waypoint(name="C", position=(2.0, 0.0), progress=2.0),
            Waypoint(name="D", position=(3.0, 0.0), progress=3.0),
            Waypoint(name="E", position=(4.0, 0.0), progress=4.0),
            Waypoint(name="F", position=(5.0, 0.0), progress=5.0),
            Waypoint(name="G", position=(6.0, 0.0), progress=6.0),
        ],
    )


def _make_alice_replan(plan_version: int = 1) -> Plan:
    """Replan from D. Progress is offset by D's progress, as PlanGenerator does."""
    return Plan(
        plan_id=_alice_plan_id(plan_version),
        waypoints=[
            Waypoint(name="D", position=(3.0, 0.0), progress=3.0),
            Waypoint(name="H", position=(3.0, 1.0), progress=4.0),
            Waypoint(name="I", position=(3.0, 2.0), progress=5.0),
        ],
    )


def _make_bob_plan(blocker: TrafficDependency) -> Plan:
    """Bob may not depart V until blocker is satisfied."""
    return Plan(
        plan_id=_bob_plan_id(0),
        waypoints=[
            Waypoint(name="V", position=(0.0, 1.0), progress=0.0, departure_blockers=[blocker]),
            Waypoint(name="W", position=(1.0, 1.0), progress=1.0),
        ],
    )


def _summarise(dispatched: list[DispatchedWaypoint]) -> list[tuple[PlanId, int, str]]:
    return [(d.plan_id, d.index, d.waypoint.name) for d in dispatched]


def _alice_at_b_enqueued_to_d(dm: DependencyManager) -> dict[str, DispatchedWaypoint]:
    """Alice on plan v0, reached B, with C and D enqueued. Returns dispatches by name."""
    dm.set_plan("alice", _make_alice_plan())
    dispatched = dm.get_valid_waypoints_and_advance("alice")
    assert dm.update_progress("alice", dispatched[0]), "B was enqueued, must be accepted"
    dispatched += dm.get_valid_waypoints_and_advance("alice")
    return {d.waypoint.name: d for d in dispatched}


def test_dm_basic_flow():
    """
    End-to-end trace through dispatch, progress, commit cut, and a replan
    spliced in after the cut.
    """
    dm = DependencyManager()
    v0, v1 = _alice_plan_id(0), _alice_plan_id(1)
    dm.set_plan("alice", _make_alice_plan())
    print(
        "alice planid:",
        dm.get_ledger("alice").plan_entries[-1].plan_id,
        dm.get_ledger("alice").plan_entries[-1].plan_id.plan_version,
    )

    dispatched = dm.get_valid_waypoints_and_advance("alice")
    assert _summarise(dispatched) == [(v0, 1, "B"), (v0, 2, "C")], (
        "A is the start and is not dispatched; B and C fill the two slots"
    )

    assert dm.update_progress("alice", dispatched[0]), "B must be accepted"
    more = dm.get_valid_waypoints_and_advance("alice")
    assert _summarise(more) == [(v0, 3, "D")], "Reaching B frees exactly one slot"

    assert dm.update_progress("alice", dispatched[1]), "C must be accepted"
    assert dm.update_progress("alice", more[0]), "D must be accepted"

    dm.compute_commit_cut()
    ledger = dm.get_ledger("alice")
    print("Cut index:", ledger.cut_index)
    print("Waypoint at the cut:", ledger.waypoints[ledger.cut_index])
    assert ledger.waypoints[ledger.cut_index].name == "D", (
        "With no blockers, the cut sits at the last enqueued waypoint"
    )

    new_plan = Plan(
        plan_id=v1,
        waypoints=[
            Waypoint(name="H", position=(0.0, 0.0), progress=0.0),
            Waypoint(name="I", position=(1.0, 0.0), progress=1.0),
            Waypoint(name="J", position=(2.0, 0.0), progress=2.0),
            Waypoint(name="K", position=(3.0, 0.0), progress=3.0),
            Waypoint(name="L", position=(4.0, 0.0), progress=4.0),
        ],
    )
    dm.update_plan_after_cut("alice", new_plan)
    print("after updating plan:", dm.get_ledger("alice").waypoints)
    print(
        "new planid:",
        dm.get_ledger("alice").plan_entries[-1].plan_id,
        dm.get_ledger("alice").plan_entries[-1].plan_id.plan_version,
    )
    assert [wp.name for wp in dm.get_ledger("alice").waypoints] == [
        "A", "B", "C", "D", "H", "I", "J", "K", "L",
    ], (
        "New plan's waypoints are appended after the cut; nothing is trimmed "
        "yet since alice has not reached past D into v1's territory"
    )


def test_dispatch_respects_max_enqueued():
    """With max_enqueued = 2, at most two waypoints are enqueued ahead of the robot."""
    dm = DependencyManager()
    v0 = _alice_plan_id(0)
    dm.set_plan("alice", _make_alice_plan())

    first = dm.get_valid_waypoints_and_advance("alice")
    assert _summarise(first) == [(v0, 1, "B"), (v0, 2, "C")], (
        "Start waypoint A is not dispatched; B and C fill the two slots"
    )
    assert dm.get_valid_waypoints_and_advance("alice") == [], (
        "Nothing more may be enqueued until alice reports progress"
    )

    assert dm.update_progress("alice", first[0]), "B was enqueued, must be accepted"
    assert _summarise(dm.get_valid_waypoints_and_advance("alice")) == [(v0, 3, "D")], (
        "Reaching B frees exactly one slot"
    )
    assert dm.get_progress("alice") == ProgressReport(
        plan_id=v0, reached_index=1, target_index=3
    )


def test_update_progress_ignores_reports_it_cannot_place():
    dm = DependencyManager()
    plan = _make_alice_plan()
    dispatched = _alice_at_b_enqueued_to_d(dm)
    before = dm.get_progress("alice")

    never_enqueued = DispatchedWaypoint(plan_id=plan.plan_id, waypoint=plan.waypoints[5], index=5)
    assert not dm.update_progress("alice", never_enqueued), (
        "F was never enqueued, so alice cannot have reached it"
    )

    other_plan = DispatchedWaypoint(plan_id=_alice_plan_id(7), waypoint=plan.waypoints[2], index=2)
    assert not dm.update_progress("alice", other_plan), (
        "A report for a plan that is not in alice's ledger must be ignored"
    )

    assert dm.update_progress("alice", dispatched["C"])
    assert not dm.update_progress("alice", dispatched["C"]), (
        "A duplicate report for the waypoint alice is already at must be ignored"
    )
    assert not dm.update_progress("alice", dispatched["B"]), (
        "A late report for a waypoint behind the robot must not move it backwards"
    )

    assert not dm.update_progress("bob", dispatched["C"]), "bob has no ledger"

    assert dm.get_progress("alice") == ProgressReport(
        plan_id=before.plan_id, reached_index=2, target_index=before.target_index
    ), "Only the accepted report for C may have changed alice's progress"


def test_get_progress_unknown_robot():
    assert DependencyManager().get_progress("alice") is None


def test_replan_keeps_old_plan_until_robot_reaches_cut():
    """
    Replan arrives while alice is between B and the cut at D. Waypoints up to
    D still belong to v0 and must still be accepted; v0 only completes once
    alice reaches D.
    """
    dm = DependencyManager()
    v0, v1 = _alice_plan_id(0), _alice_plan_id(1)
    dispatched = _alice_at_b_enqueued_to_d(dm)

    dm.update_plan_after_cut("alice", _make_alice_replan())

    ledger = dm.get_ledger("alice")
    assert [wp.name for wp in ledger.waypoints] == ["A", "B", "C", "D", "H", "I"], (
        "Waypoints after the cut are replaced; the duplicate D is not repeated"
    )
    assert dm.get_progress("alice") == ProgressReport(
        plan_id=v0, reached_index=1, target_index=3
    ), "alice is still on v0's waypoints, so progress is reported against v0"
    assert v0 not in dm._completed_plan_ids, "alice has not reached the cut yet"

    assert dm.update_progress("alice", dispatched["C"]), (
        "C was dispatched under v0 before the replan and must still be accepted"
    )
    assert _summarise(dm.get_valid_waypoints_and_advance("alice")) == [(v1, 1, "H")], (
        "First waypoint past the cut is the new plan's waypoint 1"
    )

    assert dm.update_progress("alice", dispatched["D"])
    assert v0 in dm._completed_plan_ids, "Reaching the cut finishes v0"
    assert [wp.name for wp in ledger.waypoints] == ["D", "H", "I"], (
        "Waypoints owned only by the finished plan are trimmed"
    )
    assert dm.get_progress("alice") == ProgressReport(
        plan_id=v1, reached_index=0, target_index=1
    ), "At the cut, progress switches to v1 with indices relative to v1"

    assert not dm.update_progress("alice", dispatched["D"]), (
        "A duplicate report for a dropped plan must be ignored"
    )
    assert _summarise(dm.get_valid_waypoints_and_advance("alice")) == [(v1, 2, "I")], (
        "Indices stay relative to v1 after the ledger is trimmed"
    )


def test_replan_arriving_with_robot_already_at_cut_drops_old_plan():
    dm = DependencyManager()
    dispatched = _alice_at_b_enqueued_to_d(dm)
    assert dm.update_progress("alice", dispatched["C"])
    assert dm.update_progress("alice", dispatched["D"])

    dm.update_plan_after_cut("alice", _make_alice_replan())

    assert _alice_plan_id(0) in dm._completed_plan_ids, (
        "alice is already at the cut, so v0 is finished as soon as v1 is spliced"
    )
    assert [wp.name for wp in dm.get_ledger("alice").waypoints] == ["D", "H", "I"]
    assert not dm.is_complete("alice"), "v1 still has waypoints to execute"


def test_replan_transfers_blockers_from_deduplicated_waypoint():
    dm = DependencyManager()
    _alice_at_b_enqueued_to_d(dm)

    blocker = TrafficDependency(name="bob", plan_id=_bob_plan_id(0), required_progress=1.0)
    new_plan = _make_alice_replan()
    new_plan.waypoints[0].departure_blockers.append(blocker)
    dm.update_plan_after_cut("alice", new_plan)

    retained_d = dm.get_ledger("alice").waypoints[3]
    assert retained_d.name == "D"
    assert blocker in retained_d.departure_blockers, (
        "Blocker on the new plan's duplicate D must move to the retained D"
    )


@pytest.mark.xfail(
    strict=True,
    reason="_entry_for_position attributes a non-deduplicated waypoint 0 to the previous plan",
)
def test_replan_not_starting_at_cut_location_is_dispatched_under_new_plan():
    """New plan's waypoint 0 is not the cut location, so nothing is deduplicated."""
    dm = DependencyManager()
    v1 = _alice_plan_id(1)
    dispatched = _alice_at_b_enqueued_to_d(dm)

    new_plan = Plan(
        plan_id=v1,
        waypoints=[
            Waypoint(name="H", position=(3.0, 1.0), progress=4.0),
            Waypoint(name="I", position=(3.0, 2.0), progress=5.0),
        ],
    )
    dm.update_plan_after_cut("alice", new_plan)
    assert [wp.name for wp in dm.get_ledger("alice").waypoints] == [
        "A", "B", "C", "D", "H", "I",
    ]

    assert dm.update_progress("alice", dispatched["C"])
    assert _summarise(dm.get_valid_waypoints_and_advance("alice")) == [(v1, 0, "H")], (
        "H is the new plan's own waypoint 0, not waypoint 4 of the old plan"
    )


def test_second_replan_at_same_cut_replaces_first_without_completing_it():
    dm = DependencyManager()
    v0, v1, v2 = _alice_plan_id(0), _alice_plan_id(1), _alice_plan_id(2)
    _alice_at_b_enqueued_to_d(dm)

    dm.update_plan_after_cut("alice", _make_alice_replan(plan_version=1))
    dm.update_plan_after_cut("alice", _make_alice_replan(plan_version=2))

    ledger = dm.get_ledger("alice")
    assert [entry.plan_id for entry in ledger.plan_entries] == [v0, v2], (
        "v1 never contributed a dispatched waypoint, so it leaves the ledger"
    )
    assert [wp.name for wp in ledger.waypoints] == ["A", "B", "C", "D", "H", "I"]
    assert v1 not in dm._completed_plan_ids, "v1 was replaced, not finished"

    dm.set_plan(
        "bob", _make_bob_plan(TrafficDependency(name="alice", plan_id=v1, required_progress=3.0))
    )
    assert dm.get_valid_waypoints_and_advance("bob") == [], (
        "A blocker on a replaced plan must stay unsatisfied until bob is replanned"
    )


def test_blocker_on_old_plan_is_honoured_across_replan():
    """
    Bob waits for alice to reach D under v0. Alice is replanned at D. Blockers
    are no longer restamped, so bob's blocker still names v0 and must release
    when alice reaches D, and stay released once v0 is dropped from the ledger.
    """
    dm = DependencyManager()
    dispatched = _alice_at_b_enqueued_to_d(dm)
    dm.set_plan(
        "bob",
        _make_bob_plan(
            TrafficDependency(name="alice", plan_id=_alice_plan_id(0), required_progress=3.0)
        ),
    )

    dm.update_plan_after_cut("alice", _make_alice_replan())
    assert dm.get_valid_waypoints_and_advance("bob") == [], (
        "alice is at B (progress 1.0), bob needs her at 3.0"
    )

    assert dm.update_progress("alice", dispatched["C"])
    assert dm.get_valid_waypoints_and_advance("bob") == [], "alice is only at C (progress 2.0)"

    assert dm.update_progress("alice", dispatched["D"])
    assert _summarise(dm.get_valid_waypoints_and_advance("bob")) == [(_bob_plan_id(0), 1, "W")], (
        "alice reached D, bob's blocker on v0 is satisfied"
    )


def test_blocker_on_new_plan_uses_offset_progress():
    """Bob waits for alice to reach H (progress 4.0) under v1."""
    dm = DependencyManager()
    dispatched = _alice_at_b_enqueued_to_d(dm)
    dm.update_plan_after_cut("alice", _make_alice_replan())
    dm.set_plan(
        "bob",
        _make_bob_plan(
            TrafficDependency(name="alice", plan_id=_alice_plan_id(1), required_progress=4.0)
        ),
    )

    assert dm.update_progress("alice", dispatched["C"])
    assert dm.update_progress("alice", dispatched["D"])
    assert dm.get_valid_waypoints_and_advance("bob") == [], (
        "alice is at D (progress 3.0), bob needs her at 4.0"
    )

    h = dm.get_valid_waypoints_and_advance("alice")[0]
    assert dm.update_progress("alice", h)
    assert _summarise(dm.get_valid_waypoints_and_advance("bob")) == [(_bob_plan_id(0), 1, "W")], (
        "alice reached H, bob's blocker on v1 is satisfied"
    )


def test_commit_cut_reports_location_relative_to_owning_plan():
    dm = DependencyManager()
    v0, v1 = _alice_plan_id(0), _alice_plan_id(1)
    dispatched = _alice_at_b_enqueued_to_d(dm)

    cut = dm.compute_commit_cut()
    committed = cut.committed_locations["alice"]
    assert (committed.location, committed.task_id, committed.waypoint_index, committed.progress) == (
        "D", str(v0), 3, 3.0,
    ), "Cut is at the last enqueued waypoint, which v0 owns"
    assert cut.stationary_robots == set(), "alice has waypoints left"

    dm.update_plan_after_cut("alice", _make_alice_replan())
    assert dm.update_progress("alice", dispatched["C"])
    dm.get_valid_waypoints_and_advance("alice")  # enqueues H

    committed = dm.compute_commit_cut().committed_locations["alice"]
    assert (committed.location, committed.task_id, committed.waypoint_index, committed.progress) == (
        "H", str(v1), 1, 4.0,
    ), "H is v1's waypoint 1, even though it sits at position 4 in the ledger"


def test_cut_index_survives_ledger_trim():
    """
    Cut is computed at H, then alice reaches D which trims v0's waypoints off
    the front of the ledger. The next plan must still be spliced after H.
    """
    dm = DependencyManager()
    dispatched = _alice_at_b_enqueued_to_d(dm)
    dm.update_plan_after_cut("alice", _make_alice_replan())
    assert dm.update_progress("alice", dispatched["C"])
    dm.get_valid_waypoints_and_advance("alice")  # enqueues H

    dm.compute_commit_cut()
    assert dm.update_progress("alice", dispatched["D"])  # trims A, B, C

    dm.update_plan_after_cut(
        "alice",
        Plan(
            plan_id=_alice_plan_id(2),
            waypoints=[
                Waypoint(name="H", position=(3.0, 1.0), progress=4.0),
                Waypoint(name="J", position=(4.0, 1.0), progress=5.0),
            ],
        ),
    )
    assert [wp.name for wp in dm.get_ledger("alice").waypoints] == ["D", "H", "J"], (
        "Committed H is retained and I is replaced by J"
    )


def test_stationary_robot_is_reported_in_commit_cut():
    dm = DependencyManager()
    dm.set_plan("alice", Plan(plan_id=_alice_plan_id(0), waypoints=_make_alice_plan().waypoints[:2]))
    (b,) = dm.get_valid_waypoints_and_advance("alice")
    assert dm.update_progress("alice", b)

    assert dm.is_complete("alice")
    assert dm.compute_commit_cut().stationary_robots == {"alice"}


def test_enqueue_rejected_drops_ledger_that_never_ran():
    """If nothing was ever confirmed on this ledger, a rejected enqueue
    drops it entirely so the next plan for this robot starts fresh via
    set_plan, rather than being spliced onto a ledger that never ran."""
    dm = DependencyManager()
    dm.set_plan("alice", _make_alice_plan())
    dm.get_valid_waypoints_and_advance("alice")  # enqueues B, C; nothing confirmed yet

    assert dm.on_enqueue_rejected("alice")

    assert dm.get_ledger("alice") is None
    assert not dm.has_robot("alice")


def test_enqueue_rejected_marks_failure_once_robot_has_progressed():
    """Once the robot has confirmed progress, a rejected enqueue cannot
    simply drop the ledger - other robots may depend on its progress - so
    it is marked failed instead, same as on_plan_failed."""
    dm = DependencyManager()
    _alice_at_b_enqueued_to_d(dm)

    assert dm.on_enqueue_rejected("alice")

    ledger = dm.get_ledger("alice")
    assert ledger is not None
    assert ledger.execution_failed
    assert dm.get_valid_waypoints_and_advance("alice") == [], (
        "Nothing may be dispatched to a robot whose enqueue was rejected mid-plan"
    )


def test_enqueue_rejected_unknown_robot():
    assert not DependencyManager().on_enqueue_rejected("alice")


def test_failed_robot_is_not_dispatched_and_keeps_others_blocked():
    """
    on_plan_failed keeps the robot's ledger but stops dispatching to it. Its
    plan is not completed, so robots blocked on it stay blocked until a
    replan resolves it.
    """
    dm = DependencyManager()
    v0 = _alice_plan_id(0)
    dispatched = _alice_at_b_enqueued_to_d(dm)
    dm.set_plan(
        "bob", _make_bob_plan(TrafficDependency(name="alice", plan_id=v0, required_progress=3.0))
    )

    assert dm.on_plan_failed("alice", v0), "v0 is alice's active plan"
    assert dm.update_progress("alice", dispatched["C"]), (
        "A waypoint dispatched before the failure may still be reported as reached"
    )

    assert dm.get_valid_waypoints_and_advance("alice") == [], (
        "Nothing may be dispatched to a failed robot"
    )
    assert v0 not in dm._completed_plan_ids, "A failed plan is not a completed plan"
    assert dm.get_valid_waypoints_and_advance("bob") == [], (
        "bob must stay blocked on alice's failed plan"
    )


def test_on_plan_failed_ignores_plan_that_is_no_longer_active():
    dm = DependencyManager()
    _alice_at_b_enqueued_to_d(dm)
    dm.update_plan_after_cut("alice", _make_alice_replan())

    assert not dm.on_plan_failed("alice", _alice_plan_id(0)), (
        "v0 has been replanned, a late failure for it must be ignored"
    )
    assert not dm.get_ledger("alice").execution_failed
    assert not dm.on_plan_failed("bob", _bob_plan_id(0)), "bob has no ledger"


def test_new_plan_resumes_dispatch_after_failure():
    dm = DependencyManager()
    v1 = _alice_plan_id(1)
    dispatched = _alice_at_b_enqueued_to_d(dm)
    assert dm.on_plan_failed("alice", _alice_plan_id(0))

    dm.update_plan_after_cut("alice", _make_alice_replan())

    assert dm.update_progress("alice", dispatched["C"])
    assert _summarise(dm.get_valid_waypoints_and_advance("alice")) == [(v1, 1, "H")], (
        "A new plan resolves the failure, so dispatch must resume"
    )


def test_commit_cut_for_failed_robot():
    """
    Alice fails at B with C and D dispatched. She may be anywhere up to D, so
    C and D are possible obstacles. Bob's blocker needs her at F, but a failed
    robot will not move, so her cut must not be extended.
    """
    dm = DependencyManager()
    v0 = _alice_plan_id(0)
    _alice_at_b_enqueued_to_d(dm)
    dm.set_plan(
        "bob", _make_bob_plan(TrafficDependency(name="alice", plan_id=v0, required_progress=5.0))
    )
    assert dm.on_plan_failed("alice", v0)

    cut = dm.compute_commit_cut()

    assert cut.possible_obstacle_locations == ["C", "D"], (
        "Waypoints dispatched but not confirmed reached must be reported as possible obstacles"
    )
    assert "alice" not in cut.stationary_robots, (
        "A failed robot is not parked at its goal"
    )
    assert cut.committed_locations["alice"].location == "D", (
        "A failed robot's cut must not be extended to satisfy bob's blocker"
    )


def test_commit_cut_has_no_possible_obstacles_without_a_failure():
    dm = DependencyManager()
    _alice_at_b_enqueued_to_d(dm)

    assert dm.compute_commit_cut().possible_obstacle_locations == []


def test_on_plan_complete_only_completes_plans_in_the_ledger():
    dm = DependencyManager()
    v0 = _alice_plan_id(0)
    dm.set_plan("alice", _make_alice_plan())

    assert not dm.on_plan_complete("bob", _bob_plan_id(0)), "bob has no ledger"
    assert not dm.on_plan_complete("alice", _alice_plan_id(1)), (
        "v1 is not in alice's ledger"
    )
    assert dm._completed_plan_ids == set()

    assert dm.on_plan_complete("alice", v0)
    assert v0 in dm._completed_plan_ids
