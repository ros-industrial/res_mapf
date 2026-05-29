from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from res_mapf_planning.cbs.cbs import Location


class Constraints(object):
    def __init__(self) -> None:
        self.vertex_constraints: set[VertexConstraint] = set()
        self.edge_constraints: set[EdgeConstraint] = set()

    def add_constraint(self, other: "Constraints") -> None:
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self) -> str:
        return (
            "VC: "
            + str([str(vc) for vc in self.vertex_constraints])
            + "EC: "
            + str([str(ec) for ec in self.edge_constraints])
        )


class VertexConstraint(object):
    def __init__(self, time: int, location: "Location") -> None:
        self.time = time
        self.location = location

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, VertexConstraint):
            return NotImplemented
        return self.time == other.time and self.location == other.location

    def __hash__(self) -> int:
        return hash(str(self.time) + str(self.location))

    def __str__(self) -> str:
        return "(" + str(self.time) + ", " + str(self.location) + ")"


class EdgeConstraint(object):
    def __init__(
        self, time: int, location_1: "Location", location_2: "Location"
    ) -> None:
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, EdgeConstraint):
            return NotImplemented
        return (
            self.time == other.time
            and self.location_1 == other.location_1
            and self.location_2 == other.location_2
        )

    def __hash__(self) -> int:
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))

    def __str__(self) -> str:
        return (
            "("
            + str(self.time)
            + ", "
            + str(self.location_1)
            + ", "
            + str(self.location_2)
            + ")"
        )
