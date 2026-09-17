"""Pathfinding, scheduling, and turn-by-turn simulation of drone delivery."""
import heapq
from typing import Optional
from models import Zone, ZoneType, Graph, Drone, DroneStatus
from visualisor import Visualisor


class PathNotFoundError(Exception):
    """Raised when no valid path exists from a drone to the goal zone."""
    def __init__(self, goal: Zone):
        """Initialize a PathNotFoundError.
        Args:
            goal: The zone that could not be reached.
        """
        self.goal = goal
        super().__init__(
           f"No path found to '{goal.name}'"
        )


class ZoneOccupancy:
    """Tracks how many drones currently occupy each zone.."""
    def __init__(self) -> None:
        """Initialize empty occupancy counts."""
        self.counts: dict[str, int] = {}

    def set(self, zone_name: str, count: int) -> None:
        """Set the occupancy count for a zone."""
        self.counts[zone_name] = count

    def get(self, zone_name: str) -> int:
        """Return the occupancy count for a zone, defaulting to 0."""
        return self.counts.get(zone_name, 0)

    def is_full(self, zone: Zone) -> bool:
        """Return True if a zone has reached its maximum drone capacity."""
        return self.get(zone.name) >= zone.max_drones


class Pathfinder:
    """Finds shortest paths for drones through the graph using Dijkstra."""
    count = 1

    def __init__(self, graph: Graph) -> None:
        """Initialize the pathfinder for a given graph.
        Args:
            graph: The graph to route drones through."""
        self.graph = graph
        self.planned_usage: dict[str, int] = {}

    def register_path(self, path: list[Zone]) -> None:
        """Record the zones of a path to
            penalize congestion in future routes.
        """
        for zone in path:
            self.planned_usage[zone.name] = (
                self.planned_usage.get(zone.name, 0) + 1
            )

    def usage_penalty(self, zone: Zone) -> float:
        """Return the congestion penalty for a zone based on planned usage."""
        penalty_per_drone = 2.0
        return self.planned_usage.get(zone.name, 0) * penalty_per_drone

    def zone_cost(
        self, zone: Zone, occupancy: ZoneOccupancy,
        avoid: Optional[set[str]] = None,
        is_immediate: bool = False
    ) -> float:
        """Return the cost of moving into a zone.
        Args:
            zone: The zone being entered.
            occupancy: Current zone occupancy counts.
            avoid: Zone names to treat as impassable, if this is an
                immediate move.
            is_immediate: Whether this is the drone's very next move.
        Returns:
            The movement cost, or float('inf') if the zone can't be
            entered.
        """
        if zone.zone_type == ZoneType.blocked:
            return float("inf")
        if is_immediate:
            if avoid and zone.name in avoid:
                if not zone.zone_type == ZoneType.priority:
                    return float("inf")
            if occupancy.is_full(zone):
                return float("inf")
        cost = float(zone.movement_cost())
        if not is_immediate and occupancy.is_full(zone):
            cost += 2.0
        cost += self.usage_penalty(zone)
        return cost

    def priority_bonus(self, zone: Zone) -> int:
        """Return a tie-breaking bonus, lower for priority zones."""
        return 0 if zone.zone_type == ZoneType.priority else 1

    def find_path(
        self, drone: Drone,
        occupancy: ZoneOccupancy,
        avoid: Optional[set[str]] = None
    ) -> list[Zone]:
        """Find the shortest path from the drone's zone to the goal.
        Args:
            drone: The drone to route.
            occupancy: Current zone occupancy counts.
            avoid: Zone names the drone's first move must not enter.
        Returns:
            The list of zones forming the path, including the start
            and goal.
        Raises:
            PathNotFoundError: If no path to the goal exists.
        """
        assert self.graph.end_zone is not None
        start = drone.current_zone
        goal = self.graph.end_zone
        if start == goal:
            return [start]
        initial_bonus = self.priority_bonus(start)
        heap: list[tuple[float, int, int, str, list[Zone]]] = [
            (0, initial_bonus, self.count, start.name, [start])
        ]
        visited: set[str] = set()
        while heap:
            cost, bonus, _, zone_name, path = heapq.heappop(heap)
            if zone_name in visited:
                continue
            visited.add(zone_name)
            current_zone = self.graph.zones[zone_name]
            if current_zone == goal:
                return path
            for neighbor, _ in self.graph.get_neighbors(current_zone):
                if neighbor.name in visited:
                    continue
                is_immediate = (current_zone == start)
                move_cost = self.zone_cost(
                    neighbor, occupancy, avoid, is_immediate)
                if move_cost == float("inf"):
                    continue
                self.count += 1
                new_cost = cost + move_cost
                bonus = self.priority_bonus(neighbor)
                new_path = path + [neighbor]
                heapq.heappush(
                    heap,
                    (new_cost, bonus, self.count, neighbor.name, new_path)
                )
        raise PathNotFoundError(goal)


class Move:
    """A single drone's action for one simulation turn."""
    def __init__(
        self, drone: Drone, next_zone: Zone,
        is_waiting: bool = False, is_transit: bool = False,
        connection_name: str = ""
    ) -> None:
        """Initialize a Move.
        Args:
            drone: The drone performing this move.
            next_zone: The zone the drone is moving to (or staying in).
            is_waiting: Whether the drone is waiting instead of moving.
            is_transit: Whether the drone is entering transit on a
                restricted zone.
            connection_name: Name of the connection used, if any.
        """
        self.drone = drone
        self.next_zone = next_zone
        self.is_waiting = is_waiting
        self.is_transit = is_transit
        self.connection_name = connection_name

    def is_real_move(self) -> bool:
        """Return True if this move actually changes the drone's zone."""
        return not self.is_waiting

    def __repr__(self) -> str:
        """Return a readable representation of the move."""
        if self.is_waiting:
            return f"D{self.drone.id} WAIT@{self.drone.current_zone.name}"
        if self.is_transit:
            return f"D{self.drone.id} TRANSIT>{self.next_zone.name}"
        return f"D{self.drone.id} MOVE>{self.next_zone.name}"


class Scheduler:
    """Resolves conflicting drone move requests turn by turn."""

    def __init__(self, pathfinder: Pathfinder) -> None:
        """Initialize the scheduler.
        Args:
            pathfinder: Used to reroute drones whose move is rejected.
        """
        self.pathfinder = pathfinder

    def resolve(
        self, drones: list[Drone],
        occupancy: ZoneOccupancy
    ) -> list[Move]:
        """Compute the approved moves for all active drones this turn.
        Args:
            drones: Drones that are not yet delivered.
            occupancy: Current zone occupancy counts.
        Returns:
            The list of moves to apply this turn, one per drone.
        """
        intentions: dict[int, Optional[Zone]] = {}
        for drone in drones:
            if drone.status == DroneStatus.in_transit:
                continue
            intentions[drone.id] = drone.next_zone()

        approved: dict[int, Zone] = {}
        connection_usage: dict[str, int] = {}
        avoided: dict[int, set[str]] = {
            d.id: set() for d in drones
            if d.status != DroneStatus.in_transit
        }
        pending = [
            d for d in drones
            if intentions.get(d.id) is not None
        ]
        max_rounds = len(drones) * 2 + 2
        for _ in range(max_rounds):
            if not pending:
                break
            failed = self.approve_round(
                pending, drones, intentions, approved,
                connection_usage, occupancy
            )
            if len(failed) < len(pending):
                pending = failed
                continue
            progressed = False
            adjusted = self.adjusted_occupancy(
                occupancy, drones, approved, intentions
            )
            for drone in failed:
                target = intentions[drone.id]
                if target is not None:
                    avoided[drone.id].add(target.name)
                alt = self.find_alternate_target(
                    drone, adjusted, avoided[drone.id]
                )
                if alt is not None:
                    intentions[drone.id] = alt
                    progressed = True
            if not progressed:
                break
            pending = failed
        moves = self.build_moves(drones, approved)
        return moves

    def adjusted_occupancy(
        self, occupancy: ZoneOccupancy, all_drones: list[Drone],
        approved: dict[int, Zone],
        intentions: dict[int, Optional[Zone]]
    ) -> ZoneOccupancy:
        """Return zone occupancy adjusted for drones about to leave.
        Args:
            occupancy: Current zone occupancy counts.
            all_drones: All active drones.
            approved: Moves already approved this round.
            intentions: Each drone's intended next zone.
        Returns:
            A new ZoneOccupancy with departing drones subtracted out.
        """
        adjusted = ZoneOccupancy()
        for zone_name, count in occupancy.counts.items():
            leaving = sum(
                1 for d in all_drones
                if d.current_zone.name == zone_name
                and d.id in approved
                and intentions[d.id] != d.current_zone
            )
            adjusted.set(zone_name, max(0, count - leaving))
        return adjusted

    def approve_round(
        self, pending: list[Drone], all_drones: list[Drone],
        intentions: dict[int, Optional[Zone]],
        approved: dict[int, Zone],
        connection_usage: dict[str, int],
        occupancy: ZoneOccupancy
    ) -> list[Drone]:
        """Approve as many pending drone moves as capacity allows.
        Args:
            pending: Drones still waiting for approval this round.
            all_drones: All active drones.
            intentions: Each drone's intended next zone.
            approved: Moves approved so far; updated in place.
            connection_usage: Connection usage counts; updated in place.
            occupancy: Current zone occupancy counts.
        Returns:
            The drones whose move could not be approved this round.
        """
        zone_requests: dict[str, list[Drone]] = {}
        for drone in sorted(pending, key=lambda d: d.id):
            target = intentions[drone.id]
            if target is not None:
                zone_requests.setdefault(target.name, []).append(drone)
        newly_approved: set[int] = set()
        for zone_name, competing in zone_requests.items():
            zone = self.pathfinder.graph.zones[zone_name]
            leaving = sum(
                1 for d in all_drones
                if d.current_zone.name == zone_name
                and d.id in approved
                and intentions[d.id] != d.current_zone
            )
            already_in = sum(
                1 for z in approved.values() if z.name == zone_name
            )
            current_count = occupancy.get(zone_name)
            available_slots = (
                zone.max_drones - current_count + leaving - already_in
            )
            for drone in competing:
                if available_slots > 0:
                    connection = self.pathfinder.graph.get_connection(
                        drone.current_zone, zone
                    )
                    if connection is None:
                        continue
                    conn_name = connection.name
                    used = connection_usage.get(conn_name, 0)
                    if used < connection.max_link_capacity:
                        approved[drone.id] = zone
                        newly_approved.add(drone.id)
                        available_slots -= 1
                        connection_usage[conn_name] = used + 1
        failed: list[Drone] = []
        for drone in pending:
            if drone.id not in newly_approved:
                failed.append(drone)
        return failed

    def find_alternate_target(
        self, drone: Drone, occupancy: ZoneOccupancy,
        avoid_zone_name: set[str]
    ) -> Optional[Zone]:
        """Find an alternate next zone for a drone whose move was rejected.
        Args:
            drone: The drone to reroute.
            occupancy: Current zone occupancy counts.
            avoid_zone_name: Zone names the drone's next move must avoid.
        Returns:
            The drone's new next zone, or None if no alternate route
                exists.
        """
        try:
            new_path = self.pathfinder.find_path(
                drone, occupancy, avoid=avoid_zone_name
            )
        except PathNotFoundError:
            return None
        if len(new_path) < 2:
            return None
        drone.assign_path(new_path)
        return drone.next_zone()

    def build_moves(
        self, drones: list[Drone], approved: dict[int, Zone]
    ) -> list[Move]:
        """Build the final list of Move objects from approved targets.
        Args:
            drones: Active drones.
            approved: Each approved drone's next zone.
        Returns:
            One Move per drone: a real move, a transit, or a wait.
        """
        moves: list[Move] = []
        for drone in drones:
            if drone.status == DroneStatus.in_transit:
                continue
            if drone.id in approved:
                target = approved[drone.id]
                is_transit = target.is_restricted()
                connection_name = ""
                if is_transit:
                    connection = self.pathfinder.graph.get_connection(
                        drone.current_zone, target
                    )
                    if connection is not None:
                        connection_name = connection.name
                moves.append(Move(
                    drone=drone,
                    next_zone=target,
                    is_transit=is_transit,
                    connection_name=connection_name
                ))
            else:
                moves.append(Move(
                    drone=drone,
                    next_zone=drone.current_zone,
                    is_waiting=True
                ))
        return moves


class Simulator:
    """Runs the drone delivery simulation until all drones are delivered."""
    def __init__(self, graph: Graph, drones: list[Drone]) -> None:
        """Initialize the simulator and compute each drone's first path.
        Args:
            graph: The map graph to simulate on.
            drones: The drones to deliver.
        """
        self.graph = graph
        self.drones = drones
        self.pathfinder = Pathfinder(self.graph)
        self.scheduler = Scheduler(self.pathfinder)
        self.visualiser = Visualisor()
        self.occupancy = ZoneOccupancy()
        self.turn = 0
        for drone in self.drones:
            path = self.pathfinder.find_path(drone, self. occupancy)
            drone.assign_path(path)
            if len(drones) <= 20:
                self.pathfinder.register_path(path)

    def active_drones(self) -> list[Drone]:
        """Return drones that have not yet been delivered."""
        return [
            d for d in self.drones
            if d.status != DroneStatus.delivered
        ]

    def all_delivered(self) -> bool:
        """Return True if every drone has reached the end zone."""
        return all(
            d.status == DroneStatus.delivered
            for d in self.drones
        )

    def update_occupancy(self) -> None:
        """Recompute zone occupancy counts from current drone positions."""
        new_counts: dict[str, int] = {}
        for drone in self.drones:
            if drone.status == DroneStatus.delivered:
                continue
            if drone.status == DroneStatus.in_transit:
                continue
            zone_name = drone.current_zone.name
            new_counts[zone_name] = new_counts.get(zone_name, 0) + 1
        for zone_name, count in new_counts.items():
            self.occupancy.set(zone_name, count)
        for zone_name in self.occupancy.counts:
            if zone_name not in new_counts:
                self.occupancy.set(zone_name, 0)

    def print_turn(
        self, moved: list[Move], turn: int,
    ) -> None:
        """Print the moves made during a turn.
        Args:
            moved: The moves that took place this turn.
            turn: The current turn number.
            occupancy: Current zone occupancy counts.
        """
        print(f"{self.visualiser.render_turn(moved, self.graph)}")

    def step(self, turn: int) -> None:
        """Advance the simulation by one turn.
        Args:
            turn: The current turn number."""
        assert self.graph.end_zone is not None
        active = self.active_drones()
        moves = self.scheduler.resolve(active, self.occupancy)
        moved: list[Move] = []
        for drone in active:
            if (
                drone.status == DroneStatus.in_transit
                and drone.in_transit_to is not None
            ):
                drone.advance()
                moved.append(Move(
                    drone=drone,
                    next_zone=drone.current_zone
                ))
        arrived_ids = {m.drone.id for m in moved}
        for move in moves:
            if move.drone.id in arrived_ids:
                continue
            if move.is_real_move():
                if move.is_transit:
                    move.drone.status = DroneStatus.in_transit
                    move.drone.in_transit_to = move.next_zone
                else:
                    move.drone.advance()
                moved.append(move)
            else:
                move.drone.wait()
        for drone in active:
            if drone.has_arrived(self.graph.end_zone):
                drone.status = DroneStatus.delivered
        self.update_occupancy()
        self.print_turn(moved, turn)

    def run(self) -> int:
        """Run the simulation until every drone is delivered.
        Returns:
            The total number of turns taken.
        """
        assert self.graph.end_zone is not None
        while not self.all_delivered():
            self.turn += 1
            self.step(self.turn)
        return self.turn
