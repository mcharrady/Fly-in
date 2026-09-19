"""Pathfinding, scheduling, and turn-by-turn simulation of drone
delivery.
"""
import heapq
from models import Zone, ZoneType, Connection, Graph, Drone, DroneStatus
from visualisor import Visualisor


class PathNotFoundError(Exception):
    """Raised when no valid path exists from a drone to the goal zone."""
    def __init__(self, goal: Zone) -> None:
        """Initialize a PathNotFoundError.

        Args:
            goal: The zone that could not be reached.
        """
        super().__init__(f"No path found to '{goal.name}'")


class Occupancy:
    """Tracks how full each zone and connection is for one turn."""
    def __init__(self) -> None:
        """Initialize with nothing claimed yet."""
        self.zone_counts: dict[str, int] = {}
        self.connection_counts: dict[str, int] = {}

    def zone_count(self, zone: Zone) -> int:
        """:
        """
        return self.zone_counts.get(zone.name, 0)

    def is_zone_full(self, zone: Zone) -> bool:
        """
        """
        return self.zone_count(zone) >= zone.max_drones

    def claim_zone(self, zone: Zone) -> None:
        """Reserve one slot in a zone for this turn.
        Args:
        """
        self.zone_counts[zone.name] = self.zone_count(zone) + 1

    def release_zone(self, zone: Zone) -> None:
        """Free one slot in a zone, e.g. when a drone is leaving it.
        Args:
        """
        current = self.zone_count(zone)
        self.zone_counts[zone.name] = max(0, current - 1)

    def connection_count(self, connection: Connection) -> int:
        """."""
        return self.connection_counts.get(connection.name, 0)

    def is_connection_full(self, connection: Connection) -> bool:
        """."""
        return (
            self.connection_count(connection)
            >= connection.max_link_capacity
        )

    def claim_connection(self, connection: Connection) -> None:
        """Reserve one slot on a connection for this turn.
        Args:
        """
        self.connection_counts[connection.name] = (
            self.connection_count(connection) + 1
        )


class Pathfinder:
    """Finds the cheapest currently-available path for a drone."""
    count = 1

    def __init__(self, graph: Graph) -> None:
        """Initialize the pathfinder for a given graph.

        Args:
            graph: The graph to route drones through.
        """
        self.graph = graph

    def priority_bonus(self, zone: Zone) -> int:
        """."""
        return 0 if zone.zone_type == ZoneType.priority else 1

    def move_cost(
            self, zone: Zone, connection: Connection,
            occupancy: Occupancy, is_immediate: bool
            ) -> float:
        """Return the cost of moving into a zone through a connection.

        Args:
            zone: The zone being entered.
            connection: The connection used to reach it.
            occupancy: Current zone and connection usage.
            is_immediate: Whether this is the drone's very next move.

        Returns:
            The movement cost, or float('inf') if the move can't
            happen right now.
        """
        if zone.zone_type == ZoneType.blocked:
            return float("inf")
        if is_immediate:
            if occupancy.is_zone_full(zone):
                return float("inf")
            if occupancy.is_connection_full(connection):
                return float("inf")
        cost = float(zone.movement_cost())
        if not is_immediate:
            if occupancy.is_zone_full(zone):
                cost += 2.0
            if occupancy.is_connection_full(connection):
                cost += 2.0
        return cost

    def find_path(
            self, drone: Drone, occupancy: Occupancy
            ) -> list[Zone]:
        """Find the cheapest path from the drone's zone to the goal.

        Args:
            drone: The drone to route.
            occupancy: Current zone and connection usage.

        Returns:
            The list of zones forming the path, including the start
            and goal.

        Raises:
            PathNotFoundError: If no path to the goal exists right now.
        """
        assert self.graph.end_zone is not None
        start = drone.current_zone
        goal = self.graph.end_zone
        if start == goal:
            return [start]
        heap: list[tuple[float, int, int, str, list[Zone]]] = [
            (
                0, self.priority_bonus(start),
                self.count, start.name, [start]
            )
        ]
        visited: set[str] = set()
        while heap:
            cost, _, _, zone_name, path = heapq.heappop(heap)
            if zone_name in visited:
                continue
            visited.add(zone_name)
            current_zone = self.graph.zones[zone_name]
            if current_zone == goal:
                return path
            is_immediate = (current_zone == start)
            for neighbor, Connection in self.graph.get_neighbors(current_zone):
                if neighbor.name in visited:
                    continue
                move_cost = self.move_cost(
                    neighbor, Connection, occupancy, is_immediate
                )
                if move_cost == float("inf"):
                    continue
                self.count += 1
                heapq.heappush(
                    heap,
                    (
                        cost + move_cost,
                        self.priority_bonus(neighbor),
                        self.count,
                        neighbor.name,
                        path + [neighbor]
                    )
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
    """Decides every drone's move for one turn, one drone at a time."""
    def __init__(self, pathfinder: Pathfinder) -> None:
        """Initialize the scheduler.

        Args:
            pathfinder: Used to find each drone's path when it has none.
        """
        self.pathfinder = pathfinder

    def resolve(self, drones: list[Drone], occupancy: Occupancy) -> list[Move]:
        """Decide each drone's move for this turn.

        Each drone plans its full route to the goal once, the first
        time it needs one, and then just follows it hop by hop. If
        the next hop is blocked this turn (zone or connection full),
        the drone waits and tries the same hop again next turn — it
        never abandons a working plan just because of a temporary
        block.

        Args:
            drones: Drones to move this turn (not delivered, not
                already mid-transit).
            occupancy: Zone/connection usage at the start of the turn;
                claims are added to it as drones are assigned.

        Returns:
            One move per drone.
        """
        moves: list[Move] = []
        for drone in sorted(drones, key=lambda d: d.id):
            if drone.next_zone() is None:
                try:
                    path = self.pathfinder.find_path(drone, occupancy)
                except PathNotFoundError:
                    moves.append(Move(
                        drone=drone, next_zone=drone.current_zone,
                        is_waiting=True
                    ))
                    continue
                if len(path) < 2:
                    moves.append(Move(
                        drone=drone, next_zone=drone.current_zone,
                        is_waiting=True
                    ))
                    continue
                drone.assign_path(path)
            next_zone = drone.next_zone()
            assert next_zone is not None
            connection = self.pathfinder.graph.get_connection(
                drone.current_zone, next_zone
            )
            assert connection is not None
            blocked = self.pathfinder.move_cost(
                next_zone, connection, occupancy, is_immediate=True
            ) == float("inf")
            if blocked:
                moves.append(Move(
                    drone=drone, next_zone=drone.current_zone,
                    is_waiting=True
                ))
                continue
            occupancy.release_zone(drone.current_zone)
            occupancy.claim_zone(next_zone)
            if connection is not None:
                occupancy.claim_connection(connection)
            is_transit = next_zone.is_restricted()
            connection_name = (
                connection.name if is_transit and connection is not None
                else ""
            )
            moves.append(Move(
                drone=drone, next_zone=next_zone,
                is_transit=is_transit, connection_name=connection_name
            ))
        return moves


class Simulator:
    """Runs the drone delivery simulation until all drones are delivered."""

    def __init__(self, graph: Graph, drones: list[Drone]) -> None:
        """Initialize the simulator and confirm every drone can reach the goal.

        Args:
            graph: The map graph to simulate on.
            drones: The drones to deliver.

        Raises:
            PathNotFoundError: If any drone cannot reach the goal at
                all, ignoring congestion.
        """
        self.graph = graph
        self.drones = drones
        self.pathfinder = Pathfinder(self.graph)
        self.scheduler = Scheduler(self.pathfinder)
        self.visualiser = Visualisor()
        self.turn = 0
        for drone in self.drones:
            self.pathfinder.find_path(drone, Occupancy())

    def active_drones(self) -> list[Drone]:
        """Return drones that have not yet been delivered."""
        return [d for d in self.drones if d.status != DroneStatus.delivered]

    def all_delivered(self) -> bool:
        """Return True if every drone has reached the end zone."""
        return all(d.status == DroneStatus.delivered for d in self.drones)

    def build_occupancy(self) -> Occupancy:
        """Build a fresh Occupancy snapshot from current drone positions.
        
        Returns:
        """
        occupancy = Occupancy()
        for drone in self.drones:
            if drone.status in (DroneStatus.delivered, DroneStatus.in_transit):
                if drone.status == DroneStatus.in_transit and drone.in_transit_to:
                    conn = self.graph.get_connection(drone.current_zone, drone.in_transit_to)
                    if conn:
                        occupancy.claim_connection(conn)
                continue
            zone_name = drone.current_zone.name
            occupancy.zone_counts[zone_name] = (
                occupancy.zone_counts.get(zone_name, 0) + 1
            )
        return occupancy

    def print_turn(self, moved: list[Move], turn: int) -> None:
        """Print the moves made during a turn.

        Args:
            moved: The moves that took place this turn.
            turn: The current turn number.
        """
        # print(f"Turn{turn}:", end=" ")
        print(self.visualiser.render_turn(moved, self.graph))

    def step(self, turn: int) -> None:
        """Advance the simulation by one turn.

        Args:
            turn: The current turn number.
        """
        assert self.graph.end_zone is not None
        active = self.active_drones()
        moved: list[Move] = []
        remaining: list[Drone] = []
        occupancy = self.build_occupancy()
        for drone in active:
            if (
                drone.status == DroneStatus.in_transit
                and drone.in_transit_to is not None
            ):
                drone.advance()
                moved.append(Move(drone=drone, next_zone=drone.current_zone))
            else:
                remaining.append(drone)

        for move in self.scheduler.resolve(remaining, occupancy):
            drone = move.drone
            if move.is_real_move():
                if move.is_transit:
                    drone.status = DroneStatus.in_transit
                    drone.in_transit_to = move.next_zone
                else:
                    drone.advance()
                moved.append(move)
            else:
                drone.wait()
        for drone in active:
            if drone.has_arrived(self.graph.end_zone):
                drone.status = DroneStatus.delivered
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
