"""Core data structures: zones, connections, the graph, and drones."""
from enum import Enum
from typing import Optional


class ZoneType(Enum):
    """Type of a zone, determining its movement cost and passability."""

    normal = "normal"
    blocked = "blocked"
    restricted = "restricted"
    priority = "priority"

    def movement_cost(self) -> int:
        """Return the cost of moving into a zone of this type."""
        costs: dict["ZoneType", int] = {
            ZoneType.normal: 1,
            ZoneType.priority: 1,
            ZoneType.restricted: 2
        }
        return costs[self]

    @staticmethod
    def from_string(value: str) -> "ZoneType":
        """Convert a string to a ZoneType.
        Args:
            value: The zone type name to look up.
        Returns:
            The matching ZoneType.
        Raises:
            ValueError: If value does not match any known zone type."""
        for member in ZoneType:
            if member.value == value:
                return member
        valid = ", ".join(m.value for m in ZoneType)
        raise ValueError(
            f"Invalid zone type '{value}'.\nMust be one of: {valid}"
        )


class Zone:
    """A hub on the map, with a position, type and drone capacity"""

    def __init__(
        self, name: str, x: int, y: int,
        zone_type: ZoneType = ZoneType.normal,
        max_drones: int = 1, color: Optional[str] = None,
        is_start: bool = False,
        is_end: bool = False
    ) -> None:
        """Initialize a Zone.
        Args:
            name: Unique name of the zone.
            x: X coordinate.
            y: Y coordinate.
            zone_type: Type of the zone.
            max_drones: Maximum number of drones the zone can hold.
            color: Optional display color for the visualizer.
            is_start: Whether this zone is the start hub.
            is_end: Whether this zone is the end hub."""
        self.name = name
        self.x = x
        self.y = y
        self.zone_type = zone_type
        self.max_drones = max_drones
        self.color = color
        self.is_start = is_start
        self.is_end = is_end

    def movement_cost(self) -> int:
        """Return the cost of this zone."""
        return self.zone_type.movement_cost()

    def is_restricted(self) -> bool:
        """Return True if this is a restricted zone."""
        return self.zone_type == ZoneType.restricted

    def __repr__(self) -> str:
        """Return a readable representation of the zone."""
        return (
            f"Zone({self.name}, {self.zone_type.value}),"
            f"max={self.max_drones}"
        )

    def __eq__(self, other: object) -> bool:
        """Return True if other is a Zone with the same name."""
        if not isinstance(other, Zone):
            return False
        return self.name == other.name

    def __hash__(self) -> int:
        """Return a hash based on the zone's name."""
        return hash(self.name)


class Connection:
    """A bidirectional link between two zones with a capacity limit."""

    def __init__(
        self, zone_a: Zone, zone_b: Zone,
        max_link_capacity: int = 1
    ) -> None:
        """Initialize a Connection between two zones.
        Args:
            zone_a: One end of the connection.
            zone_b: The other end of the connection.
            max_link_capacity: Maximum drones allowed to use this
                connection at once.
        """
        self.zone_a = zone_a
        self.zone_b = zone_b
        self.name = f"{self.zone_a.name}-{self.zone_b.name}"
        self.max_link_capacity = max_link_capacity

    def connects(self, zone: Zone) -> bool:
        """Return True if this connection touches the given zone."""
        return zone == self.zone_a or zone == self.zone_b

    def other_end(self, zone: Zone) -> Zone:
        """Return the zone on the other side of this connection.
        Args:
            zone: One end of the connection.
        Returns:
            The zone at the opposite end."""
        if zone == self.zone_a:
            return self.zone_b
        return self.zone_a

    def __repr__(self) -> str:
        """Return a readable representation of the connection."""
        return f"Connection({self.name}, cap = {self.max_link_capacity})"


class Graph:
    """Stores all zones and connections that make up the map."""

    def __init__(self) -> None:
        """Initialize an empty graph."""
        self.zones: dict[str, Zone] = {}
        self.connections: list[Connection] = []
        self.start_zone: Optional[Zone] = None
        self.end_zone: Optional[Zone] = None

    def add_zone(self, zone: Zone) -> None:
        """Add a zone to the graph."""
        self.zones[zone.name] = zone

    def add_connection(self, connection: Connection) -> None:
        """Add a connection to the graph."""
        self.connections.append(connection)

    def set_start(self, zone: Zone) -> None:
        """Mark a zone as the start hub."""
        self.start_zone = zone

    def set_end(self, zone: Zone) -> None:
        """Mark a zone as the end hub."""
        self.end_zone = zone

    def has_zone(self, name: str) -> bool:
        """Return True if a zone with this name exists."""
        return name in self.zones

    def get_zone(self, name: str) -> Optional[Zone]:
        """Return the zone with this name,
        or None if it doesn't exist."""
        return self.zones.get(name)

    def get_neighbors(
            self, zone: Zone
    ) -> list[tuple[Zone, Connection]]:
        """Return each neighboring zone of
        the given zone with its connection.
        """
        neighbors = []
        for conn in self.connections:
            if conn.connects(zone):
                neighbor = conn.other_end(zone)
                neighbors.append((neighbor, conn))
        return neighbors

    def get_connection(
            self, zone_a: Zone, zone_b: Zone
    ) -> Optional[Connection]:
        """Return the connection between two zones, or None if none exists."""
        for conn in self.connections:
            if conn.connects(zone_a) and conn.connects(zone_b):
                return conn
        return None

    def __repr__(self) -> str:
        """Return a readable summary of the graph's size."""
        return (
            f"Graph({len(self.zones)}) zones, "
            f"{len(self.connections)} connections"
        )


class DroneStatus(Enum):
    """Possible states of a drone during the simulation."""
    waiting = "waiting"
    in_transit = "in_transit"
    delivered = "delivered"


class Drone:
    """A single drone moving from the start hub to the end hub."""
    def __init__(self, drone_id: int, start_zone: Zone) -> None:
        """Initialize a Drone at the start zone.
        Args:
            drone_id: Unique identifier for the drone.
            start_zone: The zone the drone begins at.
        """
        self.id = drone_id
        self.current_zone: Zone = start_zone
        self.path: list[Zone] = []
        self.path_index: int = 0
        self.status: DroneStatus = DroneStatus.waiting
        self.turns_waiting: int = 0
        self.in_transit_to: Optional[Zone] = None

    def assign_path(self, path: list[Zone]) -> None:
        """Assign a new path for the drone to follow, starting from index 0."""
        self.path = path
        self.path_index = 0

    def next_zone(self) -> Optional[Zone]:
        """Return the next zone in the drone's path, or None if at the end."""
        if self.path and self.path_index + 1 < len(self.path):
            return self.path[self.path_index + 1]
        return None

    def advance(self) -> None:
        """Move the drone to the next zone in its path."""
        self.path_index += 1
        self.current_zone = self.path[self.path_index]
        self.turns_waiting = 0
        self.in_transit_to = None
        self.status = DroneStatus.waiting

    def wait(self) -> None:
        """Keep the drone in its current zone for one turn."""
        self.turns_waiting += 1
        self.status = DroneStatus.waiting

    def has_arrived(self, goal: Zone) -> bool:
        """Return True if the drone is currently at the goal zone."""
        return self.current_zone == goal

    def __repr__(self) -> str:
        """Return a readable representation of the drone's state."""
        return f"D{self.id}-{self.current_zone}({self.status.value})"
