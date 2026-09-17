"""Parses map files into a Graph and a list of Drones."""
from typing import Optional
from models import Zone, ZoneType, Connection
from models import Graph, Drone


class ParseError(Exception):
    """Raised when a line in the map file is invalid."""
    def __init__(self, line_number: int, message: str) -> None:
        """Initialize a ParseError.
        Args:
            line_number: The line where the error occurred.
            message: Description of what went wrong.
        """
        self.line_number = line_number
        self.message = message
        super().__init__(f"Line {line_number}: {message}")


class Parser:
    """Reads a map file and builds a Graph and list of Drones from it."""

    def __init__(self) -> None:
        """Initialize an empty parser."""
        self.graph: Optional[Graph] = None
        self.nb_drones: int = 0
        self.seen_connection: set[frozenset[str]] = set()
        self.seen_coordinates: set[tuple[int, int]] = set()

    def parse_metadata(
            self, raw: str, line_number: int
            ) -> dict[str, str]:
        """Parse a 'key=value key=value ...' string into a dict.
        Args:
            raw: The raw metadata content, without brackets.
            line_number: Line number, for error reporting.
        Returns:
            A dict of metadata key/value pairs.
        """
        metadata: dict[str, str] = {}
        tokens = raw.split()
        for token in tokens:
            if '=' not in token:
                raise ParseError(
                    line_number,
                    f"Invalid metadata token '{token}' "
                    f"Expected 'key=value' format"
                )
            key, value = token.split('=', 1)
            key = key.strip()
            value = value.strip()
            if '=' in value:
                raise ParseError(
                    line_number,
                    f"invalid value: {value}"
                )
            if not key:
                raise ParseError(
                    line_number,
                    f"Metadata key is missing: '{token}'"
                )
            if not value:
                raise ParseError(
                    line_number,
                    f"Metadata value is missing: '{token}'"
                )
            if key in metadata:
                raise ParseError(
                    line_number,
                    f"Duplicate metadata key '{key}'"
                )
            metadata[key] = value
        return metadata

    def extract_metadata_block(
            self, token: str, line_number: int
    ) -> dict[str, str]:
        """Validate and parse a '[key=value ...]' metadata token.
        Args:
            token: The full token, including the surrounding brackets.
            line_number: Line number, for error reporting.
        Returns:
            A dict of metadata key/value pairs.
        """
        if token.count('[') != 1 or token.count(']') != 1:
            raise ParseError(
                line_number,
                f"Malformed metadata block: '{token}'"
            )
        if not token.startswith('[') or not token.endswith(']'):
            raise ParseError(
                line_number,
                f"Malformed metadata block: '{token}'"
            )
        raw_metadata = token[1:-1]
        return self.parse_metadata(raw_metadata, line_number)

    def parse_zone_type(
            self, value: str, line_number: int
    ) -> ZoneType:
        """Convert a string to a ZoneType, raising ParseError if invalid."""
        try:
            return ZoneType.from_string(value)
        except ValueError as e:
            raise ParseError(line_number, str(e))

    def parse_capacity(
            self, value: str, field_name: str, line_number: int
    ) -> int:
        """Parse and validate a positive integer capacity value.
        Args:
            value: The raw string value to parse.
            field_name: Name of the field, used in error messages.
            line_number: Line number, for error reporting.
        Returns:
            The parsed positive integer.
        """
        try:
            n = int(value)
        except ValueError:
            raise ParseError(
                line_number,
                f"{field_name} must be an integer"
            )
        if n <= 0:
            raise ParseError(
                line_number,
                f"{field_name} must be a positive integer"
            )
        return n

    def parse_zone(
            self, tokens: list[str], line_number: int,
            is_start: bool = False, is_end: bool = False
    ) -> None:
        """Parse a hub/start_hub/end_hub line and add the zone to the graph.
        Args:
            tokens: The line's tokens after the keyword (name, x, y,
                and optionally a metadata block).
            line_number: Line number, for error reporting.
            is_start: Whether this zone is the start hub.
            is_end: Whether this zone is the end hub.
        """
        assert self.graph is not None
        if len(tokens) not in (3, 4):
            raise ParseError(
                line_number,
                "Expected '<name> <x> <y>' format"
            )
        name, x_str, y_str = tokens[0], tokens[1], tokens[2]
        metadata: dict[str, str] = {}
        if len(tokens) == 4:
            metadata = self.extract_metadata_block(tokens[3], line_number)
        unknown = set(metadata) - {"zone", "max_drones", "color"}
        if unknown:
            raise ParseError(
                line_number,
                f"Unknown metadata key(s): {', '.join(unknown)}"
            )
        if '-' in name:
            raise ParseError(
                line_number,
                f"Zone name '{name}' has a dash"
            )
        try:
            x = int(x_str)
            y = int(y_str)
        except ValueError:
            raise ParseError(
                line_number,
                "Zone coordinates must be integers"
            )
        if self.graph.has_zone(name):
            raise ParseError(
                line_number,
                f"Duplicate zone name '{name}'."
            )
        if (x, y) in self.seen_coordinates:
            raise ParseError(
                line_number,
                f"Duplicate coordinates: {(x, y)}"
            )
        self.seen_coordinates.add((x, y))
        zone_type = self.parse_zone_type(
            metadata.get('zone', 'normal'), line_number
        )
        if (is_start or is_end) and zone_type == ZoneType.blocked:
            raise ParseError(
                line_number,
                "start and end zones can not be blocked"
            )
        max_drones = self.parse_capacity(
            metadata.get('max_drones', '1'),
            'max_drones',
            line_number
        )
        if is_start or is_end:
            max_drones = self.nb_drones
        color = metadata.get('color', None)
        zone = Zone(
            name=name,
            x=x,
            y=y,
            zone_type=zone_type,
            max_drones=max_drones,
            color=color,
            is_start=is_start,
            is_end=is_end
        )
        self.graph.add_zone(zone)
        if is_start:
            self.graph.set_start(zone)
        if is_end:
            self.graph.set_end(zone)

    def parse_nb_drones(self, tokens: list[str], line_number: int) -> int:
        """Parse the nb_drones line and store the drone count.
        Args:
            tokens: The line's whitespace-split tokens.
            line_number: Line number, for error reporting.
        Returns:
            The parsed number of drones.
        """
        if len(tokens) != 2:
            raise ParseError(
                line_number,
                "nb_drones must be 'nb_drones: <number>'"
                )
        value = tokens[1]
        try:
            nb = int(value)
        except ValueError:
            raise ParseError(
                line_number,
                "nb_drones must be an integer"
            )
        if nb <= 0:
            raise ParseError(
                line_number,
                "nb_drones must be a positive integer"
            )
        self.nb_drones = nb
        return nb

    def parse_connection(self, tokens: list[str], line_number: int) -> None:
        """Parse a connection line and add it to the graph.
        Args:
            tokens: The line's tokens after the keyword ('<zone1>-<zone2>'
                and optionally a metadata block).
            line_number: Line number, for error reporting.
        """
        assert self.graph is not None
        if len(tokens) < 1:
            raise ParseError(
                line_number,
                "Connection must be "
                "'connection: <zone1>-<zone2> [metadata]'"
            )
        content = tokens[0]
        if '-' not in content:
            raise ParseError(
                line_number,
                "Connection must be "
                "'connection: <zone1>-<zone2> [metadata]'"
            )
        metadata: dict[str, str] = {}
        if len(tokens) == 2:
            metadata = self.extract_metadata_block(tokens[1], line_number)
        unknown = set(metadata) - {"max_link_capacity"}
        if unknown:
            raise ParseError(
                line_number,
                f"Unknown metadata key(s): {', '.join(unknown)}"
            )
        parts = content.split('-', 1)
        name_a = parts[0].strip()
        name_b = parts[1].strip()
        if not name_a or not name_b:
            raise ParseError(
                line_number,
                "Missing zone in connection expression"
            )
        if name_a == name_b:
            raise ParseError(
                line_number,
                "a zone can not be linked to itself"
            )
        zone_a = self.graph.get_zone(name_a)
        zone_b = self.graph.get_zone(name_b)
        if not zone_a:
            raise ParseError(
                line_number,
                f"Connection has an undefined zone '{name_a}'"
            )
        if not zone_b:
            raise ParseError(
                line_number,
                f"Connection has an undefined zone '{name_b}'"
            )
        pair: frozenset[str] = frozenset([name_a, name_b])
        if pair in self.seen_connection:
            raise ParseError(
                line_number,
                f"Duplicate connection between '{name_a}' and '{name_b}'"
            )
        self.seen_connection.add(pair)
        max_link_capacity = self.parse_capacity(
            metadata.get('max_link_capacity', '1'),
            'max_link_capacity',
            line_number
        )
        connection = Connection(
            zone_a=zone_a,
            zone_b=zone_b,
            max_link_capacity=max_link_capacity
        )
        self.graph.add_connection(connection)

    def parse_lines(self, lines: list[str]) -> None:
        """Parse every line of the map file, dispatching by keyword.
        Args:
            lines: The raw lines read from the map file.
        """
        nb_drones_set = False
        start_found = False
        end_found = False
        for line_number, raw_line in enumerate(lines, start=1):
            line = raw_line.split('#')[0].strip()
            if not line:
                continue
            keyword = line.split(maxsplit=1)[0]
            if keyword == "nb_drones:":
                if nb_drones_set:
                    raise ParseError(
                        line_number,
                        "Duplicate nb_drones."
                    )
                tokens = line.split()
                self.parse_nb_drones(tokens, line_number)
                nb_drones_set = True
            elif keyword == "start_hub:":
                if not nb_drones_set:
                    raise ParseError(
                        line_number,
                        "nb_drones must be defined before any zone."
                    )
                if start_found:
                    raise ParseError(
                        line_number,
                        "Duplicate start_hub"
                    )
                tokens = line.split(maxsplit=4)
                self.parse_zone(tokens[1:], line_number, is_start=True)
                start_found = True
            elif keyword == "end_hub:":
                if not nb_drones_set:
                    raise ParseError(
                        line_number,
                        "nb_drones must be defined before any zone."
                    )
                if end_found:
                    raise ParseError(
                        line_number,
                        "Duplicate end_hub"
                    )
                tokens = line.split(maxsplit=4)
                self.parse_zone(tokens[1:], line_number, is_end=True)
                end_found = True
            elif keyword == "hub:":
                if not nb_drones_set:
                    raise ParseError(
                        line_number,
                        "nb_drones must be defined before any zone."
                    )
                tokens = line.split(maxsplit=4)
                self.parse_zone(tokens[1:], line_number)
            elif keyword == "connection:":
                tokens = line.split(maxsplit=2)
                self.parse_connection(tokens[1:], line_number)
            else:
                raise ParseError(
                    line_number,
                    f"invalide line format: '{line}'"
                )

    def validate_graph(self) -> None:
        """Check that the graph has exactly one start hub and one end hub."""
        assert self.graph is not None
        if self.graph.start_zone is None:
            raise ParseError(0, "Map must have a start_hub")
        if self.graph.end_zone is None:
            raise ParseError(0, "Map must have a end_hub")

    def parse_file(self, filepath: str) -> tuple[Graph, list[Drone]]:
        """Read a map file and build its Graph and Drones.
        Args:
            filepath: Path to the map file to read.
        Returns:
            A tuple of the built Graph and the list of Drones created
            at the start hub."""
        self.graph = Graph()
        self.nb_drones = 0
        self.seen_connection = set()
        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
                checked_lines = lines
                if not any(c.strip() for c in checked_lines):
                    raise ParseError(0, "Empty Map file !")
        except PermissionError:
            raise PermissionError("you have no permission !!!")
        except FileNotFoundError:
            raise FileNotFoundError(f"Map file not found: {filepath}")
        self.parse_lines(lines)
        self.validate_graph()
        assert self.graph is not None
        assert self.graph.start_zone is not None
        drones = [
            Drone(drone_id=i + 1, start_zone=self.graph.start_zone)
            for i in range(self.nb_drones)
        ]
        return (self.graph, drones)
