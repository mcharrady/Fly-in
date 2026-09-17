"""Renders simulation turns as colored terminal output."""
from typing import TYPE_CHECKING
from models import Zone, Graph
if TYPE_CHECKING:
    from simulator import Move


class Visualisor:
    """Renders each turn's drone moves as colored text for the terminal."""
    ANSI_CODES: dict[str, str] = {
        "black": "\033[30m",
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[93m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[97m",
        "gray": "\033[90m",
        "grey": "\033[90m",
        "orange": "\033[38;5;202m",
        "purple": "\033[38;5;90m",
        "pink": "\033[38;5;200m",
        "brown": "\033[38;5;94m",
        "gold": "\033[38;5;11m",
        "violet": "\033[38;5;161m",
        "maroon": "\033[38;5;130m",
        "crimson": "\033[38;5;124m",
        "darkred": "\033[38;5;88m",
        "rainbow": "\033[38;5;106m"
    }

    RESET: str = "\033[0m"

    def color_for(self, zone: Zone) -> str:
        """Return the ANSI color code for a zone, or '' if none is set."""
        if zone.color is None:
            return ""
        return self.ANSI_CODES.get(zone.color.lower(), "")

    def colorize(self, text: str, zone: Zone) -> str:
        """Wrap text in a zone's ANSI color code, if it has one.
        Args:
            text: The text to colorize.
            zone: The zone whose color to use.
        Returns:
            The colorized text, or the original text if the zone has
            no color."""
        color = self.color_for(zone)
        if not color:
            return text
        return f"{color}{text}{self.RESET}"

    def format_move(self, move: "Move", graph: Graph) -> str:
        """Format a single drone move as a colored label."""
        destination = move.next_zone
        if move.is_transit:
            label = move.connection_name
            zones = label.split('-', 1)
            zone_a = graph.zones[zones[0]]
            zone_b = graph.zones[zones[1]]
            colored_label = (f"{self.colorize(zones[0], zone_a)}"
                             f"-{self.colorize(zones[1], zone_b)}")
        else:
            label = destination.name
            colored_label = self.colorize(label, destination)
        return f"D{move.drone.id}-{colored_label}"

    def render_turn(self, moves: list["Move"], graph: Graph) -> str:
        """Format all moves for a turn into one space-separated line."""
        sorted_moves = sorted(moves, key=lambda m: m.drone.id)
        turn = [self.format_move(m, graph) for m in sorted_moves]
        return " ".join(turn)
