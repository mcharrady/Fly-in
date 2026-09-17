"""Entry point: parses a map file and runs the drone delivery simulation."""
import sys
from parser import Parser, ParseError
from simulator import Simulator, PathNotFoundError


def main() -> int:
    """Parse a map file, run the simulation, and print the result.
    Returns:
        Process exit code: 0 on success, 1 on any error.
    """
    if len(sys.argv) != 2:
        print(
            "Usage: python3 main.py <map_file>",
            file=sys.stderr
        )
        return 1
    map_file = sys.argv[1]
    try:
        parser = Parser()
        graph, drones = parser.parse_file(map_file)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nSimulation interrupted by the user.", file=sys.stderr)
        return 1
    except ParseError as e:
        print(f"Parser error: {e}", file=sys.stderr)
        return 1
    except PermissionError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    try:
        simulator = Simulator(graph, drones)
        total_turns = simulator.run()
    except PathNotFoundError as e:
        print(f"Simulation error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nSimulation interrupted by the user.", file=sys.stderr)
        return 1
    except Exception as e:
        print(e)
    print()
    print(f"Simulation complete in {total_turns} turns.")
    assert graph.end_zone is not None
    print(f"{len(drones)} drone(s) delivered to '{graph.end_zone.name}'.")
    return 0


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
