#!/usr/bin/env python3
"""Start the same browser simulation on Windows, macOS, and Linux."""
import argparse
from pathlib import Path
import sys

parser = argparse.ArgumentParser(description="Run the Astoria traffic simulation in your browser")
parser.add_argument("--port", type=int, default=8011)
parser.add_argument("--no-browser", action="store_true")
args = parser.parse_args()
sys.path.insert(0, str(Path(__file__).resolve().parent / "simulation"))
try:
    from browser_simulation import main
    main(port=args.port, open_browser=not args.no_browser)
except RuntimeError as error:
    parser.exit(1, f"{error}\n")
except OSError as error:
    parser.exit(1, f"Could not start the server: {error}\nIf another copy is already running, open http://127.0.0.1:{args.port} or use --port 8012.\n")
