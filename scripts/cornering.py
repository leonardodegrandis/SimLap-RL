"""Solve and simulate the configured steady-state corner."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simlap.run import main

if __name__ == "__main__":
    main("cornering")
