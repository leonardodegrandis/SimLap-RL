"""Run a trained policy for one lap and log a lap_time maneuver for the dashboard."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simlap.rl.evaluate import main

if __name__ == "__main__":
    main("lap_time")
