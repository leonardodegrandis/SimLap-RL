"""The engineering viewer must preserve every CSV channel and sample."""

import json
import tempfile
import unittest
from pathlib import Path

from simlap.dashboard import read_run


class DashboardTests(unittest.TestCase):
    def test_full_resolution_and_nullable_extra_channels(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            (directory / "dashboard.json").write_text(json.dumps({
                "rows": [{"time_s": 0}, {"time_s": 1}], "columns": ["time_s"], "summary": {"samples": 3}
            }))
            (directory / "telemetry.csv").write_text("time_s,battery_current_a,custom_temperature_k,command_yaw_rate_radps\n0,10,290,\n0.5,-20,300,\n1,0,295,\n")
            (directory / "channels.json").write_text('{"custom_temperature_k": "Custom thermal state"}')
            data = read_run(directory)
            self.assertEqual(len(data["rows"]), 3)
            self.assertEqual(data["rows"][1]["battery_current_a"], -20)
            self.assertEqual(data["rows"][1]["custom_temperature_k"], 300)
            self.assertIsNone(data["rows"][1]["command_yaw_rate_radps"])
            self.assertIn("command_yaw_rate_radps", data["columns"])
            self.assertEqual(data["channel_descriptions"]["custom_temperature_k"], "Custom thermal state")

    def test_older_run_without_channel_descriptions(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            (directory / "dashboard.json").write_text('{"rows": []}')
            (directory / "telemetry.csv").write_text("time_s,speed_mps\n0,0\n")
            self.assertEqual(read_run(directory)["channel_descriptions"], {})
