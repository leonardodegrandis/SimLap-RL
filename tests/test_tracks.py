import math
import tempfile
import unittest
from pathlib import Path

from simlap.tracks import StadiumTrack, load_track
from simlap.tracks.base import signed_progress
from simlap.tracks.export import export_track


class TrackTests(unittest.TestCase):
    def test_closed_geometry_and_analytic_projection(self):
        track = load_track()
        self.assertAlmostEqual(track.length_m, 240 + 60 * math.pi)
        for index in range(400):
            s = index * track.length_m / 400
            point = track.sample(s)
            for offset in (-2, 0, 2):
                x = point.x_m - offset * math.sin(point.heading_rad)
                y = point.y_m + offset * math.cos(point.heading_rad)
                projection = track.project(x, y)
                self.assertAlmostEqual(signed_progress(s, projection.point.s_m, track.length_m), 0, places=8)
                self.assertAlmostEqual(projection.lateral_m, offset, places=8)
        self.assertEqual(track.sample(0), track.sample(track.length_m))

    def test_signed_progress_crosses_finish_in_both_directions(self):
        self.assertAlmostEqual(signed_progress(99, 1, 100), 2)
        self.assertAlmostEqual(signed_progress(1, 99, 100), -2)
        self.assertAlmostEqual(signed_progress(20, 19, 100), -1)

    def test_track_validation_and_export(self):
        with self.assertRaises(ValueError):
            StadiumTrack(width_m=60)
        with tempfile.TemporaryDirectory() as folder:
            export_track(load_track(), folder, 3)
            self.assertIn("left_x_m", (Path(folder) / "geometry.csv").read_text())
            self.assertIn("<svg", (Path(folder) / "geometry.svg").read_text())
