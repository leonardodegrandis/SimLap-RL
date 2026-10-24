"""Export sampled centerline/boundaries and a simple plan view of a track."""

import argparse
import csv
import math
from pathlib import Path

from .stadium import load_track


def export_track(track, directory, spacing_m=1.0):
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("Track sample spacing must be finite and positive")
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    count = math.ceil(track.length_m / spacing_m)
    rows = []
    for i in range(count + 1):
        s = i * track.length_m / count
        p = track.sample(s)
        nx, ny = -math.sin(p.heading_rad) * track.width_m / 2, math.cos(p.heading_rad) * track.width_m / 2
        rows.append(dict(s_m=s, x_m=p.x_m, y_m=p.y_m, heading_rad=p.heading_rad, curvature_pm=p.curvature_pm,
                         left_x_m=p.x_m + nx, left_y_m=p.y_m + ny, right_x_m=p.x_m - nx, right_y_m=p.y_m - ny))
    with (directory / "geometry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    xs = [row[key] for row in rows for key in ("left_x_m", "right_x_m")]
    ys = [row[key] for row in rows for key in ("left_y_m", "right_y_m")]
    x0, y0, width, height = min(xs) - 8, -max(ys) - 8, max(xs) - min(xs) + 16, max(ys) - min(ys) + 16
    lines = []
    for prefix, color in (("left_", "#333"), ("right_", "#333"), ("", "#1670ad")):
        points = " ".join(f'{r[prefix + "x_m"]:.3f},{-r[prefix + "y_m"]:.3f}' for r in rows)
        lines.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="0.5"/>')
    svg = f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0} {y0} {width} {height}"><rect x="{x0}" y="{y0}" width="{width}" height="{height}" fill="white"/>{"".join(lines)}<circle cx="0" cy="0" r="1.2" fill="red"/></svg>'
    (directory / "geometry.svg").write_text(svg + "\n")
    return directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track")
    parser.add_argument("--output", default="tracks/generated/oval")
    parser.add_argument("--spacing", type=float, default=1.0)
    args = parser.parse_args()
    print(export_track(load_track(args.track), args.output, args.spacing))


if __name__ == "__main__":
    main()
