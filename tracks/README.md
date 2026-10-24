# Track geometries

`oval.json` describes a counterclockwise stadium: two **120 m straights**, two **30 m radius semicircles**, and **8 m road width**. The centerline length is approximately **428.50 m**. The start is `(0, 0)` heading along +X; corners turn left. All dimensions are meters.

These are road geometry parameters, not an optimal racing line or a target speed trajectory. The stadium is tangent-continuous; curvature steps at the straight/arc joins. A future clothoid/spline implementation can smooth those joins behind the same `Track` interface.

Generate coordinates and a plan-view SVG:

```sh
.venv/bin/python -m simlap.tracks.export --track tracks/oval.json
```

Outputs are `tracks/generated/oval/geometry.csv` and `geometry.svg`. The CSV contains arc length, center coordinates, heading, curvature and both track boundaries. Change the JSON and regenerate; JSON remains the source of truth. `--spacing` sets the approximate sampling interval and `--output` selects the destination.

Geometry algorithms live in `simlap/tracks/`. Add a new implementation of `Track.sample()` and `Track.project()` and extend `load_track()` when introducing other track types. Projection is used to check boundaries and measure signed progress. The physics engine does not depend on track geometry.
