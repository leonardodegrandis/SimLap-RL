"""Read-only local dashboard; serves completed runs and exact downloadable logs."""

import argparse
import csv
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from .config import ROOT

DOWNLOADS = {"telemetry.csv", "engine.log", "config.json", "metadata.json", "summary.json", "channels.json"}


def read_run(directory):
    """Use the recorded CSV, not the downsampled preview, for engineering plots."""
    data = json.loads((directory / "dashboard.json").read_text())
    with (directory / "telemetry.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        data["columns"] = reader.fieldnames
        data["rows"] = [{key: float(value) if value else None for key, value in row.items()} for row in reader]
    descriptions = directory / "channels.json"
    data["channel_descriptions"] = json.loads(descriptions.read_text()) if descriptions.exists() else {}
    return data


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, results, **kwargs):
        self.results = Path(results).resolve()
        super().__init__(*args, directory=str(ROOT / "dashboard"), **kwargs)

    def do_GET(self):
        url = urlparse(self.path)
        if not url.path.startswith("/api/"):
            return super().do_GET()
        try:
            query = parse_qs(url.query)
            if url.path == "/api/runs":
                runs = []
                for path in sorted(self.results.glob("*/summary.json"), reverse=True):
                    if (path.parent / "dashboard.json").exists():
                        try:
                            runs.append(json.loads(path.read_text()))
                        except (OSError, ValueError):
                            continue
                return self.send_data(json.dumps(runs).encode(), "application/json")
            run_id = query.get("id", [""])[0]
            run = (self.results / run_id).resolve()
            if not run_id or run.parent != self.results:
                return self.send_error(400, "Invalid run ID")
            if url.path == "/api/run":
                return self.send_data(json.dumps(read_run(run), allow_nan=False).encode(), "application/json")
            if url.path == "/api/file":
                name = query.get("name", [""])[0]
                if name not in DOWNLOADS:
                    return self.send_error(400, "Invalid filename")
                return self.send_data((run / name).read_bytes(), "text/plain; charset=utf-8", name)
            self.send_error(404)
        except (OSError, ValueError):
            self.send_error(404, "Run not found")

    def send_data(self, data, content_type, filename=None):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", default="results")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, results=args.results))
    print(f"SimLap dashboard: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
