"""Round-trip pull -> push against a fake Limelight. Run: python tools/test_limelight_config.py"""

import json
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import limelight_config as llc


class FakeLL(BaseHTTPRequestHandler):
    disk = {}

    def reply(self, obj, code=200):
        self.send_response(code)
        self.end_headers()
        self.wfile.write(json.dumps(obj).encode() if obj is not None else b"")

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/pipeline-atindex":
            self.reply(self.disk.get(int(parse_qs(u.query)["index"][0]), {"pipeline_type": "default"}))
        elif u.path in ("/status", "/hwreport"):
            self.reply({"ok": 1})
        elif u.path == "/cal-file":
            self.reply(self.disk.get("cal"), 200 if "cal" in self.disk else 404)
        else:
            self.reply(None, 404)

    def do_POST(self):
        u = urlparse(self.path)
        body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        if u.path == "/upload-pipeline":
            self.disk[int(parse_qs(u.query)["index"][0])] = json.loads(body)
        elif u.path == "/cal-file":
            self.disk["cal"] = json.loads(body)
        self.reply(None)

    def log_message(self, *_):
        pass


srv = ThreadingHTTPServer(("127.0.0.1", 5807), FakeLL)
threading.Thread(target=srv.serve_forever, daemon=True).start()

pipe = {"pipeline_type": "fiducial", "robot_pose": [0.1, 0.2, 0.3, 0, 15, 180]}
FakeLL.disk = {0: pipe, "cal": {"fx": 900}}
with tempfile.TemporaryDirectory() as d:
    llc.main(["pull", "127.0.0.1", "--dir", d])
    saved = Path(d, "127.0.0.1")
    assert json.loads((saved / "pipeline_0.json").read_text()) == pipe
    assert len(list(saved.glob("pipeline_*.json"))) == 10

    FakeLL.disk = {}  # "factory reset"
    llc.main(["push", "127.0.0.1", "--dir", d])
    assert FakeLL.disk[0] == pipe and FakeLL.disk["cal"] == {"fx": 900}
srv.shutdown()
print("ok")
