import os
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# In-memory "device" state
DEVICE_STATE = {
    "mode": "PID",
    "Kp": 1.0,
    "Ti": 1.0,
    "Td": 0.0,
    "system_stability": 0.0,
    "error_signal": 0.0,
    "output_signal": 0.0
}

# Allowed modes
VALID_MODES = {"P", "PI", "PD", "PID"}

def get_env(name, default=None, opt_type=str):
    val = os.environ.get(name)
    if val is None:
        if default is not None:
            return default
        else:
            raise RuntimeError(f"Missing environment variable: {name}")
    try:
        return opt_type(val)
    except Exception:
        raise RuntimeError(f"Invalid value for environment variable {name}: {val}")

class PIDControllerHandler(BaseHTTPRequestHandler):
    def _set_response(self, status=200, content_type="application/json"):
        self.send_response(status)
        self.send_header("Content-type", content_type)
        self.end_headers()

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length)
        try:
            payload = json.loads(body)
        except Exception:
            self._set_response(400)
            self.wfile.write(json.dumps({"error": "Invalid JSON"}).encode())
            return

        if self.path == "/mode":
            mode = payload.get("mode")
            if not mode or mode not in VALID_MODES:
                self._set_response(400)
                self.wfile.write(json.dumps({"error": f"Invalid mode. Must be one of {sorted(VALID_MODES)}"}).encode())
                return
            DEVICE_STATE["mode"] = mode
            self._set_response(200)
            self.wfile.write(json.dumps({"result": "Mode updated", "mode": mode}).encode())
            return

        elif self.path == "/pid":
            kp = payload.get("Kp")
            ti = payload.get("Ti")
            td = payload.get("Td")
            errors = []
            if kp is not None:
                try:
                    DEVICE_STATE["Kp"] = float(kp)
                except Exception:
                    errors.append("Kp must be a number")
            if ti is not None:
                try:
                    DEVICE_STATE["Ti"] = float(ti)
                except Exception:
                    errors.append("Ti must be a number")
            if td is not None:
                try:
                    DEVICE_STATE["Td"] = float(td)
                except Exception:
                    errors.append("Td must be a number")
            if errors:
                self._set_response(400)
                self.wfile.write(json.dumps({"error": errors}).encode())
                return
            self._set_response(200)
            self.wfile.write(json.dumps({
                "result": "PID parameters updated",
                "Kp": DEVICE_STATE["Kp"],
                "Ti": DEVICE_STATE["Ti"],
                "Td": DEVICE_STATE["Td"]
            }).encode())
            return

        else:
            self._set_response(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode())

    def do_GET(self):
        if self.path == "/" or self.path == "/state":
            self._set_response(200)
            self.wfile.write(json.dumps(DEVICE_STATE).encode())
        else:
            self._set_response(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode())

def main():
    server_host = get_env("HTTP_SERVER_HOST", "0.0.0.0")
    server_port = get_env("HTTP_SERVER_PORT", 8080, int)
    httpd = HTTPServer((server_host, server_port), PIDControllerHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()

if __name__ == "__main__":
    main()