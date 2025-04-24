import os
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Environment variable configuration
DEVICE_IP = os.environ.get('DEVICE_IP', '127.0.0.1')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8000'))

# Simulated PID controller state
PID_STATE = {
    "mode": "PID",  # Default mode
    "Kp": 1.0,
    "Ti": 1.0,
    "Td": 0.0,
    "last_error": 0.0,
    "last_output": 0.0
}

VALID_MODES = {"P", "PI", "PD", "PID"}


class PIDControllerHTTPHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200, content_type='application/json'):
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def do_POST(self):
        parsed_path = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length > 0:
            raw_post_data = self.rfile.read(content_length).decode('utf-8')
            try:
                post_data = json.loads(raw_post_data)
            except Exception:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error": "Malformed JSON."}).encode())
                return
        else:
            post_data = {}

        if parsed_path.path == "/mode":
            self.handle_mode(post_data)
        elif parsed_path.path == "/pid":
            self.handle_pid(post_data)
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode())

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == "/status":
            self._set_headers(200)
            self.wfile.write(json.dumps({
                "mode": PID_STATE["mode"],
                "Kp": PID_STATE["Kp"],
                "Ti": PID_STATE["Ti"],
                "Td": PID_STATE["Td"],
                "last_error": PID_STATE["last_error"],
                "last_output": PID_STATE["last_output"]
            }).encode())
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode())

    def handle_mode(self, post_data):
        mode = post_data.get("mode")
        if mode not in VALID_MODES:
            self._set_headers(400)
            self.wfile.write(json.dumps({"error": "Invalid mode. Must be one of P, PI, PD, PID."}).encode())
            return
        PID_STATE["mode"] = mode
        self._set_headers(200)
        self.wfile.write(json.dumps({"status": "success", "mode": mode}).encode())

    def handle_pid(self, post_data):
        errors = []
        kp = post_data.get("Kp", PID_STATE["Kp"])
        ti = post_data.get("Ti", PID_STATE["Ti"])
        td = post_data.get("Td", PID_STATE["Td"])
        try:
            kp = float(kp)
        except Exception:
            errors.append("Kp must be a float.")
        try:
            ti = float(ti)
        except Exception:
            errors.append("Ti must be a float.")
        try:
            td = float(td)
        except Exception:
            errors.append("Td must be a float.")
        if errors:
            self._set_headers(400)
            self.wfile.write(json.dumps({"error": errors}).encode())
            return
        PID_STATE["Kp"] = kp
        PID_STATE["Ti"] = ti
        PID_STATE["Td"] = td
        self._set_headers(200)
        self.wfile.write(json.dumps({
            "status": "success",
            "Kp": kp,
            "Ti": ti,
            "Td": td
        }).encode())


def run():
    server_address = (SERVER_HOST, SERVER_PORT)
    httpd = HTTPServer(server_address, PIDControllerHTTPHandler)
    print(f"PID Controller HTTP server running at http://{SERVER_HOST}:{SERVER_PORT}")
    httpd.serve_forever()


if __name__ == "__main__":
    run()