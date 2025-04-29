import os
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib import request, parse, error

DEVICE_IP = os.environ.get("DEVICE_IP")
DEVICE_HTTP_PORT = int(os.environ.get("DEVICE_HTTP_PORT", 80))

SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", 8080))

DEVICE_BASE_URL = f"http://{DEVICE_IP}:{DEVICE_HTTP_PORT}"

def get_device_data():
    try:
        with request.urlopen(f"{DEVICE_BASE_URL}/api/v1/data", timeout=5) as resp:
            return resp.status, resp.read(), dict(resp.getheaders())
    except error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)
    except Exception as e:
        return 502, json.dumps({"error": f"Device unreachable: {str(e)}"}).encode(), {}

def post_device_ota():
    try:
        req = request.Request(f"{DEVICE_BASE_URL}/api/v1/ota", method="POST")
        with request.urlopen(req, timeout=5) as resp:
            return resp.status, resp.read(), dict(resp.getheaders())
    except error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)
    except Exception as e:
        return 502, json.dumps({"error": f"Device unreachable: {str(e)}"}).encode(), {}

def post_device_thresholds(payload):
    try:
        data = json.dumps(payload).encode()
        req = request.Request(f"{DEVICE_BASE_URL}/api/v1/thresh", data=data, headers={'Content-Type': 'application/json'}, method="POST")
        with request.urlopen(req, timeout=5) as resp:
            return resp.status, resp.read(), dict(resp.getheaders())
    except error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)
    except Exception as e:
        return 502, json.dumps({"error": f"Device unreachable: {str(e)}"}).encode(), {}

class SmartTempH100Driver(BaseHTTPRequestHandler):
    def _set_headers(self, status=200, headers=None):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        if headers:
            for k, v in headers.items():
                if k.lower() == "content-type":
                    continue
                self.send_header(k, v)
        self.end_headers()

    def do_GET(self):
        if self.path == "/data":
            status, body, headers = get_device_data()
            self._set_headers(status, headers)
            self.wfile.write(body)
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not found"}).encode())

    def do_POST(self):
        if self.path == "/ota":
            status, body, headers = post_device_ota()
            self._set_headers(status, headers)
            self.wfile.write(body)
        elif self.path == "/thresh":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length) if content_length > 0 else b''
            try:
                payload = json.loads(body)
                status, resp_body, headers = post_device_thresholds(payload)
                self._set_headers(status, headers)
                self.wfile.write(resp_body)
            except json.JSONDecodeError:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error": "Invalid JSON"}).encode())
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not found"}).encode())

def run():
    server_address = (SERVER_HOST, SERVER_PORT)
    httpd = HTTPServer(server_address, SmartTempH100Driver)
    print(f"SmartTemp-H100 Driver HTTP server running at http://{SERVER_HOST}:{SERVER_PORT}")
    httpd.serve_forever()

if __name__ == "__main__":
    if not DEVICE_IP:
        raise RuntimeError("DEVICE_IP environment variable must be set.")
    run()