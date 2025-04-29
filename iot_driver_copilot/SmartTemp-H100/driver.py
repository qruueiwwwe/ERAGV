import os
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import threading
import requests

# Configuration from Environment Variables
DEVICE_HOST = os.environ.get('DEVICE_HOST', '127.0.0.1')
DEVICE_HTTP_PORT = int(os.environ.get('DEVICE_HTTP_PORT', '80'))
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))

# Helper Functions for Device Communication via HTTP/REST API
def device_get_data():
    url = f'http://{DEVICE_HOST}:{DEVICE_HTTP_PORT}/api/data'
    try:
        resp = requests.get(url, timeout=5)
        resp.raise_for_status()
        return resp.json(), 200
    except Exception as e:
        return {"error": f"Device unreachable: {str(e)}"}, 502

def device_post_ota():
    url = f'http://{DEVICE_HOST}:{DEVICE_HTTP_PORT}/api/ota'
    try:
        resp = requests.post(url, timeout=10)
        resp.raise_for_status()
        return resp.json() if resp.content else {"status": "OTA triggered"}, 200
    except Exception as e:
        return {"error": f"OTA failed: {str(e)}"}, 502

def device_post_thresh(payload):
    url = f'http://{DEVICE_HOST}:{DEVICE_HTTP_PORT}/api/thresh'
    try:
        resp = requests.post(url, json=payload, timeout=5)
        resp.raise_for_status()
        return resp.json() if resp.content else {"status": "Thresholds updated"}, 200
    except Exception as e:
        return {"error": f"Threshold update failed: {str(e)}"}, 502

# HTTP Handler
class SmartTempHandler(BaseHTTPRequestHandler):
    def _set_headers(self, code=200, content_type="application/json"):
        self.send_response(code)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        
    def _parse_json(self):
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                return None
            return json.loads(self.rfile.read(content_length).decode('utf-8'))
        except Exception:
            return None

    def do_OPTIONS(self):
        self._set_headers(200)
    
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/data":
            data, code = device_get_data()
            self._set_headers(code)
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not found"}).encode('utf-8'))

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/ota":
            data, code = device_post_ota()
            self._set_headers(code)
            self.wfile.write(json.dumps(data).encode('utf-8'))
        elif parsed.path == "/thresh":
            payload = self._parse_json()
            if not payload or not isinstance(payload, dict):
                self._set_headers(400)
                self.wfile.write(json.dumps({"error": "Invalid JSON payload"}).encode('utf-8'))
                return
            data, code = device_post_thresh(payload)
            self._set_headers(code)
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not found"}).encode('utf-8'))

# Threaded HTTP Server
class ThreadedHTTPServer(HTTPServer, threading.Thread):
    def run(self):
        self.serve_forever()

def main():
    server_address = (SERVER_HOST, SERVER_PORT)
    httpd = HTTPServer(server_address, SmartTempHandler)
    print(f"SmartTemp-H100 HTTP Driver running at http://{SERVER_HOST}:{SERVER_PORT}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()

if __name__ == '__main__':
    main()