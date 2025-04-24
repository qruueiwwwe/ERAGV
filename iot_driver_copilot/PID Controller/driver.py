import os
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Get configuration from environment variables
DEVICE_HOST = os.environ.get('DEVICE_HOST', '127.0.0.1')
DEVICE_PORT = int(os.environ.get('DEVICE_PORT', '9000'))  # Port for device protocol, if needed
HTTP_SERVER_HOST = os.environ.get('HTTP_SERVER_HOST', '0.0.0.0')
HTTP_SERVER_PORT = int(os.environ.get('HTTP_SERVER_PORT', '8080'))

# Simulated device state
device_state = {
    'mode': 'PID',  # Default mode
    'pid': {
        'Kp': 1.0,
        'Ti': 1.0,
        'Td': 0.0
    }
}

VALID_MODES = ['P', 'PI', 'PD', 'PID']

class PIDControllerHTTPRequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, code=200, content_type='application/json'):
        self.send_response(code)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def _handle_mode_post(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        try:
            payload = json.loads(body)
            mode = payload.get('mode')
            if not mode or mode not in VALID_MODES:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': f'Invalid mode. Valid modes: {VALID_MODES}'}).encode())
                return
            device_state['mode'] = mode
            self._set_headers(200)
            self.wfile.write(json.dumps({'status': 'success', 'mode': mode}).encode())
        except Exception as e:
            self._set_headers(400)
            self.wfile.write(json.dumps({'error': 'Invalid JSON or payload', 'detail': str(e)}).encode())

    def _handle_pid_post(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        try:
            payload = json.loads(body)
            kp = payload.get('Kp')
            ti = payload.get('Ti')
            td = payload.get('Td')
            if kp is None or ti is None or td is None:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': 'Kp, Ti, and Td must be specified'}).encode())
                return
            device_state['pid']['Kp'] = float(kp)
            device_state['pid']['Ti'] = float(ti)
            device_state['pid']['Td'] = float(td)
            self._set_headers(200)
            self.wfile.write(json.dumps({'status': 'success', 'pid': device_state['pid']}).encode())
        except Exception as e:
            self._set_headers(400)
            self.wfile.write(json.dumps({'error': 'Invalid JSON or payload', 'detail': str(e)}).encode())

    def _handle_get_status(self):
        self._set_headers(200)
        status = {
            'mode': device_state['mode'],
            'pid': device_state['pid']
        }
        self.wfile.write(json.dumps({'status': 'ok', 'device': status}).encode())

    def do_POST(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/mode':
            self._handle_mode_post()
        elif parsed_path.path == '/pid':
            self._handle_pid_post()
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({'error': 'Not found'}).encode())

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/':
            self._set_headers(200, 'text/html')
            html = """
            <html>
                <head><title>PID Controller HTTP Driver</title></head>
                <body>
                <h2>PID Controller Driver</h2>
                <p>POST /mode with {"mode": "P/PI/PD/PID"} to set mode.<br>
                   POST /pid with {"Kp": float, "Ti": float, "Td": float} to set PID params.<br>
                   GET /status for current state.</p>
                </body>
            </html>
            """
            self.wfile.write(html.encode())
        elif parsed_path.path == '/status':
            self._handle_get_status()
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({'error': 'Not found'}).encode())

def run(server_class=HTTPServer, handler_class=PIDControllerHTTPRequestHandler):
    httpd = server_class((HTTP_SERVER_HOST, HTTP_SERVER_PORT), handler_class)
    print(f"PID Controller HTTP Driver running at http://{HTTP_SERVER_HOST}:{HTTP_SERVER_PORT}/")
    httpd.serve_forever()

if __name__ == '__main__':
    run()