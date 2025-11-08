#!/usr/bin/env python3
"""Simple standalone web dashboard for direct motor testing.
   Provides four buttons (left/right forward/reverse) and stop.
   Sends commands over serial to the Arduino running motor test firmware.
"""

import os
import threading
import time
from http import HTTPStatus
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import serial

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>Motor Test Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f3f4f6; margin: 0; padding: 0; }
    header { background: #1f2937; color: white; padding: 1rem 2rem; }
    main { padding: 2rem; max-width: 600px; margin: 0 auto; }
    h1 { margin: 0 0 1rem 0; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 1rem; }
    button { padding: 1.25rem; font-size: 1.2rem; border: none; border-radius: 8px; cursor: pointer; background: #2563eb; color: white; }
    button:hover { background: #1d4ed8; }
    .stop { grid-column: 1 / -1; background: #dc2626; }
    .stop:hover { background: #b91c1c; }
    #status { margin-top: 1rem; color: #111827; }
  </style>
</head>
<body>
  <header>
    <h1>Motor Test Dashboard</h1>
    <p id="status">Connecting...</p>
  </header>
  <main>
    <div class="grid">
      <button data-cmd="LF">Left Forward</button>
      <button data-cmd="RF">Right Forward</button>
      <button data-cmd="LR">Left Reverse</button>
      <button data-cmd="RR">Right Reverse</button>
      <button class="stop" data-cmd="STOP">Stop</button>
    </div>
  </main>
  <script>
    async function send(cmd) {
      try {
        const res = await fetch('/api/cmd', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ command: cmd })
        });
        const data = await res.json();
        document.getElementById('status').textContent = data.message || 'Sent ' + cmd;
      } catch (err) {
        document.getElementById('status').textContent = 'Error: ' + err;
      }
    }
    document.querySelectorAll('button[data-cmd]').forEach(btn => {
      btn.addEventListener('click', () => send(btn.dataset.cmd));
    });
    document.getElementById('status').textContent = 'Ready';
  </script>
</body>
</html>
"""

SERIAL_PORT = os.environ.get('MOTOR_TEST_PORT', '/dev/ttyACM1')
BAUD = int(os.environ.get('MOTOR_TEST_BAUD', '115200'))

_serial_lock = threading.Lock()
_serial_conn = None


def get_serial():
    global _serial_conn
    with _serial_lock:
        if _serial_conn is not None and _serial_conn.is_open:
            return _serial_conn
        _serial_conn = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        time.sleep(2.0)  # allow board reset
        return _serial_conn


def send_command(cmd: str) -> str:
    ser = get_serial()
    with _serial_lock:
        ser.write((cmd.strip() + '\n').encode('utf-8'))
        ser.flush()
    return f"Command '{cmd}' sent"


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        if self.path in ('/', '/index.html'):
            content = HTML.encode('utf-8')
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        else:
            self.send_error(HTTPStatus.NOT_FOUND, 'Not Found')

    def do_POST(self):  # noqa: N802
        if self.path != '/api/cmd':
            self.send_error(HTTPStatus.NOT_FOUND, 'Not Found')
            return
        length = int(self.headers.get('Content-Length', '0'))
        body = self.rfile.read(length) if length > 0 else b''
        command = 'STOP'
        try:
            import json
            payload = json.loads(body.decode('utf-8') or '{}')
            command = payload.get('command', 'STOP')
        except Exception:
            pass
        try:
            message = send_command(command)
            response = { 'status': 'ok', 'message': message }
            payload = json.dumps(response).encode('utf-8')
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        except Exception as exc:
            self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
            payload = ("{\"status\": \"error\", \"message\": \"" + str(exc) + "\"}").encode('utf-8')
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    def log_message(self, format, *args):  # noqa: A003
        return


def main():
    host = os.environ.get('MOTOR_TEST_HOST', '0.0.0.0')
    port = int(os.environ.get('MOTOR_TEST_PORT_HTTP', '8091'))
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"Motor dashboard running at http://{host}:{port}/ -> Serial {SERIAL_PORT}@{BAUD}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        with _serial_lock:
            if _serial_conn and _serial_conn.is_open:
                _serial_conn.close()


if __name__ == '__main__':
    main()
