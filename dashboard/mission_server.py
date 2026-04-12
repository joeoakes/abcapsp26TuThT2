import json
import os
import ssl
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT = 8080
MISSION_FILE = "/tmp/latest_mission.json"
DASHBOARD_FILE = os.path.join(os.path.dirname(__file__), "Mission_Dashboard_HTML")

CERT_FILE = os.environ.get("TLS_CERT", os.path.join(os.path.dirname(__file__),
                           "../https/certs/server.crt"))
KEY_FILE  = os.environ.get("TLS_KEY",  os.path.join(os.path.dirname(__file__),
                           "../https/certs/server.key"))

class MissionHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        print(f"[SERVER] {self.address_string()} - {format % args}")

    def _send(self, code, content_type, body):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path == "/" or self.path == "/dashboard":
            # Serve the dashboard HTML
            try:
                with open(DASHBOARD_FILE, "rb") as f:
                    html = f.read()
                self._send(200, "text/html; charset=utf-8", html)
            except FileNotFoundError:
                self._send(404, "text/plain", "dashboard.html not found")

        elif self.path == "/latest_mission":
            # Return the latest mission JSON
            if os.path.exists(MISSION_FILE):
                with open(MISSION_FILE, "r") as f:
                    data = f.read()
                self._send(200, "application/json", data)
            else:
                self._send(200, "application/json", json.dumps({"status": "no_mission_yet"}))

        elif self.path == "/health":
            self._send(200, "application/json", json.dumps({"ok": True}))

        else:
            self._send(404, "text/plain", "Not found")

    def do_POST(self):
        if self.path == "/mission_end":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length).decode("utf-8")

            try:
                data = json.loads(body)
                # Save to file
                with open(MISSION_FILE, "w") as f:
                    json.dump(data, f)
                print(f"[SERVER] Mission saved: result={data.get('mission_result','?')} "
                      f"moves={data.get('moves_total','?')}")
                self._send(200, "application/json", json.dumps({"ok": True}))
            except json.JSONDecodeError as e:
                print(f"[SERVER] Bad JSON: {e}")
                self._send(400, "application/json", json.dumps({"error": str(e)}))
        else:
            self._send(404, "text/plain", "Not found")


if __name__ == "__main__":
    if not os.path.exists(CERT_FILE):
        print(f"[ERROR] TLS cert not found: {CERT_FILE}")
        raise SystemExit(1)
    if not os.path.exists(KEY_FILE):
        print(f"[ERROR] TLS key not found: {KEY_FILE}")
        raise SystemExit(1)

    server = HTTPServer(("0.0.0.0", PORT), MissionHandler)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=CERT_FILE, keyfile=KEY_FILE)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"[SERVER] Mission server running on https://0.0.0.0:{PORT}")
    print(f"[SERVER] Dashboard: https://localhost:{PORT}/")
    print(f"[SERVER] Mission endpoint: POST https://localhost:{PORT}/mission_end")
    print(f"[SERVER] Latest data: GET https://localhost:{PORT}/latest_mission")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[SERVER] Shutting down.")
        server.server_close()
