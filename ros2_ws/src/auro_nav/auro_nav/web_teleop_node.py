"""Minimal web-based teleop for driving the robot from any browser."""

import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from functools import partial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Auro Teleop</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: -apple-system, sans-serif; background: #1a1a2e; color: #eee;
       display: flex; flex-direction: column; align-items: center; height: 100vh;
       touch-action: none; user-select: none; }
h1 { margin: 12px 0 4px; font-size: 1.4em; color: #0ff; }
.status { font-size: 0.85em; color: #888; margin-bottom: 8px; }
.speed-display { display: flex; gap: 20px; margin-bottom: 10px; font-size: 0.95em; }
.speed-display span { color: #0f0; }
.dpad { display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px);
        gap: 6px; margin-bottom: 12px; }
.dpad button { border: 2px solid #444; border-radius: 12px; background: #2a2a4a;
               color: #fff; font-size: 1.6em; cursor: pointer; transition: background 0.1s; }
.dpad button:active, .dpad button.active { background: #0066cc; border-color: #0ff; }
.dpad button.stop { background: #661a1a; border-color: #f44; font-size: 1.1em; }
.dpad button.stop:active, .dpad button.stop.active { background: #cc0000; }
.dpad button.empty { visibility: hidden; }
.speed-controls { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; }
.speed-controls button { padding: 10px 16px; border: 1px solid #555; border-radius: 8px;
                          background: #2a2a4a; color: #eee; font-size: 0.9em; cursor: pointer; }
.speed-controls button:active { background: #444; }
.keys-hint { margin-top: 12px; font-size: 0.75em; color: #666; text-align: center; line-height: 1.6; }
</style>
</head>
<body>

<h1>Auro Teleop</h1>
<div class="status" id="status">Connecting...</div>
<div class="speed-display">
  Lin: <span id="lin">0.20</span> m/s &nbsp;|&nbsp; Ang: <span id="ang">0.50</span> rad/s
</div>

<div class="dpad">
  <button data-cmd="fwd_left"  >&#8598;</button>
  <button data-cmd="forward"   >&#8593;</button>
  <button data-cmd="fwd_right" >&#8599;</button>
  <button data-cmd="left"      >&#8592;</button>
  <button data-cmd="stop" class="stop">STOP</button>
  <button data-cmd="right"     >&#8594;</button>
  <button data-cmd="back_left" >&#8601;</button>
  <button data-cmd="backward"  >&#8595;</button>
  <button data-cmd="back_right">&#8600;</button>
</div>

<div class="speed-controls">
  <button onclick="adjSpeed('lin', -0.02)">Lin &minus;</button>
  <button onclick="adjSpeed('lin', +0.02)">Lin +</button>
  <button onclick="adjSpeed('ang', -0.1)">Ang &minus;</button>
  <button onclick="adjSpeed('ang', +0.1)">Ang +</button>
</div>

<div class="keys-hint">
  Keyboard: WASD or arrow keys to drive, Space = stop<br>
  Q/Z = lin speed, E/C = ang speed
</div>

<script>
let lin = 0.20, ang = 0.50;
let activeCmd = null;
let sendInterval = null;

function adjSpeed(type, delta) {
  if (type === 'lin') lin = Math.max(0.02, Math.min(1.0, +(lin + delta).toFixed(2)));
  else ang = Math.max(0.1, Math.min(3.0, +(ang + delta).toFixed(2)));
  document.getElementById('lin').textContent = lin.toFixed(2);
  document.getElementById('ang').textContent = ang.toFixed(2);
}

const cmds = {
  forward:    () => [lin, 0],
  backward:   () => [-lin, 0],
  left:       () => [0, ang],
  right:      () => [0, -ang],
  fwd_left:   () => [lin, ang],
  fwd_right:  () => [lin, -ang],
  back_left:  () => [-lin, ang],
  back_right: () => [-lin, -ang],
  stop:       () => [0, 0],
};

function sendCmd(cmd) {
  const [lx, az] = cmds[cmd]();
  fetch('/cmd', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({linear_x: lx, angular_z: az}),
  }).then(r => {
    document.getElementById('status').textContent = 'Connected';
    document.getElementById('status').style.color = '#0f0';
  }).catch(() => {
    document.getElementById('status').textContent = 'Disconnected';
    document.getElementById('status').style.color = '#f44';
  });
}

function startCmd(cmd) {
  if (activeCmd === cmd) return;
  stopCmd();
  activeCmd = cmd;
  sendCmd(cmd);
  if (cmd !== 'stop') {
    sendInterval = setInterval(() => sendCmd(cmd), 200);
  }
  document.querySelectorAll('.dpad button').forEach(b => b.classList.remove('active'));
  const btn = document.querySelector(`[data-cmd="${cmd}"]`);
  if (btn) btn.classList.add('active');
}

function stopCmd() {
  if (sendInterval) { clearInterval(sendInterval); sendInterval = null; }
  activeCmd = null;
  sendCmd('stop');
  document.querySelectorAll('.dpad button').forEach(b => b.classList.remove('active'));
}

// Touch events
document.querySelectorAll('.dpad button').forEach(btn => {
  const cmd = btn.dataset.cmd;
  btn.addEventListener('touchstart', e => { e.preventDefault(); startCmd(cmd); });
  btn.addEventListener('touchend', e => { e.preventDefault(); if (cmd !== 'stop') stopCmd(); });
  btn.addEventListener('mousedown', e => { e.preventDefault(); startCmd(cmd); });
  btn.addEventListener('mouseup', e => { e.preventDefault(); if (cmd !== 'stop') stopCmd(); });
});

// Keyboard events
const keyMap = {
  ArrowUp: 'forward', ArrowDown: 'backward', ArrowLeft: 'left', ArrowRight: 'right',
  w: 'forward', s: 'backward', a: 'left', d: 'right',
  W: 'forward', S: 'backward', A: 'left', D: 'right',
};

document.addEventListener('keydown', e => {
  if (e.repeat) return;
  if (e.key === ' ') { e.preventDefault(); startCmd('stop'); return; }
  if (e.key === 'q' || e.key === 'Q') { adjSpeed('lin', 0.02); return; }
  if (e.key === 'z' || e.key === 'Z') { adjSpeed('lin', -0.02); return; }
  if (e.key === 'e' || e.key === 'E') { adjSpeed('ang', 0.1); return; }
  if (e.key === 'c' || e.key === 'C') { adjSpeed('ang', -0.1); return; }
  const cmd = keyMap[e.key];
  if (cmd) { e.preventDefault(); startCmd(cmd); }
});

document.addEventListener('keyup', e => {
  const cmd = keyMap[e.key];
  if (cmd && activeCmd === cmd) stopCmd();
});

document.getElementById('status').textContent = 'Ready';
document.getElementById('status').style.color = '#0f0';
</script>
</body>
</html>"""


class WebTeleopHandler(BaseHTTPRequestHandler):
    def __init__(self, publisher, logger, *args, **kwargs):
        self.publisher = publisher
        self.logger = logger
        super().__init__(*args, **kwargs)

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode())

    def do_POST(self):
        if self.path == '/cmd':
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            msg = Twist()
            msg.linear.x = float(body.get('linear_x', 0.0))
            msg.angular.z = float(body.get('angular_z', 0.0))
            self.publisher.publish(msg)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # suppress per-request logging


class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        handler = partial(WebTeleopHandler, self.pub, self.get_logger())
        self.server = HTTPServer(('0.0.0.0', port), handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Web teleop running at http://0.0.0.0:{port}')


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
