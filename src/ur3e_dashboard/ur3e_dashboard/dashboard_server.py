"""Web dashboard for driving the UR3e pick-and-place cell.

A ROS node plus a plain-stdlib HTTP server on a background thread. Deliberately
no FastAPI/uvicorn/rosbridge: the whole point is that this runs from a bare
ROS Humble install with nothing to pip install.

  GET  /              the single-page UI
  GET  /api/state     latest orchestrator status (JSON, plus camera liveness)
  POST /api/task      {"task": "...", "x": .., "y": ..} -> calls /ur3e/run_task
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ur3e_msgs.srv import RunTask

# How long after the last message a feed is still considered live.
STALE_AFTER = 3.0


class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')

        self.declare_parameter('port', 8080)
        self.declare_parameter('bind', '127.0.0.1')
        self.port = int(self.get_parameter('port').value)
        self.bind = str(self.get_parameter('bind').value)

        cbg = ReentrantCallbackGroup()

        self._lock = threading.Lock()
        self._status = {}
        self._status_stamp = 0.0
        self._camera_stamp = 0.0

        self.create_subscription(
            String, '/ur3e/task_status', self._status_cb, 10,
            callback_group=cbg)
        # Only used for a liveness dot in the UI - decoding frames here would
        # cost far more than it tells the operator.
        self.create_subscription(
            Image, '/camera/image_raw', self._camera_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=cbg)

        self._task_client = self.create_client(
            RunTask, '/ur3e/run_task', callback_group=cbg)

        share = get_package_share_directory('ur3e_dashboard')
        self._index = os.path.join(share, 'static', 'index.html')

        self._httpd = ThreadingHTTPServer(
            (self.bind, self.port), _make_handler(self))
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'Dashboard on http://{self.bind}:{self.port}')

    # --- ROS side --------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._status = data
            self._status_stamp = self._now()

    def _camera_cb(self, _msg: Image) -> None:
        with self._lock:
            self._camera_stamp = self._now()

    # --- served to the browser -------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            status = dict(self._status)
            age = self._now() - self._status_stamp if self._status_stamp else None
            cam_age = (self._now() - self._camera_stamp
                       if self._camera_stamp else None)
        status['orchestrator_online'] = age is not None and age < STALE_AFTER
        status['camera_online'] = cam_age is not None and cam_age < STALE_AFTER
        status['status_age'] = round(age, 2) if age is not None else None
        return status

    def run_task(self, task: str, x: float, y: float) -> dict:
        if not self._task_client.wait_for_service(timeout_sec=2.0):
            return {'accepted': False,
                    'message': 'orchestrator not running (/ur3e/run_task absent)'}
        req = RunTask.Request()
        req.task = task
        req.x = float(x)
        req.y = float(y)
        future = self._task_client.call_async(req)
        # The node spins on a MultiThreadedExecutor and this runs on an HTTP
        # worker thread, so waiting on the future here does not deadlock.
        if not _wait(future, timeout=5.0):
            return {'accepted': False, 'message': 'task call timed out'}
        res = future.result()
        return {'accepted': bool(res.accepted), 'message': str(res.message)}


def _wait(future, timeout: float) -> bool:
    ev = threading.Event()
    future.add_done_callback(lambda _f: ev.set())
    return ev.wait(timeout)


def _make_handler(node: DashboardServer):
    class Handler(BaseHTTPRequestHandler):
        # Silence the default per-request stderr logging.
        def log_message(self, *_args):
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, payload: dict) -> None:
            self._send(code, json.dumps(payload).encode(), 'application/json')

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                try:
                    with open(node._index, 'rb') as fh:
                        self._send(200, fh.read(), 'text/html; charset=utf-8')
                except OSError as exc:
                    self._send(500, f'index missing: {exc}'.encode(),
                               'text/plain')
            elif self.path == '/api/state':
                self._json(200, node.snapshot())
            else:
                self._send(404, b'not found', 'text/plain')

        def do_POST(self):
            if self.path != '/api/task':
                self._send(404, b'not found', 'text/plain')
                return
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError) as exc:
                self._json(400, {'accepted': False,
                                 'message': f'bad request: {exc}'})
                return
            task = str(body.get('task', '')).strip().lower()
            if not task:
                self._json(400, {'accepted': False, 'message': 'task required'})
                return
            try:
                x = float(body.get('x', 0.0))
                y = float(body.get('y', 0.0))
            except (TypeError, ValueError):
                self._json(400, {'accepted': False,
                                 'message': 'x and y must be numbers'})
                return
            self._json(200, node.run_task(task, x, y))

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
