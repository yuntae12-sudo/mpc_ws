#!/usr/bin/env python3
"""Serve dashboard assets and proxy custom behavior messages over HTTP."""

import functools
import http.server
import json
import threading

import rospy
from behavior_planner.msg import BehaviorContext, UrbanFeatureDebug


def ros_to_dict(value):
    if hasattr(value, "to_sec"):
        return value.to_sec()
    if hasattr(value, "__slots__"):
        return {name: ros_to_dict(getattr(value, name)) for name in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [ros_to_dict(item) for item in value]
    return value


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    snapshot = {"context": None, "feature": None}
    snapshot_lock = threading.Lock()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/api/snapshot":
            with self.snapshot_lock:
                body = json.dumps(self.snapshot, separators=(",", ":")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def log_message(self, fmt, *args):
        # Avoid printing one line for every 10 Hz snapshot request.
        if not self.path.startswith("/api/snapshot"):
            super().log_message(fmt, *args)


def set_snapshot(key, message):
    with DashboardHandler.snapshot_lock:
        DashboardHandler.snapshot[key] = ros_to_dict(message)


def main():
    rospy.init_node("behavior_dashboard_server")
    port = int(rospy.get_param("~port", 8088))
    package_path = rospy.get_param("~package_path")
    rospy.Subscriber(
        "/behavior/context", BehaviorContext,
        lambda message: set_snapshot("context", message), queue_size=1)
    rospy.Subscriber(
        "/behavior/feature_debug", UrbanFeatureDebug,
        lambda message: set_snapshot("feature", message), queue_size=1)
    handler = functools.partial(DashboardHandler, directory=package_path)
    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo("Behavior dashboard: http://localhost:%d/web/index.html", port)
    rospy.on_shutdown(server.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
