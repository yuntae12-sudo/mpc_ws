#!/usr/bin/env python3
"""Open the behavior dashboard after its HTTP server has started."""

import time
import webbrowser

import rospy


def main():
    rospy.init_node("behavior_dashboard_opener")
    url = rospy.get_param("~url", "http://localhost:8088/web/index.html")
    time.sleep(1.5)
    if not webbrowser.open(url):
        rospy.logwarn("Could not open a browser automatically. Open %s", url)


if __name__ == "__main__":
    main()
