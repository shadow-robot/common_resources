#!/usr/bin/python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
import threading
import rospy
import json
import urlparse
import rospkg

import cgi
import os


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
    pass


class SgsRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, *args):
        self.SUCCESS_CODE = 200
        self.FAIL_CODE = 503
        self.BAD_REQUEST_CODE = 400
        self.NOT_FOUND_CODE = 404
        BaseHTTPRequestHandler.__init__(self, *args)

    def do_GET(self):
        self.send_response(self.SUCCESS_CODE)
        page_path = rospkg.RosPack().get_path('sr_watchdog') + self.path
        if self.path == "/":
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            page_path = rospkg.RosPack().get_path('sr_watchdog') + self.path + "gui/main.html"
        elif self.path[-3:] == "css":
            self.send_header('Content-Type', 'text/css')
            self.end_headers()
        elif self.path[-4:] == "woff":
            self.send_header('Content-Type', 'font/woff')
            self.end_headers()
        elif self.path[-5:] == "woff2":
            self.send_header('Content-Type', 'font/woff2')
            self.end_headers()
        elif self.path[-3:] == "tff":
            self.send_header('Content-Type', 'font/tff')
            self.end_headers()
        elif self.path[-3:] == "eot":
            self.send_header('Content-Type', 'font/eot')
            self.end_headers()
        elif self.path[-2:] == "js":
            self.send_header('Content-Type', 'text/javascript')
            self.end_headers()
        elif self.path == "/main/favicon.ico":
            self.send_header('Content-Type', 'image/ico')
            self.end_headers()
        elif self.path[-3:] == "jpg":
            self.send_header('Content-Type', 'image/jpg')
            self.end_headers()
        elif self.path[-3:] == "png":
            self.send_header('Content-Type', 'image/png')
            self.end_headers()
        f = open(page_path)
        self.wfile.write(f.read())
        f.close()


if __name__ == "__main__":
    rospy.init_node("watchdog_webserver")

    port = rospy.get_param('~port', 8081)
    address = ("", port)
    server = ThreadedHTTPServer(address, SgsRequestHandler)

    rospy.loginfo("Serving at port %s" % port)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    rospy.spin()

    rospy.loginfo("Shutting down server")
    server.shutdown()
    rospy.signal_shutdown("Exit")
