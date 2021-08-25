import os
import multiprocessing
import threading
import signal
from typing import *
import dataclasses
import functools

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

import dash
import flask
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd

# Functionality resides in these submodules
from .node import Dashboard_Node
from .handler import Dashboard_Handler


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def main(args=None):
    rclpy.init()

    dashboard_node = Dashboard_Node()
    threading.Thread(target=ros2_thread, args=[dashboard_node]).start()

    def sigint_handler(signal, frame):
        """
        SIGINT handler
        We have to know when to tell rclpy to shut down, because
        it's in a child thread which would stall the main thread
        shutdown sequence. So we use this handler to call
        rclpy.shutdown() and then call the previously-installed
        SIGINT handler for Flask
        """
        rclpy.shutdown()
        if prev_sigint_handler is not None:
            prev_sigint_handler(signal)

    prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

    dashboard_handler = Dashboard_Handler(dashboard_node)
    dashboard_handler.run_server(debug=True)

if __name__ == '__main__':
    main()