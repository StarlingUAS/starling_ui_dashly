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

import dash
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd

from std_msgs.msg import String

class Dashboard_Node(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        # ROS Setup
        self.mission_start_publisher_ = self.create_publisher(String, 'mission_start', 10)
        self.emergency_stop_publisher_ = self.create_publisher(String, 'emergency_stop', 10)
        self.emergency_cutoff_publisher_ = self.create_publisher(String, 'emergency_cutoff', 10)
        timer_period = 0.5  # seconds

    def call_mission_start(self):
        msg = String()
        msg.data = 'mission_start'
        self.mission_start_publisher_.publish(msg)
        self.get_logger().info('mission_start published')

    def call_emergency_stop(self):
        msg = String()
        msg.data = 'emergency_stop'
        self.emergency_stop_publisher_.publish(msg)
        self.get_logger().info('emergency_stop published')

    def call_emergency_cutoff(self):
        msg = String()
        msg.data = 'emergency_cutoff'
        self.emergency_cutoff_publisher_.publish(msg)
        self.get_logger().info('emergency_cutoff published')

# Helper methods for registering callbacks
@dataclasses.dataclass
class CallbackMethod:
    args: List
    kwargs: Dict
    method: Callable

    def __call__(self, *args, **kwargs):
        return self.method(*args, **kwargs)

def app_callback(*args, **kwargs):
    def decorator(method):
        return CallbackMethod(args, kwargs, method)
    return decorator

class Dashboard_Handler():

    def __init__(self, dashboard_node):

        self.dashboard_node = dashboard_node

        self.name = 'Starling-Dashboard'
        self.external_stylesheets:List[str] = [dbc.themes.CERULEAN]
        self.external_scripts:List[str] = []

        self.app = dash.Dash(
            self.name,
            title=self.name,
            external_stylesheets=self.external_stylesheets,
            external_scripts=self.external_scripts)

        self._generate_app_layout()
        self._register_callbacks()

    def _generate_app_layout(self):
        self.app.layout = html.Div(children=[
            # html.H1(children='Starling Control Panel'),
            self._generate_navigation_bar(),


            html.Div([
                html.Div(dcc.Input(id='input-on-submit', type='text')),
                html.Button('Submit', id='submit-val', n_clicks=0),
                html.Div(id='container-button-basic',
                        children='Enter a value and press submit')
            ])

        ])

    def _generate_navigation_bar(self):
        self.layout_navbar = dbc.Navbar([
                dbc.Row([
                        # dbc.Col(html.Img(src=))
                        dbc.Col(dbc.NavbarBrand("Starling Control Panel", className="ml-2"))
                    ],
                    align="center",
                    no_gutters=True
                ),
            ],
            color="dark",
            dark=True
        )
        return self.layout_navbar

    def _register_callbacks(self):
        # Register Callbacks
        for k, v in vars(self.__class__).items():
            if isinstance(v, CallbackMethod):
                self.app.callback(*v.args, **v.kwargs)(functools.partial(v, self))

    @app_callback(
        dash.dependencies.Output('container-button-basic', 'children'),
        [dash.dependencies.Input('submit-val', 'n_clicks')],
        [dash.dependencies.State('input-on-submit', 'value')])
    def _update_output(self, n_clicks, value):
        if n_clicks > 0:
            self.dashboard_node.call_mission_start()
        return 'The input value was "{}" and the button has been clicked {} times'.format(
            value,
            n_clicks
        )

    def run_server(self, **kwargs):
        self.app.run_server(**kwargs)

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