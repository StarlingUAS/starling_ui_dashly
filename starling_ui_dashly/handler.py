import os
import threading
import signal
from typing import *
import dataclasses
import functools

import dash
import flask
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'starling_ui_dashly'

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

        self._register_static_assets()

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
                        dbc.Col(html.Img(src='/static/starling.jpg', height="30px")),
                        dbc.Col(dbc.NavbarBrand("Starling Control Panel", className="ml-2"))
                    ],
                    align="center",
                    no_gutters=True
                ),
                dbc.Row([
                        dbc.Nav([
                            dbc.NavLink("Internal link", href="/l/components/nav"),
                            dbc.NavLink("External link", href="https://github.com"),
                            dbc.NavLink(
                                "External relative",
                                href="/l/components/nav",
                                external_link=True,
                            ),
                        ])
                    ],
                    no_gutters=True,
                    className="ml-auto flex-nowrap mt-3 mt-md-0",
                    align="center"
                )
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

    def _register_static_assets(self):
        package_share_directory = get_package_share_directory(PACKAGE_NAME)
        image_directory = os.path.join(package_share_directory, 'static')
        list_of_images = [os.path.basename(x) for x in os.listdir(image_directory) if x.endswith('png') or x.endswith('jpg')]
        def serve_image(image_path):
            if image_path not in list_of_images:
                raise Exception('"{}" is excluded from the allowed static files'.format(image_path))
            return flask.send_from_directory(image_directory, image_path)
        self.app.server.route(f'/static/<image_path>')(serve_image)

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
