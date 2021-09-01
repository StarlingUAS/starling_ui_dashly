import os
import threading
import signal
from typing import *
import dataclasses
import functools
import datetime

import dash
import flask
from dash.dependencies import Input, Output, State
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd

from ament_index_python.packages import get_package_share_directory

try:
    from .utils import app_callback, register_callbacks
    from .trajectory_manager_component import Trajectory_Component
    from .control_panel_manager_component import Control_Panel_Component
except Exception:
    from utils import app_callback, register_callbacks
    from trajectory_manager_component import Trajectory_Component
    from control_panel_manager_component import Control_Panel_Component

PACKAGE_NAME = 'starling_ui_dashly'
SITE_NAME = 'Starling Control Panel'
SITE_LOGO = '/static/starling.jpg'

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
            external_scripts=self.external_scripts,
            suppress_callback_exceptions=True)

        # Useful shared variables
        self.estop_active = False
        self.components = {
            'control': Control_Panel_Component(self.dashboard_node),
            'traj': Trajectory_Component(self.dashboard_node)
        }

        # Set up handling for different URLs
        self.site_paths = {
            '/': {
                "name": 'Control Panel',
                "desc": 'Primary control page',
                "func": self._generate_control_panel,
            },
            '/load_trajectories': {
                "name": 'Load Trajectories',
                'desc': 'Load one or more trajectories to execute',
                'func': self._generate_load_page
            }
        }

        self._register_static_assets()

        self._generate_app_layout()
        register_callbacks(self, self.app) # From Base Class

        for k, v in self.components.items(): # Register callbacks from component class members
            register_callbacks(v, self.app)

    def _register_static_assets(self):
        # Dash currently does not support loading of static documents
        # Therefore we add local routes to resources within the 'static' directory
        package_share_directory = get_package_share_directory(PACKAGE_NAME)
        image_directory = os.path.join(package_share_directory, 'static')
        list_of_images = [os.path.basename(x) for x in os.listdir(image_directory) if x.endswith('png') or x.endswith('jpg')]
        def serve_image(image_path):
            if image_path not in list_of_images:
                raise Exception('"{}" is excluded from the allowed static files'.format(image_path))
            return flask.send_from_directory(image_directory, image_path)
        self.app.server.route(f'/static/<image_path>')(serve_image)


    def _generate_app_layout(self):
        self.app.layout = html.Div(children=[
            dcc.Location(id='url', refresh=False),
            self._generate_navigation_bar(),
            html.Div([
                html.Div(id='page-content',
                style={
                    "height": "100%",
                })
            ], style={
                "height": "93vh",
                "width": "97vw",
                "margin": "auto"
            })
        ])

    def _generate_navigation_bar(self):
        self.layout_navbar = dbc.Navbar([
                html.A(
                    dbc.Row([
                            dbc.Col(html.Img(src=SITE_LOGO, height="30px")),
                            dbc.Col(dbc.NavbarBrand(SITE_NAME, className="ml-2"))
                        ],
                        align="center",
                        no_gutters=True
                    ),
                    href="/"
                ),
                dbc.Row([
                        dbc.Nav([
                            dbc.NavItem(dbc.NavLink(desc['name'], href=path))
                            for path, desc in self.site_paths.items()
                        ], pills=True, card=True, navbar=True)
                    ],
                    no_gutters=True,
                    className="ml-auto flex-nowrap mt-3 mt-md-0",
                    align="center",
                )
            ],
            color="dark",
            dark=True
        )
        return self.layout_navbar


    ######################################
    ###### Control Panel       ###########
    ######################################

    def _generate_control_panel(self):
        return self.components['control'].generate_layout()

    ######################################
    ###### Load Panel       ###########
    ######################################

    def _generate_load_page(self):
        return self.components['traj'].generate_layout()

    ###############################
    ########## UTILS ##############
    ###############################

    @app_callback(
        Output('page-content', 'children'),
        Input('url', 'pathname'))
    def __display_page(self, pathname):
        if pathname in self.site_paths:
            return self.site_paths[pathname]['func']()
        else:
            return '404'

    def run_server(self, **kwargs):
        self.app.run_server(**kwargs)



if __name__=='__main__':
    Dashboard_Handler(None).run_server(debug=True)