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
            external_scripts=self.external_scripts,
            suppress_callback_exceptions=True)

        # Useful shared variables
        self.estop_active = False

        self._register_static_assets()

        self._generate_app_layout()
        self._register_callbacks()

    def _register_callbacks(self):
        # Register Callbacks as actual app callbacks
        for k, v in vars(self.__class__).items():
            if isinstance(v, CallbackMethod):
                self.app.callback(*v.args, **v.kwargs)(functools.partial(v, self))

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
                            dbc.Col(html.Img(src='/static/starling.jpg', height="30px")),
                            dbc.Col(dbc.NavbarBrand("Starling Control Panel", className="ml-2"))
                        ],
                        align="center",
                        no_gutters=True
                    ),
                    href="/"
                ),
                dbc.Row([
                        dbc.Nav([
                            dbc.NavItem(dbc.NavLink("Control Panel", href="/")),
                            dbc.NavItem(dbc.NavLink("Load Resource", href="/load")),
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
        # Main page
        self.layout_control_panel = html.Div([
            dbc.Row([
                dbc.Col(
                    self.__generate_control_panel_mission_control(),
                    align="center",
                    width=6,
                    style={"height":"95%", "border-style": "solid"}
                ),
                dbc.Col(
                    self.__generate_control_panel_emergency_stop(),
                    align="center",
                    width=5,
                    style={"height":"95%", "border-style": "solid"}
                )
            ],
            # align="center",
            justify="around",
            style={"height": "100%"}
            )
        ], style={"height": "100%"})
        return self.layout_control_panel

    def __generate_control_panel_mission_control(self):
        return html.Div([
            html.H2("MISSION CONTROL"),
            dbc.Row([
                dbc.Col(html.Div([
                    html.H3("Start Mission (/mission_start)"),
                    dbc.Button(
                        html.Img(src='/static/go-button.png', style={"max-width": "75%"}),
                        id="mc_btn_mission_start",
                        size="lg",
                        outline=True,
                        color="success"),
                    html.P("Status: ", id="mc_btn_mission_start_status_text"),
                ])),
                dbc.Col(html.Div([
                    html.H3("Cancel Mission (/mission_abort)"),
                    dbc.Button(
                        html.Img(src='/static/stop-button.png', style={"max-width": "75%"}),
                        id="mc_btn_mission_abort",
                        size="lg",
                        outline=True,
                        color="warning"),
                    html.P("Status: ", id="mc_btn_mission_abort_status_text"),
                ])),
            ],
            justify="center",
            ),
            dbc.Row([
                dbc.Col(html.H2("SYSTEM STATUS")),
                dbc.Col(dbc.FormGroup([
                    dbc.Checkbox(id="mc_system_status_refresh_toggle", className="form-check-input", checked=False),
                    dbc.Label("Auto-Refresh (2s)", html_for="mc_system_status_refresh_toggle", className="form-check-label"),
                ], check=True)),
                dbc.Col(dbc.Button("REFRESH", id="mc_system_status_btn_refresh", block=True, color="info"), align="center"),
            ]),
            dcc.Interval("mc_system_status_interval", interval=2*1000),
            html.Div(id="mc_system_status_div")
        ])

    @app_callback(
        Output("mc_system_status_interval", "disabled"),
        [Input("mc_system_status_refresh_toggle", "checked")])
    def __control_panel_system_status_auto_refresh_toggle(self, checked):
        return not checked

    @app_callback(
        Output("mc_btn_mission_start_status_text", "children"),
        [Input("mc_btn_mission_start", "n_clicks")])
    def __control_panel_btn_press_mission_start(self, n_clicks):
        if n_clicks and n_clicks > 0:
            self.dashboard_node.call_mission_start()
            return f"Status: Mission Start Topic Sent {self._get_time()}"
        else:
            return "Status:"

    @app_callback(
        Output("mc_btn_mission_abort_status_text", "children"),
        [Input("mc_btn_mission_abort", "n_clicks")])
    def __control_panel_btn_press_mission_abort(self, n_clicks):
        if n_clicks and n_clicks > 0:
            self.dashboard_node.call_mission_abort()
            return f"Status: Mission Abort Topic Sent {self._get_time()}"
        else:
            return "Status:"

    @app_callback(
        Output("mc_system_status_div", "children"),
        [Input("mc_system_status_btn_refresh", "n_clicks"),
         Input("mc_system_status_interval", "n_intervals")])
    def __control_panel_system_status_update(self, n_clicks, n_intervals):
        # Call self.dashboard_node.get_system_status()
        return f"System status update at {self._get_time()}"

    def __generate_control_panel_emergency_stop(self):
        return html.Div([
            html.H2("EMERGENCY STOP (/emergency_stop)", id="mc_title_estop"),
            html.H3("STATUS: DISENGAGED", id='mc_btn_estop_status', style={"color": "green"}),
            dbc.Row([
                dbc.Button(
                    html.Img(src='/static/estop-button.png', id='mc_btn_estop_img', style={"max-width": "100%", "max-height": "100%"}),
                    id="mc_btn_estop",
                    size="lg",
                    outline=True,
                    color="danger"
                )
                ],
                align="center",
                justify="center"
            ),
            html.H4("Ensure ESTOP is disengaged before flight"),
        ])

    @app_callback(
        [Output("mc_btn_estop_status", "children"),
         Output("mc_btn_estop_status", "style"),
         Output("mc_btn_estop", "active"),
         Output("mc_btn_estop_img", "style")],
        [Input("mc_btn_estop", "n_clicks")]
    )
    def __control_panel_btn_press_estop(self, n_clicks):
        if n_clicks and n_clicks > 0:
            self.estop_active = not self.estop_active
            self.dashboard_node.toggle_emergency_stop(self.estop_active)
            if self.estop_active:
                return "STATUS: ENGAGED", {"color": "red"}, True, {"filter": "brightness(50%)"}
        return "STATUS: DISENGAGED", {"color": "green"}, False, {"filter": "brightness(100%)"}

    ######################################
    ###### Load Panel       ###########
    ######################################

    def _generate_load_page(self):
        # Load Resources page
        self.layout_load_page = html.Div([
            html.H1(children='LOAD RESOURCES')
        ])
        return self.layout_load_page

    @app_callback(
        Output('page-content', 'children'),
        Input('url', 'pathname'))
    def __display_page(self, pathname):
        if pathname == '/':
            return self._generate_control_panel()
        elif pathname == '/load':
            return self._generate_load_page()
        else:
            return '404'

    def run_server(self, **kwargs):
        self.app.run_server(**kwargs)

    ###############################
    ########## UTILS ##############
    ###############################
    def _get_time(self, format="%H:%M:%S"):
        x = datetime.datetime.now()
        return x.strftime(format)


if __name__=='__main__':
    Dashboard_Handler(None).run_server(debug=True)