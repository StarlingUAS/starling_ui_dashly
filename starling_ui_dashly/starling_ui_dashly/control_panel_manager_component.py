import pandas as pd

import dash
from dash.dependencies import Input, Output, State
from dash import dcc, html
import dash_bootstrap_components as dbc
# import dash_core_components as dcc
# import dash_html_components as html

try:
    from .utils import app_callback, register_callbacks, get_time, Dashboard_Component
except Exception:
    from utils import app_callback, register_callbacks, get_time, Dashboard_Component

class Control_Panel_Component(Dashboard_Component):

    def __init__(self, dashboard_node):
        self.dashboard_node = dashboard_node
        self.estop_active = False

    def generate_layout(self):
        return self._generate_control_panel()

    def generate_pre_layout(self):
        return html.Div([])

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
            html.Hr(),
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
            html.Hr(),
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
            return f"Status: Mission Start Topic Sent {get_time()}"
        else:
            return "Status:"

    @app_callback(
        Output("mc_btn_mission_abort_status_text", "children"),
        [Input("mc_btn_mission_abort", "n_clicks")])
    def __control_panel_btn_press_mission_abort(self, n_clicks):
        if n_clicks and n_clicks > 0:
            self.dashboard_node.call_mission_abort()
            return f"Status: Mission Abort Topic Sent {get_time()}"
        else:
            return "Status:"

    @app_callback(
        Output("mc_system_status_div", "children"),
        [Input("mc_system_status_btn_refresh", "n_clicks"),
         Input("mc_system_status_interval", "n_intervals")])
    def __control_panel_system_status_update(self, n_clicks, n_intervals):
        # Call self.dashboard_node.get_system_status()
        return f"System status update at {get_time()}"

    def __generate_control_panel_emergency_stop(self):
        return html.Div([
            html.H2("EMERGENCY STOP (/emergency_stop)", id="mc_title_estop"),
            html.H3("STATUS: DISENGAGED", id='mc_btn_estop_status', style={"color": "green"}),
            dbc.Row([
                dbc.Button(
                    html.Img(src='/static/estop-button.png', id='mc_btn_estop_img', style={"max-width": "90%"}),
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