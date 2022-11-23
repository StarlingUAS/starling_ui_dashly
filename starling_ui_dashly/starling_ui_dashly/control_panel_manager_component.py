import pandas as pd
import json

import dash
from dash.dependencies import Input, Output, State, MATCH, ALL
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
        vehic_namespace = self.dashboard_node.get_current_vehicle_namespaces()
        current_allocation = self.dashboard_node.get_current_allocation()
        return html.Div([ 
            html.P(f"System status update at {get_time()}, {len(vehic_namespace)} vehicles detected"),
            html.P("No allocation" if len(current_allocation) == 0 else f"Allocation Detected: {current_allocation}"),
            dbc.Row([
                dbc.Col([
                    html.Div([
                        dbc.Button(
                            f"Abort {vehicle_name}",
                            id={'type': f"mc_btn_mission_abort_drone", 'index': vehicle_name},
                            size="lg",
                            block=True,
                            color="warning"),
                        dbc.Button(
                            f"ESTOP {vehicle_name}",
                            id={'type': f"mc_btn_emergency_stop_drone", 'index': vehicle_name},
                            size="lg",
                            block=True,
                            color="danger"),
                    ],
                    className="d-grid gap-5")
                ])
                for vehicle_name in (vehic_namespace if len(current_allocation) == 0 else current_allocation)
            ]),
            html.Div("", id="mc_system_status_individual_vehicle_status_msg")
        ])

    @app_callback(
        Output("mc_system_status_individual_vehicle_status_msg", "children"),
        [Input({"type": "mc_btn_mission_abort_drone", "index": ALL}, 'n_clicks'),
         Input({"type": "mc_btn_emergency_stop_drone", "index": ALL}, 'n_clicks')]
    )
    def __control_panel_individual_drone_abort_press(self, abort_n_clicks, estop_n_clicks):
        ctx = dash.callback_context
        if ctx.triggered:
            trigger_id = ctx.triggered[0]['prop_id'].split('.')[0]
            trigger_dict = json.loads(trigger_id)
            vehicle_name = trigger_dict["index"]
            
            if trigger_dict["type"] == "mc_btn_mission_abort_drone":
                self.dashboard_node.call_mission_abort_drone(vehicle_name)
                return f"Abort {vehicle_name} pressed at {get_time()}"
            else:
                self.dashboard_node.send_emergency_stop_drone(vehicle_name)
                return f"ESTOP {vehicle_name} pressed at {get_time()}"

        return "Waiting for button press"

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