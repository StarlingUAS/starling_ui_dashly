import pandas as pd

import dash
from dash.dependencies import Input, Output, State
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html

try:
    from .utils import app_callback, register_callbacks, Dashboard_Component
except Exception:
    from utils import app_callback, register_callbacks, Dashboard_Component

class Trajectory_Component(Dashboard_Component):

    def __init__(self):
        pass

    def generate_layout(self):
        return self._generate_load_page()

    def _generate_load_page(self):
        # Load Resources page
        self.layout_load_page = html.Div([
            html.H1(children='LOAD RESOURCES')
        ])
        return self.layout_load_page