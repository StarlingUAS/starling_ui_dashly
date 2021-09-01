import base64
import datetime
import io
import json
import copy

import dash
import dash_table
from dash.dependencies import Input, Output, State, MATCH, ALL
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html


import pandas as pd

try:
    from .utils import app_callback, register_callbacks, Dashboard_Component
except Exception:
    from utils import app_callback, register_callbacks, Dashboard_Component


HELP_TEXT = '''
### Loading Trajectories into Starling

One or more trajectory files can be uploaded using this page to be submitted to Starling to be run on one or more physical drones.
These can be uploaded by either clicking and selecting in the window, or by dragging and dropping files into the area.
Loaded trajectories will show in the list below the uploading box. There, the trajectories can be inspected and individually removed.
The trajectories can also be inspected in 3D on the right hand screen to ensure the correct trajectories have been loaded.

Once happy with the loaded trajectories, pressing the `submit` button will send **all loaded trajectories** over the `\submit_trajectories` service to the Allocator node to be executed by the vehicle.
The trajectories will be converted into [JointTrajectory.msg](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectory.html) and [JointTrajectoryPoint.msg](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html).

#### Single Trajectory Files

The format for these will be the following for all file formats.
The first row will be a header row where the first column is the `time_from_start` and the remaining columns are the target x, y and z position:


| Time      | x | y | z |
| ----------- | ----------- | ----------- | ----------- |
| 0.0      | 0.0      | 5.0 | 0.0 |
| 1.5   | 2.0        | 3.0 | -1.0 |
| 4.5      | 6.74      | 0.02 | -2.0 |

##### Supported File Formats:
These must be parseable by the `pandas` python utility

* .csv
* .xls

#### Multiple Trajectory Files

The utility will expect a JSON file (ending in `.json`) of the following format

```json
{
    "1": {
        "columns": ["time", "x", "y", "z"],
        "data": [
            [0.0, 0.0, 5.0, 0.0],
            [1.5, 2.0, 3.0, -1.0],
            [4.5, 6.75, 0.02, -2.0]
        ]
    },
    "2": {
        "columns": ["time", "x", "y", "z"],
        "data: [...]
    }
}
```

The `data` field of each trajectory block matches that of the single trajectory file. In each row, the first element is the `time_from_start`, and the remaining are the target x, y, and z position.
'''

TEST_DATAFRAME = pd.DataFrame([
                [1, 0, 0, 1],
                [2, 1, 0, 1],
                [3, 1, 1, 1],
                [4, 0, 1, 1],
                [5, 0, 0, 1],
            ], columns=['time', 'x', 'y', 'z'])

TEST_DATA = {
            'filename': 'test_file.csv',
            'content_type': '',
            'content_string': '',
            'data': TEST_DATAFRAME.to_dict(orient='records'),
            'columns': [c for c in TEST_DATAFRAME.columns]
        }

class Trajectory_Component(Dashboard_Component):

    def __init__(self, dashboard_node):
        self.dashboard_node = dashboard_node

    def generate_layout(self):
        return self._generate_load_page()

    def parse_trajectory_from_file(self, contents, filename, date):
        content_type, content_string = contents.split(',')

        decoded = base64.b64decode(content_string)
        single = True
        try:
            if filename.endswith('.csv'):
                # Assume that the user uploaded a CSV file
                df = pd.read_csv(
                    io.StringIO(decoded.decode('utf-8')))
            elif filename.endswith('.xls'):
                # Assume that the user uploaded an excel file
                df = pd.read_excel(io.BytesIO(decoded))
            elif filename.endswith('.json'):
                # Assume user uploaded a json file with multiple trajectories
                single = False
                output = json.loads(decoded.decode('utf-8'))
                df = [
                    pd.DataFrame(d['data'], columns=d['columns'])
                    for k, d in output.items()
                ]
        except Exception as e:
            print(e)
            return html.Div([
                'There was an error processing this file:' + filename
            ])

        if single:
            df = [df]

        return [{
                'filename': f'{f"traj{i}-" if not single else ""}{filename}',
                'content_type': content_type,
                'content_string': content_string,
                'columns': [c for c in _df.columns],
                'data': _df.to_dict(orient='records')
        } for i, _df in enumerate(df)]

    #########################################
    #######  Dashboard and Layouts   ########
    #########################################
    def _generate_load_page(self):
        # Main page
        self.layout_control_panel = html.Div([
            dbc.Row([
                dbc.Col(
                    self.__generate_load_traj_panel(),
                    align="center",
                    width=5,
                    style={"height":"95%", "border-style": "solid"}
                ),
                dbc.Col(
                    self.__generate_visualisation(),
                    align="center",
                    width=6,
                    style={"height":"95%", "border-style": "solid"}
                )
            ],
            # align="center",
            justify="around",
            style={"height": "100%"}
            )
        ], style={"height": "100%"})
        return self.layout_control_panel

    def __generate_help_screen_modal(self):
        return dbc.Modal(
            [
                dbc.ModalHeader("Help and User Guide"),
                dbc.ModalBody(dcc.Markdown(HELP_TEXT)),
                dbc.ModalFooter(
                    dbc.Button(
                        "Close", id="lt_help_screen_modal_btn_close", className="ml-auto", n_clicks=0
                    )
                ),
            ],
            id="lt_help_screen_modal",
            scrollable=True,
            is_open=False,
            size="lg"
        )
    @app_callback(
        Output("lt_help_screen_modal", "is_open"),
        [Input("lt_help_screen_modal_btn_open", "n_clicks"),
         Input("lt_help_screen_modal_btn_close", "n_clicks")],
        [State("lt_help_screen_modal", "is_open")],
    )
    def __toggle_help_screen_modal(self, n1, n2, is_open):
        if n1 or n2:
            return not is_open
        return is_open

    def __generate_load_traj_panel(self):
        return html.Div([
            dcc.Store(id='lt_store'),
            html.H2("Load Trajectories"),
            html.Hr(),
            self.__generate_help_screen_modal(),
            dbc.Row([
                dbc.Col(html.H5("Upload Joint Trajectory CSV files"), width=10, align="center"),
                dbc.Col(dbc.Button("Help", id="lt_help_screen_modal_btn_open", className="right", color="info"), width=2, align="center")
            ], style={"margin-bottom": "10px"}),
            dcc.Upload(
                id='lt_upload_data',
                children=html.Div([
                    'Drag and Drop or ',
                    html.A('Select Files')
                ]),
                style={
                    'width': '100%',
                    'height': '60px',
                    'lineHeight': '60px',
                    'borderWidth': '1px',
                    'borderStyle': 'dashed',
                    'borderRadius': '5px',
                    'textAlign': 'center',
                    'margin': 'auto',
                    'margin-bottom': '10px'
                },
                # Allow multiple files to be uploaded
                multiple=True
            ),
            dbc.Button('Confirm & Submit Trajectories (/submit_trajectories)', color='primary', block=True),
            html.Hr(),
            dbc.Row([
                dbc.Col(html.H6("Uploaded Trajectories:", id="lt_upload_trajectory_header"), align="center"),
                dbc.Col(dbc.Button("Clear", color="danger", id="lt_upload_trajectory_btn_clear", size='sm', style={'float':'right'}), align="center")
            ], style={"margin-bottom":"8px"}),
            html.Div(id='lt_output_data_upload'),
        ])

    def __generate_loaded_trajectory_list(self, loaded_trajectories):
        if len(loaded_trajectories) <= 0:
            return dbc.ListGroup([dbc.ListGroupItem("No trajectories uploaded", color="info")])
        return dbc.ListGroup([
                dbc.ListGroupItem([
                #    [dbc.ListGroupItemHeading(f'{i}. {traj["filename"]}'),
                #     dbc.ListGroupItemText(traj["df"].to_string())]
                    dbc.Row([
                        dbc.Col(html.H6(f'{i+1}) {traj["filename"]}', style={"text-align": "left", "padding": "8px"}), width=6, align="center"),
                        dbc.Col(dbc.ButtonGroup([
                            dbc.Button("Details", id={'type': "ld_traj_ui_btn_details", 'index': i}, color="info"),
                            dbc.Button("Duplicate", id={'type': "ld_traj_ui_btn_duplicate", 'index': i}, color="primary"),
                            dbc.Button("Delete", id={'type': "ld_traj_ui_btn_delete", 'index': i}, color="danger")
                        ], style={
                            "float": "right"
                        }), width=6)
                    ], justify="between"),
                    dbc.Collapse(
                        dbc.Card(dbc.CardBody([
                            dash_table.DataTable(
                                columns=[{"name": i, "id": i} for i in traj['columns']],
                                data=traj['data'],
                                style_cell={'textAlign': 'left'},
                            )
                        ]),
                        style={
                            'height': '150px',
                            'overflow': 'auto',
                        }
                        ),
                        id={'type':"ld_traj_ui_collapse_details", 'index': i},
                        is_open=False,
                    )
                ])
                for i, traj in enumerate(loaded_trajectories)
            ])

    @app_callback(
        [Output('lt_output_data_upload', 'children'),
         Output('lt_upload_data', 'contents'),
         Output('lt_store', 'data')],
        [Input('lt_upload_data', 'contents'),
         Input('lt_upload_trajectory_btn_clear', 'n_clicks'),
         Input({'type': "ld_traj_ui_btn_duplicate", 'index': ALL}, 'n_clicks'),
         Input({'type': "ld_traj_ui_btn_delete", 'index': ALL}, 'n_clicks')],
        [State('lt_upload_data', 'filename'),
         State('lt_upload_data', 'last_modified'),
         State('lt_store', 'data')])
    def upload_trajectories(self,
        list_of_contents, clear_btn_n_clicks, duplicate_btn_n_clicks, delete_btn_n_clicks,
        list_of_names, list_of_dates, json_store):

        if json_store:
            store = json.loads(json_store)
        else:
            store = {'traj': []}

        ctx = dash.callback_context
        if ctx.triggered:
            trigger_id = ctx.triggered[0]['prop_id'].split('.')[0]

            # Update list of trajectories if a new one uploaded
            if trigger_id=='lt_upload_data':

                if list_of_contents is not None:
                    for c, n, d in zip(list_of_contents, list_of_names, list_of_dates):
                        outputs = self.parse_trajectory_from_file(c, n, d)
                        print(f'Loaded {len(outputs)} new trajectorie(s) from {n}')
                        store['traj'].extend(outputs)

            # If clear button pressed, clear all trajectories
            elif trigger_id=='lt_upload_trajectory_btn_clear':
                print('Clearing all trajectories')
                store['traj'].clear()

            # If duplicate button pressed, duplicate this specific trajectory
            elif '{' in trigger_id:
                trigger_dict = json.loads(trigger_id)
                if trigger_dict["type"]=='ld_traj_ui_btn_duplicate':
                    index = int(trigger_dict["index"])
                    trajectory = store['traj'][index]
                    store['traj'].append(copy.copy(trajectory))
                    print(f'Duplicating trajectory {index} (trajectory: {trajectory["filename"]})')

                elif trigger_dict["type"]=='ld_traj_ui_btn_delete':
                    index = int(trigger_dict["index"])
                    del store['traj'][index]
                    print(f'Deleted trajectory {index}')


        # Set filename to None so it can accept the same file multiple times
        return self.__generate_loaded_trajectory_list(store['traj']), None, json.dumps(store)

    @app_callback(
        Output({'type':"ld_traj_ui_collapse_details", 'index': MATCH}, 'is_open'),
        [Input({'type': "ld_traj_ui_btn_details", 'index': MATCH}, 'n_clicks')],
        [State({'type':"ld_traj_ui_collapse_details", 'index': MATCH}, 'is_open')])
    def __display_loaded_trajectory_details(self, n_clicks, is_open):
        if n_clicks and n_clicks > 0:
            return not is_open
        return False

    def __generate_visualisation(self):
        return html.Div([])
