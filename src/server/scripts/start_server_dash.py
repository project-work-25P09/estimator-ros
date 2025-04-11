#!/usr/bin/env python3
import pandas as pd
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
import os
import time
import threading

file = "./data/trajectory001.csv"
app = dash.Dash(__name__)

trajectory_name = os.path.basename(file1).split('.')[0]

def main():
    thread = threading.Thread(target=update_data_periodically)
    thread.daemon = True
    thread.start()
    app.run_server(host="0.0.0.0", port=8000, debug=True)

def read_and_clean_csv(file_path):
    delimiter_pattern = r"[,\t\s]+"
    df = pd.read_csv(file_path, delimiter=delimiter_pattern, engine="python")
    df.columns = df.columns.str.strip()
    return df

app.layout = html.Div([
    dcc.Graph(id='real-time-plot'),
    dcc.Interval(
        id='interval-update',
        interval=1000,
        n_intervals=0
    )
])

@app.callback(
    Output('real-time-plot', 'figure'),
    [Input('interval-update', 'n_intervals')]
)
def update_plot(n_intervals):
    df = read_and_clean_csv(file)

    if {'x', 'y', 'z'}.issubset(df.columns):
        new_fig = go.Figure()

        new_fig.add_trace(go.Scatter3d(
            x=df['x'], y=df['y'], z=df['z'],
            mode='lines+markers',
            marker=dict(size=5, color='blue'),
            line=dict(width=2, color='blue'),
            name=trajectory_name
        ))

        new_fig.update_layout(
            scene=dict(
                xaxis_title="X Axis",
                yaxis_title="Y Axis",
                zaxis_title="Z Axis",
                aspectmode="cube",
                camera=dict(
                    eye=dict(x=2, y=1, z=3)
                )
            ),
            margin=dict(l=0, r=0, b=0, t=20),
        )

        return new_fig

def update_data_periodically():
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
