#!/usr/bin/env python3
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point as RosPoint

positions = []
positions_lock = threading.Lock()

class EstPositionListener(Node):
    def __init__(self):
        super().__init__('est_position_listener')
        self.subscription = self.create_subscription(
            RosPoint,
            '/est_position',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global positions, positions_lock
        with positions_lock:
            positions.append((msg.x, msg.y, msg.z))
            if len(positions) > 1000:
                positions.pop(0)

def ros_spin():
    rclpy.init(args=None)
    node = EstPositionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

app = dash.Dash(__name__)
app.layout = html.Div([
    dcc.Graph(id='real-time-plot'),
    dcc.Interval(
        id='interval-update',
        interval=250,
        n_intervals=0
    )
])

@app.callback(
    Output('real-time-plot', 'figure'),
    [Input('interval-update', 'n_intervals')]
)
def update_plot(n_intervals):
    global positions, positions_lock
    with positions_lock:
        data = positions.copy()

    if not data:
        return go.Figure()

    x = [pt[0] for pt in data]
    y = [pt[1] for pt in data]
    z = [pt[2] for pt in data]

    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='lines+markers',
        marker=dict(size=5, color='blue'),
        line=dict(width=2, color='blue'),
        name="Estimated Trajectory"
    ))
    fig.update_layout(
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
    return fig

def main():
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8000, debug=True)

if __name__ == '__main__':
    main()
