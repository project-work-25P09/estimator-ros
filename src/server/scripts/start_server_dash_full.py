#!/usr/bin/env python3

# sama kuin csv3D.py

import pandas as pd
import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objects as go
import os
import numpy as np
import threading
import time

import rclpy
from rclpy.node import Node
from estimation.msg import Estimation

data_ros = []
data_ros_lock = threading.Lock()

data_x_idx, data_y_idx, data_z_idx, data_yaw_idx, data_pitch_idx, data_roll_idx, data_acc_x_idx, data_acc_y_idx, data_acc_z_idx, data_acc_yaw_idx, data_acc_pitch_idx, data_acc_roll_idx, data_mag_x_idx, data_mag_y_idx, data_mag_z_idx, data_mag_strength_idx, data_mouse_movement_idx, data_mouse_speed_idx, data_mouse_direction_idx, data_mouse_distance_idx = range(20)

class EstimationListener(Node):
    def __init__(self):
        super().__init__('estimation_listener')
        self.subscription = self.create_subscription(
            Estimation,
            '/estimation',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global data_ros, data_ros_lock
        with data_ros_lock:
            # print(f"Recieved message: {msg.x:0.2f}, {time.time():0.2f}")
            data_ros.append((msg.x, msg.y, msg.z, msg.yaw, msg.pitch, msg.roll, msg.acc_x, msg.acc_y, msg.acc_z, msg.acc_yaw, msg.acc_pitch, msg.acc_roll, msg.mag_x, msg.mag_y, msg.mag_z, msg.mag_strength, msg.mouse_movement, msg.mouse_speed, msg.mouse_direction, msg.mouse_distance))
            if len(data_ros) > 1000:
                data_ros.pop(0)


def ros_spin():
    rclpy.init(args=None)
    node = EstimationListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Define the file paths
# file1 = "./data/niko_data/3D_data.csv"
file2 = "./data/niko_data/3D_real.csv"
# rpy_file = "./data/niko_data/rpy.csv"  # Add the RPY file path
acc_file = "./data/niko_data/acc.csv"  # Add the acceleration file path
mouse_file = "./data/niko_data/mouse.csv"  # Add the mouse file path
macc_file = "./data/niko_data/Macc.csv"  # Add the Macc file path



# Initialize the Dash app
app = dash.Dash(__name__)

# Define a function to read CSV and clean the columns
def read_and_clean_csv(file_path):
    delimiter_pattern = r"[,\t\s]+"  # Regex for comma, tab, or any whitespace
    df = pd.read_csv(file_path, delimiter=delimiter_pattern, engine="python")
    df.columns = df.columns.str.strip()  # Remove any leading/trailing spaces from column names
    return df

# Function to create the flat arrow coordinates with rotation
def create_flat_arrows(roll, pitch, yaw):
    # Create rotation matrices
    def rotation_matrix_x(angle):
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
    
    def rotation_matrix_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
    
    def rotation_matrix_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])
    
    # Create two flat arrows - top arrow and bottom arrow
    
    # Top arrow (red) - This will be a thin flat arrow pointing forward
    # Arrow body
    top_x = np.array([-1.7, 0, 0])
    top_y = np.array([0, 0, 0])
    top_z = np.array([0.001, 0.001, 0.001])  # Slightly above the XY plane
    
    # Arrow wings
    top_wings_x = np.array([0, 0])
    top_wings_y = np.array([-1, 1])
    top_wings_z = np.array([0.001, 0.001])
    
    # Bottom arrow (blue) - This will be a thin flat arrow pointing forward
    # Arrow body
    bottom_x = np.array([-1.7, 0, 0])
    bottom_y = np.array([0, 0, 0])
    bottom_z = np.array([-0.001, -0.001, -0.001])  # Slightly below the XY plane
    
    # Arrow wings
    bottom_wings_x = np.array([0, 0])
    bottom_wings_y = np.array([-1, 1])
    bottom_wings_z = np.array([-0.001, -0.001])
    
    # Create rotation matrix (apply in ZYX order)
    R_z = rotation_matrix_z(yaw)
    R_y = rotation_matrix_y(pitch)
    R_x = rotation_matrix_x(roll)
    R = R_z @ R_y @ R_x
    
    # Apply rotation to all points
    # Top arrow
    top_coords = np.vstack((top_x, top_y, top_z))
    rotated_top = R @ top_coords
    
    top_wings_coords = np.vstack((top_wings_x, top_wings_y, top_wings_z))
    rotated_top_wings = R @ top_wings_coords
    
    # Bottom arrow
    bottom_coords = np.vstack((bottom_x, bottom_y, bottom_z))
    rotated_bottom = R @ bottom_coords
    
    bottom_wings_coords = np.vstack((bottom_wings_x, bottom_wings_y, bottom_wings_z))
    rotated_bottom_wings = R @ bottom_wings_coords
    
    return (
        rotated_top[0], rotated_top[1], rotated_top[2],
        rotated_top_wings[0], rotated_top_wings[1], rotated_top_wings[2],
        rotated_bottom[0], rotated_bottom[1], rotated_bottom[2],
        rotated_bottom_wings[0], rotated_bottom_wings[1], rotated_bottom_wings[2]
    )

# Extract filenames (without path) for trajectory names
trajectory1_name = "3D_data"
# trajectory1_name = os.path.basename(file1).split('.')[0]
trajectory2_name = os.path.basename(file2).split('.')[0]

# Layout for Dash app
app.layout = html.Div([  
    html.Div([
        # Dropdown, buttons for interval control, pause, and reset
        html.Div([
            dcc.Dropdown(
                id='interval-dropdown',
                options=[
                    {'label': 'Interval 50 ms', 'value': 50},
                    {'label': 'Interval 100 ms', 'value': 100},
                    {'label': 'Interval 500 ms', 'value': 500},
                    {'label': 'Interval 1000 ms', 'value': 1000},
                    {'label': 'Interval 2000 ms', 'value': 2000},
                    {'label': 'Interval 5000 ms', 'value': 5000},
                ],
                value=2000,
                clearable=False,
                style={'width': '140px', 'display': 'inline-block','margin-top': '3px'}
            ),
            html.Button("Pause/Continue", id='pause-button', n_clicks=0, style={'margin-left': '5px'}),
            html.Button("Reset", id='reset-button', n_clicks=0, style={'margin-left': '5px'})
        ], style={
            'display': 'flex',  # Flexbox layout to align items horizontally
            'align-items': 'center',  # Vertically align items
            'background-color': 'rgba(0, 0, 30, 0.8)',  # Slight transparency for the background
            'padding': '5px',
            'border-radius': '5px',  # Rounded corners for appearance
            'width': '310px',  # Ensure the background adjusts dynamically to content
            'z-index': 9999  # Ensure it stays on top
        }),

        # Graph container div, with relative positioning to keep control elements on top
        html.Div([
            # Main row with 3D plot and RPY graph side by side
            html.Div([
                # 3D Plot
                html.Div([
                    dcc.Graph(id='real-time-plot', style={
                        'width': '100%', 
                        'height': '500px',
                        'background-color': 'rgba(0, 0, 30, 0.8)',
                        'padding': '5px',
                        'border-radius': '5px',
                    }),
                ], style={'width': '30%', 'display': 'inline-block', 'vertical-align': 'top'}),
                
                # RPY Plot - Locked to the right with 5px margin
                html.Div([
                    html.H3("Current Orientation", style={'textAlign': 'center', 'margin-top': '5px', 'color': 'white'}),
                    dcc.Graph(id='orientation-plot', style={'height': '400px'}),
                    html.Div([
                        html.Div([
                            html.Strong("Roll: ", style={'color': 'white'}),
                            html.Span(id='roll-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Pitch: ", style={'color': 'white'}),
                            html.Span(id='pitch-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Yaw: ", style={'color': 'white'}),
                            html.Span(id='yaw-value', children="0.0", style={'color': 'white'})
                        ])
                    ], style={'display': 'flex', 'justifyContent': 'center', 'marginTop': '10px'})
                ], style={
                    'width': '30%',  # Lock to right with explicit width
                    'display': 'inline-block', 
                    'vertical-align': 'top',
                    'background-color': 'rgba(0, 0, 30, 0.8)',
                    'padding': '5px',
                    'border-radius': '5px',
                    'margin-left': '15px'  # Ensure 5px space from 3D plot
                }),
                
                # Acceleration Plot - Next to the Orientation Plot
                html.Div([
                    html.H3("Current Acceleration", style={'textAlign': 'center', 'margin-top': '5px', 'color': 'white'}),
                    dcc.Graph(id='acceleration-plot', style={'height': '400px'}),
                    html.Div([
                        html.Div([
                            html.Strong("Acc X: ", style={'color': 'white'}),
                            html.Span(id='acc-x-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Acc Y: ", style={'color': 'white'}),
                            html.Span(id='acc-y-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Acc Z: ", style={'color': 'white'}),
                            html.Span(id='acc-z-value', children="0.0", style={'color': 'white'})
                        ])
                    ], style={'display': 'flex', 'justifyContent': 'center', 'marginTop': '10px'})
                ], style={
                    'width': '20%',
                    'display': 'inline-block', 
                    'vertical-align': 'top',
                    'background-color': 'rgba(0, 0, 30, 0.8)',
                    'padding': '5px',
                    'border-radius': '5px',
                    'margin-left': '5px'
                }),
                
                # Magnetometer Acceleration Plot
                html.Div([
                    html.H3("Magnetometer Acceleration", style={'textAlign': 'center', 'margin-top': '5px', 'color': 'white'}),
                    dcc.Graph(id='macc-plot', style={'height': '400px'}),
                    html.Div([
                        html.Div([
                            html.Strong("Acc X: ", style={'color': 'white'}),
                            html.Span(id='macc-x-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Acc Y: ", style={'color': 'white'}),
                            html.Span(id='macc-y-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Acc Z: ", style={'color': 'white'}),
                            html.Span(id='macc-z-value', children="0.0", style={'color': 'white'})
                        ], style={'marginRight': '20px'}),
                        html.Div([
                            html.Strong("Field: ", style={'color': 'white'}),
                            html.Span(id='macc-field-value', children="0", style={'color': 'white'})
                        ])
                    ], style={'display': 'flex', 'justifyContent': 'center', 'marginTop': '10px'})
                ], style={
                    'width': '20%',
                    'display': 'inline-block', 
                    'vertical-align': 'top',
                    'background-color': 'rgba(0, 0, 30, 0.8)',
                    'padding': '5px',
                    'border-radius': '5px',
                    'margin-left': '5px'
                }),
                
                # Mouse Movement Plot - Next to the Acceleration Plot
                html.Div([
                    html.H3("Current Mouse Movement", style={'textAlign': 'center', 'margin-top': '5px', 'color': 'white'}),
                    html.Div([
                        html.Div([
                            html.Div([
                                html.Strong("Speed: ", style={'color': 'white'}),
                                html.Span(id='mouse-speed-value', children="0.0", style={'color': 'white'})
                            ], style={'marginBottom': '30px', 'textAlign': 'center'}),
                            html.Div([
                                html.Strong("Direction: ", style={'color': 'white'}),
                                html.Span(id='mouse-direction-value', children="0.0", style={'color': 'white'})
                            ], style={'marginBottom': '30px', 'textAlign': 'center'}),
                            html.Div([
                                html.Strong("Distance: ", style={'color': 'white'}),
                                html.Span(id='mouse-distance-value', children="0.0", style={'color': 'white'})
                            ], style={'textAlign': 'center'})
                        ], style={'display': 'flex', 'flexDirection': 'column', 'justifyContent': 'center', 'height': '400px'})
                    ], style={'padding': '10px'})
                ], style={
                    'width': '20%',
                    'display': 'inline-block', 
                    'vertical-align': 'top',
                    'background-color': 'rgba(0, 0, 30, 0.8)',
                    'padding': '5px',
                    'border-radius': '5px',
                    'margin-left': '5px'
                }),
            ], style={'display': 'flex', 'justify-start': 'space-between', 'margin-top': '10px'}),
            
             # Axes plots below
            dcc.Graph(id='x-plot'),

            # New orientation data plot
            html.H3("Orientation Data over Time", style={'textAlign': 'center', 'margin-top': '20px'}),
            dcc.Graph(id='orientation-data-plot'),

            html.H3("Orientation Acceleration over Time", style={'textAlign': 'center', 'margin-top': '20px'}),
            dcc.Graph(id='orientation-acceleration-plot'),
            
            # New acceleration data plot
            html.H3("Acceleration Data over Time", style={'textAlign': 'center', 'margin-top': '20px'}),
            dcc.Graph(id='acceleration-data-plot'),
            
            # New magnetometer acceleration data plot
            html.H3("Magnetometer Acceleration Data over Time", style={'textAlign': 'center', 'margin-top': '20px'}),
            dcc.Graph(id='macc-data-plot'),
            
            # New mouse movement data plot
            html.H3("Mouse Movement Data over Time", style={'textAlign': 'center', 'margin-top': '20px'}),
            dcc.Graph(id='mouse-data-plot'),

        ], style={'position': 'relative'}),  # Adjusted height

        # Interval component for automatic updates
        dcc.Interval(
            id='interval-update',
            interval=1000,  # Default interval
            n_intervals=0
        )
    ], style={'position': 'relative'})  # Adjusted height
])

# Callback to update the interval dynamically
@app.callback(
    Output('interval-update', 'interval'),
    Input('interval-dropdown', 'value')
)
def update_interval(selected_interval):
    return selected_interval

# Callback to update mouse movement values
@app.callback(
    [Output('mouse-speed-value', 'children'),
     Output('mouse-direction-value', 'children'),
     Output('mouse-distance-value', 'children')],
    [Input('interval-update', 'n_intervals')],
    [State('pause-button', 'n_clicks')]
)
def update_mouse_movement(n_intervals, pause_clicks):
    if pause_clicks % 2 == 1:
        return dash.no_update, dash.no_update, dash.no_update
    
    try:
        # Read the mouse data with explicit delimiter
        mouse_df = pd.read_csv(mouse_file, delimiter=',')
        
        if mouse_df.empty or not all(col in mouse_df.columns for col in ['Speed', 'Direction', 'Covered Distance']):
            # If the expected columns aren't found, try different column names
            if all(col in mouse_df.columns for col in ['speed', 'direction', 'covered_distance']):
                # Get the latest mouse values (last row)
                latest_mouse = mouse_df.iloc[-1]
                speed = float(latest_mouse['speed'])
                direction = float(latest_mouse['direction'])
                distance = float(latest_mouse['covered_distance'])
            else:
                # Return default values if data is not available
                return "N/A", "N/A", "N/A"
        else:
            # Get the latest mouse values (last row)
            latest_mouse = mouse_df.iloc[-1]
            speed = float(latest_mouse['Speed'])
            direction = float(latest_mouse['Direction'])
            distance = float(latest_mouse['Covered Distance'])
        
        speed_text = f"{speed:.4f} m/s"
        direction_text = f"{direction:.2f}°"
        distance_text = f"{distance:.4f} m"
    
    except Exception as e:
        print(f"Error reading mouse data: {e}")
        # Try reading the file with different parameters if first attempt fails
        try:
            mouse_df = pd.read_csv(mouse_file, delimiter=',', skipinitialspace=True)
            latest_mouse = mouse_df.iloc[-1]
            speed = float(latest_mouse['Speed'])
            direction = float(latest_mouse['Direction'])
            distance = float(latest_mouse['Covered Distance'])
            speed_text = f"{speed:.4f} m/s"
            direction_text = f"{direction:.2f}°"
            distance_text = f"{distance:.4f} m"
        except:
            speed_text, direction_text, distance_text = "Error", "Error", "Error"
    
    return speed_text, direction_text, distance_text

# Callback to update the orientation visualization
@app.callback(
    [Output('orientation-plot', 'figure'),
     Output('roll-value', 'children'),
     Output('pitch-value', 'children'),
     Output('yaw-value', 'children')],
    [Input('interval-update', 'n_intervals')],
    [State('pause-button', 'n_clicks')]
)
def update_orientation(n_intervals, pause_clicks):
    if pause_clicks % 2 == 1:
        return dash.no_update, dash.no_update, dash.no_update, dash.no_update
    
    try:
        # Read the RPY data
        # rpy_df = read_and_clean_csv(rpy_file)
        
        # if rpy_df.empty or not all(col in rpy_df.columns for col in ['Roll', 'Pitch', 'Yaw']):
        #     # Create default visualization with zero angles
        #     roll, pitch, yaw = 0, 0, 0
        #     roll_text, pitch_text, yaw_text = "N/A", "N/A", "N/A"
        # else:
        
        if not len(data_ros):
            roll, pitch, yaw = 0, 0, 0
            roll_text, pitch_text, yaw_text = "N/A", "N/A", "N/A"
        else:
            # Get the latest RPY values (last row)
            latest_rpy = data_ros[-1]
            roll = float(latest_rpy[data_roll_idx])
            pitch = float(latest_rpy[data_pitch_idx])
            yaw = float(latest_rpy[data_yaw_idx])
            roll_text = f"{roll:.4f} rad ({np.degrees(roll):.1f}°)"
            pitch_text = f"{pitch:.4f} rad ({np.degrees(pitch):.1f}°)"
            yaw_text = f"{yaw:.4f} rad ({np.degrees(yaw):.1f}°)"
    
    except Exception as e:
        print(f"Error reading RPY data: {e}")
        roll, pitch, yaw = 0, 0, 0
        roll_text, pitch_text, yaw_text = "Error", "Error", "Error"
    
    # Create the two flat arrows representation
    (top_x, top_y, top_z, 
     top_wings_x, top_wings_y, top_wings_z,
     bottom_x, bottom_y, bottom_z,
     bottom_wings_x, bottom_wings_y, bottom_wings_z) = create_flat_arrows(roll, pitch, yaw)
    
    # Create 3D figure with flat arrows
    fig = go.Figure()
    
    # Add the top arrow (red)
    fig.add_trace(go.Scatter3d(
        x=top_x, y=top_y, z=top_z,
        mode='lines',
        line=dict(color='red', width=3),
        name='Top'
    ))
    
    # Add top arrow wings
    fig.add_trace(go.Scatter3d(
        x=top_wings_x, y=top_wings_y, z=top_wings_z,
        mode='lines',
        line=dict(color='red', width=3),
        showlegend=False
    ))
    
    # Add the bottom arrow (blue)
    fig.add_trace(go.Scatter3d(
        x=bottom_x, y=bottom_y, z=bottom_z,
        mode='lines',
        line=dict(color='blue', width=3),
        name='Bottom'
    ))
    
    # Add bottom arrow wings
    fig.add_trace(go.Scatter3d(
        x=bottom_wings_x, y=bottom_wings_y, z=bottom_wings_z,
        mode='lines',
        line=dict(color='blue', width=3),
        showlegend=False
    ))
    
    # Add a sphere at the origin to represent the center point
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(size=4, color='yellow'),
        name='Origin/Back'
    ))
    
    # Add a red dot to the left wing tip
    fig.add_trace(go.Scatter3d(
        x=[top_wings_x[0]], y=[top_wings_y[0]], z=[top_wings_z[0]],
        mode='markers',
        marker=dict(size=3, color='red'),
        name='Left Wing Tip'
    ))
    
    # Add a green dot to the right wing tip
    fig.add_trace(go.Scatter3d(
        x=[top_wings_x[1]], y=[top_wings_y[1]], z=[top_wings_z[1]],
        mode='markers',
        marker=dict(size=3, color='green'),
        name='Right Wing Tip'
    ))
    
    # Add a blue dot to the arrow head (the front point of the arrow)
    fig.add_trace(go.Scatter3d(
        x=[top_x[0]], y=[top_y[0]], z=[top_z[0]],  # Arrow head position (end of arrow)
        mode='markers',
        marker=dict(size=5, color='purple'),
        name='Front'
    ))
    
    # Set up the layout for better visualization
    fig.update_layout(
        scene=dict(
            xaxis=dict(visible=False,range=[-2, 2], title='X'),
            yaxis=dict(visible=False,range=[-2, 2], title='Y'),
            zaxis=dict(range=[-2, 2], title='Z'),
            aspectmode='cube',
            camera=dict(
                eye=dict(x=1.1, y=0., z=0)
            )
        ),
        margin=dict(l=0, r=0, b=0, t=30),
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01
        ),
        paper_bgcolor='rgba(0, 0, 0, 0.8)',  # Match the blue box background
        plot_bgcolor='rgba(0, 0, 30, 0.8)',   # Match the blue box background
        font=dict(color='white')  # White text for better visibility
    )
    
    return fig, roll_text, pitch_text, yaw_text

# Callback to update the acceleration visualization
@app.callback(
    [Output('acceleration-plot', 'figure'),
     Output('acc-x-value', 'children'),
     Output('acc-y-value', 'children'),
     Output('acc-z-value', 'children')],
    [Input('interval-update', 'n_intervals')],
    [State('pause-button', 'n_clicks')]
)
def update_acceleration(n_intervals, pause_clicks):
    if pause_clicks % 2 == 1:
        return dash.no_update, dash.no_update, dash.no_update, dash.no_update
    
    try:
        # Read the acceleration data
        acc_df = read_and_clean_csv(acc_file)
        
        if acc_df.empty or not all(col in acc_df.columns for col in ['acc_x', 'acc_y', 'acc_z']):
            # Create default visualization with zero acceleration
            acc_x, acc_y, acc_z = 0, 0, 0
            acc_x_text, acc_y_text, acc_z_text = "N/A", "N/A", "N/A"
        else:
            # Get the latest acceleration values (last row)
            latest_acc = acc_df.iloc[-1]
            acc_x = float(latest_acc['acc_x'])
            acc_y = float(latest_acc['acc_y'])
            acc_z = float(latest_acc['acc_z'])
            acc_x_text = f"{acc_x:.4f} m/s²"
            acc_y_text = f"{acc_y:.4f} m/s²"
            acc_z_text = f"{acc_z:.4f} m/s²"
    
    except Exception as e:
        print(f"Error reading acceleration data: {e}")
        acc_x, acc_y, acc_z = 0, 0, 0
        acc_x_text, acc_y_text, acc_z_text = "Error", "Error", "Error"
    
    # Create a 3D figure for acceleration
    fig = go.Figure()
    
    # Scale factor to make the arrow length proportional to acceleration magnitude
    scale_factor = 0.5
    
    # Calculate magnitude of acceleration for normalizing if needed
    acc_magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    if acc_magnitude > 0:
        # Determine color based on whether the vector goes above or below the xy-plane
        # Blue if below xy-plane (z < 0), Red if above (z >= 0)
        vector_color = 'blue' if acc_z < 0 else 'red'
        
        # Draw an arrow from origin to show acceleration direction and magnitude
        fig.add_trace(go.Scatter3d(
            x=[0, acc_x * scale_factor], 
            y=[0, acc_y * scale_factor], 
            z=[0, acc_z * scale_factor],
            mode='lines',
            line=dict(color=vector_color, width=10),
            name='Acceleration Vector'
        ))
    
    # Add coordinate axes
    axis_length = 2.0
    
    # X-axis (red)
    fig.add_trace(go.Scatter3d(
        x=[0, axis_length], y=[0, 0], z=[0, 0],
        mode='lines',
        line=dict(color='red', width=2),
        name='X-axis'
    ))
    
    # Y-axis (green)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, axis_length], z=[0, 0],
        mode='lines',
        line=dict(color='green', width=2),
        name='Y-axis'
    ))
    
    # Z-axis (blue)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, 0], z=[0, axis_length],
        mode='lines',
        line=dict(color='blue', width=2),
        name='Z-axis'
    ))
    
    # Add a visual representation of the XY plane (semi-transparent)
    x_plane = np.linspace(-axis_length, axis_length, 10)
    y_plane = np.linspace(-axis_length, axis_length, 10)
    x_plane_grid, y_plane_grid = np.meshgrid(x_plane, y_plane)
    z_plane_grid = np.zeros_like(x_plane_grid)
    
    fig.add_trace(go.Surface(
        x=x_plane_grid,
        y=y_plane_grid,
        z=z_plane_grid,
        colorscale=[[0, 'rgba(200, 200, 200, 0.1)'], [1, 'rgba(200, 200, 200, 0.1)']],
        showscale=False,
        name='XY Plane'
    ))
    
    # Add a sphere at the origin
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(size=4, color='yellow'),
        name='Origin'
    ))
    
    # Set up the layout for better visualization
    fig.update_layout(
        scene=dict(
            xaxis=dict(visible=False,range=[-axis_length, axis_length], title='X'),
            yaxis=dict(visible=False,range=[-axis_length, axis_length], title='Y'),
            zaxis=dict(visible=False,range=[-axis_length, axis_length], title='Z'),
            aspectmode='cube',
            camera=dict(
                eye=dict(x=1.2, y=1.2, z=1.2)
            )
        ),
        margin=dict(l=0, r=0, b=0, t=30),
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01
        ),
        paper_bgcolor='rgba(0, 0, 0, 0.8)',  # Match the blue box background
        plot_bgcolor='rgba(0, 0, 30, 0.8)',   # Match the blue box background
        font=dict(color='white')  # White text for better visibility
    )
    
    return fig, acc_x_text, acc_y_text, acc_z_text

# New callback to update magnetometer acceleration visualization
@app.callback(
    [Output('macc-plot', 'figure'),
     Output('macc-x-value', 'children'),
     Output('macc-y-value', 'children'),
     Output('macc-z-value', 'children'),
     Output('macc-field-value', 'children')],
    [Input('interval-update', 'n_intervals')],
    [State('pause-button', 'n_clicks')]
)
def update_macc(n_intervals, pause_clicks):
    if pause_clicks % 2 == 1:
        return dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update
    
    try:
        # Read the Macc data
        macc_df = read_and_clean_csv(macc_file)
        
        if macc_df.empty or not all(col in macc_df.columns for col in ['Macc_x', 'Macc_y', 'Macc_z', 'field']):
            # Create default visualization with zero values
            macc_x, macc_y, macc_z, field = 0, 0, 0, 0
            macc_x_text, macc_y_text, macc_z_text, field_text = "N/A", "N/A", "N/A", "N/A"
        else:
            # Get the latest Macc values (last row)
            latest_macc = macc_df.iloc[-1]
            macc_x = float(latest_macc['Macc_x'])
            macc_y = float(latest_macc['Macc_y'])
            macc_z = float(latest_macc['Macc_z'])
            field = int(latest_macc['field'])
            macc_x_text = f"{macc_x:.4f} m/s²"
            macc_y_text = f"{macc_y:.4f} m/s²"
            macc_z_text = f"{macc_z:.4f} m/s²"
            field_text = f"{field}"
    
    except Exception as e:
        print(f"Error reading Macc data: {e}")
        macc_x, macc_y, macc_z, field = 0, 0, 0, 0
        macc_x_text, macc_y_text, macc_z_text, field_text = "Error", "Error", "Error", "Error"
    
    # Create a 3D figure for Macc
    fig = go.Figure()
    
    # Scale factor to make the arrow length proportional to Macc magnitude
    scale_factor = 0.5
    
    # Calculate magnitude of Macc for vector width
    macc_magnitude = np.sqrt(macc_x**2 + macc_y**2 + macc_z**2)
    
    # Determine color based on magnetic field strength
    if macc_magnitude > 0:
        vector_color = 'blue' if macc_z < 0 else 'red'
        vector_width = field  # Scale width with magnitude
        
        # Draw an arrow from origin to show Macc direction and magnitude
        fig.add_trace(go.Scatter3d(
            x=[0, macc_x * scale_factor], 
            y=[0, macc_y * scale_factor], 
            z=[0, macc_z * scale_factor],
            mode='lines',
            line=dict(color=vector_color, width=vector_width),
            name='Acceleration Vector'
        ))
    
    # Add coordinate axes
    axis_length = 2.0
    
    # X-axis (red)
    fig.add_trace(go.Scatter3d(
        x=[0, axis_length], y=[0, 0], z=[0, 0],
        mode='lines',
        line=dict(color='red', width=2),
        name='X-axis'
    ))
    
    # Y-axis (green)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, axis_length], z=[0, 0],
        mode='lines',
        line=dict(color='green', width=2),
        name='Y-axis'
    ))
    
    # Z-axis (blue)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, 0], z=[0, axis_length],
        mode='lines',
        line=dict(color='blue', width=2),
        name='Z-axis'
    ))
    
    # Add a visual representation of the XY plane (semi-transparent)
    x_plane = np.linspace(-axis_length, axis_length, 10)
    y_plane = np.linspace(-axis_length, axis_length, 10)
    x_plane_grid, y_plane_grid = np.meshgrid(x_plane, y_plane)
    z_plane_grid = np.zeros_like(x_plane_grid)
    
    fig.add_trace(go.Surface(
        x=x_plane_grid,
        y=y_plane_grid,
        z=z_plane_grid,
        colorscale=[[0, 'rgba(200, 200, 200, 0.1)'], [1, 'rgba(200, 200, 200, 0.1)']],
        showscale=False,
        name='XY Plane'
    ))
    
    # Add a sphere at the origin
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(size=4, color='yellow'),
        name='Origin'
    ))
    
    # Set up the layout for better visualization
    fig.update_layout(
        scene=dict(
            xaxis=dict(visible=False, range=[-axis_length, axis_length], title='X'),
            yaxis=dict(visible=False, range=[-axis_length, axis_length], title='Y'),
            zaxis=dict(visible=False, range=[-axis_length, axis_length], title='Z'),
            aspectmode='cube',
            camera=dict(
                eye=dict(x=1.2, y=1.2, z=1.2)
            )
        ),
        margin=dict(l=0, r=0, b=0, t=30),
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01
        ),
        paper_bgcolor='rgba(0, 0, 0, 0.8)',  # Match the blue box background
        plot_bgcolor='rgba(0, 0, 30, 0.8)',   # Match the blue box background
        font=dict(color='white')  # White text for better visibility
    )
    
    return fig, macc_x_text, macc_y_text, macc_z_text, field_text

# Callback to update the trajectory plots
@app.callback(
    [Output('real-time-plot', 'figure'),
     Output('x-plot', 'figure')],  # Changed from three outputs to just one combined plot
    [Input('interval-update', 'n_intervals'),
     Input('reset-button', 'n_clicks')],
    [State('pause-button', 'n_clicks')]
)
def update_plots(n_intervals, reset_clicks, pause_clicks):
    if pause_clicks % 2 == 1:
        return dash.no_update, dash.no_update
    
    # df1 = read_and_clean_csv(file1)
    df2 = read_and_clean_csv(file2)

    # if not {'x', 'y', 'z'}.issubset(df1.columns) or not {'x', 'y', 'z'}.issubset(df2.columns):
    #     return dash.no_update, dash.no_update
    
    arr_x = [a[data_x_idx] for a in data_ros]
    arr_y = [a[data_y_idx] for a in data_ros]
    arr_z = [a[data_z_idx] for a in data_ros]
    real_arr_x = df2['x']
    real_arr_y = df2['y']
    real_arr_z = df2['z']

    # 3D plot remains the same
    new_fig = go.Figure()
    new_fig.add_trace(go.Scatter3d(
        x=arr_x, y=arr_y, z=arr_z,
        mode='lines+markers',
        marker=dict(size=3, color='blue'),
        line=dict(width=1, color='blue'),
        name=trajectory1_name
    ))
    new_fig.add_trace(go.Scatter3d(
        x=real_arr_x, y=real_arr_y, z=real_arr_z,
        mode='lines+markers',
        marker=dict(size=3, color='green'),
        line=dict(width=1, color='green'),
        name=trajectory2_name
    ))
    new_fig.update_layout(
        scene=dict(
            xaxis_title="X Axis",
            yaxis_title="Y Axis",
            zaxis_title="Z Axis",
            aspectmode="cube",
            camera=dict(eye=dict(x=1.9, y=0.7, z=0.1))
        ),
        margin=dict(l=0, r=0, b=0, t=0),
        height=500,
        legend=dict(
            orientation="v",  # Vertical legend
            yanchor="bottom",
            y=0,  # Position at bottom
            xanchor="right",
            x=1,
            bgcolor='rgba(200, 200, 255, 0.5)'  # Semi-transparent background
        ),
        paper_bgcolor='rgba(0, 0, 0, 0.8)',  # Blue box background
        plot_bgcolor='rgba(0, 255, 0, 0.8)',    # Blue box background
        font=dict(color='white')  # White text for better visibility
    )
    
    # Combined XYZ plot
    combined_fig = go.Figure()
    
    # Add X data
    combined_fig.add_trace(go.Scatter(
        x=list(range(len(arr_x))), y=arr_x,
        mode='lines+markers', name=f"{trajectory1_name} X",
        line=dict(color='red')
    ))
    combined_fig.add_trace(go.Scatter(
        x=list(range(len(real_arr_x))), y=real_arr_x,
        mode='lines+markers', name=f"{trajectory2_name} X",
        line=dict(color='darkred')
    ))
    
    # Add Y data
    combined_fig.add_trace(go.Scatter(
        x=list(range(len(arr_y))), y=arr_y,
        mode='lines+markers', name=f"{trajectory1_name} Y",
        line=dict(color='green')
    ))
    combined_fig.add_trace(go.Scatter(
        x=list(range(len(real_arr_y))), y=real_arr_y,
        mode='lines+markers', name=f"{trajectory2_name} Y",
        line=dict(color='darkgreen')
    ))
    
    # Add Z data
    combined_fig.add_trace(go.Scatter(
        x=list(range(len(arr_z))), y=arr_z,
        mode='lines+markers', name=f"{trajectory1_name} Z",
        line=dict(color='blue')
    ))
    combined_fig.add_trace(go.Scatter(
        x=list(range(len(real_arr_z))), y=real_arr_z,
        mode='lines+markers', name=f"{trajectory2_name} Z",
        line=dict(color='darkblue')
    ))
    
    combined_fig.update_layout(
        title="Combined X, Y, and Z Data over Time",
        xaxis_title="Time",
        yaxis_title="Value",
        legend=dict(
            orientation="h",  # Horizontal legend
            yanchor="bottom",
            y=1.02,  # Position above plot
            xanchor="right",
            x=1
        )
    )
    
    return new_fig, combined_fig

@app.callback(
    [Output('orientation-data-plot', 'figure'),
     Output('acceleration-data-plot', 'figure'),
     Output('macc-data-plot', 'figure'),
     Output('mouse-data-plot', 'figure'),
     Output('orientation-acceleration-plot', 'figure')],
    [Input('interval-update', 'n_intervals'),
     Input('reset-button', 'n_clicks')],
    [State('pause-button', 'n_clicks')]
)
def update_data_plots(n_intervals, reset_clicks, pause_clicks):
    if pause_clicks % 2 == 1:
        return dash.no_update, dash.no_update, dash.no_update, dash.no_update, dash.no_update
    
    # Process orientation data
    try:
        # rpy_df = read_and_clean_csv(rpy_file)
        orientation_fig = go.Figure()
        
        # if not rpy_df.empty and all(col in rpy_df.columns for col in ['Roll', 'Pitch', 'Yaw']):
            # Add Roll data

        arr_roll = [a[data_roll_idx] for a in data_ros]
        arr_pitch = [a[data_pitch_idx] for a in data_ros]
        arr_yaw = [a[data_yaw_idx] for a in data_ros]
        
        orientation_fig.add_trace(go.Scatter(
            x=list(range(len(arr_roll))), 
            y=arr_roll,
            mode='lines+markers', 
            name='Roll',
            line=dict(color='red')
        ))
        
        # Add Pitch data
        orientation_fig.add_trace(go.Scatter(
            x=list(range(len(arr_pitch))), 
            y=arr_pitch,
            mode='lines+markers', 
            name='Pitch',
            line=dict(color='green')
        ))
        
        # Add Yaw data
        orientation_fig.add_trace(go.Scatter(
            x=list(range(len(arr_yaw))), 
            y=arr_yaw,
            mode='lines+markers', 
            name='Yaw',
            line=dict(color='blue')
        ))
    except Exception as e:
        print(f"Error processing orientation data: {e}")
        orientation_fig = go.Figure()
    
    orientation_fig.update_layout(
        title="Combined Roll, Pitch, and Yaw Data over Time",
        xaxis_title="Time",
        yaxis_title="Value (radians)",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )
    
    # Process acceleration data
    try:
        acc_df = read_and_clean_csv(acc_file)
        acceleration_fig = go.Figure()
        
        if not acc_df.empty and all(col in acc_df.columns for col in ['acc_x', 'acc_y', 'acc_z']):
            # Add X acceleration
            acceleration_fig.add_trace(go.Scatter(
                x=list(range(len(acc_df['acc_x']))), 
                y=acc_df['acc_x'],
                mode='lines+markers', 
                name='Acc X',
                line=dict(color='red')
            ))
            
            # Add Y acceleration
            acceleration_fig.add_trace(go.Scatter(
                x=list(range(len(acc_df['acc_y']))), 
                y=acc_df['acc_y'],
                mode='lines+markers', 
                name='Acc Y',
                line=dict(color='green')
            ))
            
            # Add Z acceleration
            acceleration_fig.add_trace(go.Scatter(
                x=list(range(len(acc_df['acc_z']))), 
                y=acc_df['acc_z'],
                mode='lines+markers', 
                name='Acc Z',
                line=dict(color='blue')
            ))
    except Exception as e:
        print(f"Error processing acceleration data: {e}")
        acceleration_fig = go.Figure()
    
    acceleration_fig.update_layout(
        title="Combined X, Y, and Z Acceleration Data over Time",
        xaxis_title="Time",
        yaxis_title="Acceleration (m/s²)",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )
    
        # Process Macc data
    try:
        macc_df = read_and_clean_csv(macc_file)
        macc_fig = go.Figure()
        
        if not macc_df.empty and all(col in macc_df.columns for col in ['Macc_x', 'Macc_y', 'Macc_z', 'field']):
            # Add X Macc data
            macc_fig.add_trace(go.Scatter(
                x=list(range(len(macc_df['Macc_x']))), 
                y=macc_df['Macc_x'],
                mode='lines+markers', 
                name='Macc X',
                line=dict(color='red')
            ))
            
            # Add Y Macc data
            macc_fig.add_trace(go.Scatter(
                x=list(range(len(macc_df['Macc_y']))), 
                y=macc_df['Macc_y'],
                mode='lines+markers', 
                name='Macc Y',
                line=dict(color='green')
            ))
            
            # Add Z Macc data
            macc_fig.add_trace(go.Scatter(
                x=list(range(len(macc_df['Macc_z']))), 
                y=macc_df['Macc_z'],
                mode='lines+markers', 
                name='Macc Z',
                line=dict(color='blue')
            ))

            macc_fig.add_trace(go.Bar(
                x=list(range(len(macc_df['field']))), 
                y=macc_df['field'],
                width=1,
                name='Field',
                marker=dict(color='purple'),
                yaxis='y2',
                opacity=0.3,
                
            ))
            
           
    except Exception as e:
        print(f"Error processing Macc data: {e}")
        macc_fig = go.Figure()
    
    macc_fig.update_layout(
        
        yaxis2=dict(
        title="Field Strenght",
        overlaying='y',  # Place it on the same plot
        side='right',
          showgrid=False  # Position it on the right side
        ),
        yaxis=dict(
        title="Magnetometer Acceleration Data over Time"),
        xaxis_title="Time",
        yaxis_title="Acceleration (m/s²)",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )

    # Process mouse movement data
    try:
        mouse_df = pd.read_csv(mouse_file, delimiter=',')
        mouse_fig = go.Figure()
        
        # Check for expected columns with both uppercase and lowercase naming conventions
        if not mouse_df.empty:
            # Handle the case with uppercase column names
            if all(col in mouse_df.columns for col in ['Speed', 'Direction', 'Covered Distance']):
                # Add Distance data
                mouse_fig.add_trace(go.Bar(
                    x=list(range(len(mouse_df['Covered Distance']))), 
                    y=mouse_df['Covered Distance'],
                    
                    name='Distance',
                    marker=dict(color='blue'),
                    yaxis='y2',
                    opacity=0.1,
                ))

                # Add Speed data
                mouse_fig.add_trace(go.Scatter(
                    x=list(range(len(mouse_df['Speed']))), 
                    y=mouse_df['Speed'],
                    mode='lines', 
                    name='Speed',
                    yaxis='y2',
                    line=dict(color='red', width=1)
                ))
                
                # Add Direction data
                mouse_fig.add_trace(go.Scatter(
                    x=list(range(len(mouse_df['Direction']))), 
                    y=mouse_df['Direction'],
                    mode='lines', 
                    name='Direction',
                    line=dict(color='green', width=1),
                ))
                
                
            
    except Exception as e:
        print(f"Error processing mouse data: {e}")
        mouse_fig = go.Figure()
    
    mouse_fig.update_layout(
        yaxis2=dict(
        title="Distance (m) & Speed (m/s)",
        overlaying='y',  # Place it on the same plot
        side='right',
            # Position it on the right side
        ),
        yaxis=dict(title="Mouse Movement Data over Time",showgrid=False),
        xaxis_title="Time",
        yaxis_title="Direction (deg)",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )

    try:
        # rpy_df = read_and_clean_csv(rpy_file)
        orientation_acc_fig = go.Figure()
        
        # if not rpy_df.empty and all(col in rpy_df.columns for col in ['Roll', 'Pitch', 'Yaw']):
            # Calculate acceleration (derivative) of orientation
            # First, make a copy to avoid modifying the original dataframe
        # rpy_acc_df = rpy_df[['Roll', 'Pitch', 'Yaw']].copy()


        
        # Calculate the difference between consecutive values (acceleration)
        # rpy_acc_df['Roll_acc'] = rpy_acc_df['Roll'].diff()
        # rpy_acc_df['Pitch_acc'] = rpy_acc_df['Pitch'].diff()
        # rpy_acc_df['Yaw_acc'] = rpy_acc_df['Yaw'].diff()
        
        arr_acc_roll = [a[data_acc_roll_idx] for a in data_ros]
        arr_acc_pitch = [a[data_acc_pitch_idx] for a in data_ros]
        arr_acc_yaw = [a[data_acc_yaw_idx] for a in data_ros]

        # Drop the first row which has NaN values after differentiation
        # rpy_acc_df = rpy_acc_df.iloc[1:].reset_index(drop=True)
        
        # Add Roll acceleration data
        orientation_acc_fig.add_trace(go.Scatter(
            x=list(range(len(arr_acc_roll))), 
            y=arr_acc_roll,
            mode='lines+markers', 
            name='Roll Acceleration',
            line=dict(color='red')
        ))
        
        # Add Pitch acceleration data
        orientation_acc_fig.add_trace(go.Scatter(
            x=list(range(len(arr_acc_pitch))), 
            y=arr_acc_pitch,
            mode='lines+markers', 
            name='Pitch Acceleration',
            line=dict(color='green')
        ))
        
        # Add Yaw acceleration data
        orientation_acc_fig.add_trace(go.Scatter(
            x=list(range(len(arr_acc_yaw))), 
            y=arr_acc_yaw,
            mode='lines+markers', 
            name='Yaw Acceleration',
            line=dict(color='blue')
        ))
    except Exception as e:
        print(f"Error processing orientation acceleration data: {e}")
        orientation_acc_fig = go.Figure()
    
    orientation_acc_fig.update_layout(
        title="Roll, Pitch, and Yaw Acceleration over Time",
        xaxis_title="Time",
        yaxis_title="Angular Acceleration (rad/s²)",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )   
    
    return orientation_fig, acceleration_fig, macc_fig, mouse_fig, orientation_acc_fig

def main():
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    app.run(host="0.0.0.0", port=8000, debug=True)

if __name__ == '__main__':
    main()
