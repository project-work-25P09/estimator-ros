#!/usr/bin/env python3
import os
import asyncio
import json
import time
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi import Depends
from typing import List, Dict, Any, Optional
import rclpy
from rclpy.node import Node
from estimation.msg import Estimation
import threading
from ament_index_python.packages import get_package_share_directory
import os.path
from datetime import datetime

from server_pkg.server_utils import ConnectionManager
from server_pkg.server_api import APICommand, ServerAPI
from server_pkg.server_db import init_db, save_recording, list_recordings, get_recording, delete_recording, compare_recordings, save_estimation_data, get_estimation_data, get_reference_trajectories

# Global variables for data management
data_ros = []
data_ros_lock = threading.Lock()
is_recording = False
recording_buffer = []
current_recording_id = None

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
        global data_ros, data_ros_lock, is_recording, recording_buffer
        with data_ros_lock:
            # Convert ROS timestamp to seconds (for plotting)
            timestamp_sec = msg.stamp.sec + (msg.stamp.nanosec / 1e9)
            
            # Create data point
            data_point = {
                "timestamp": timestamp_sec,
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
                "yaw": msg.yaw,
                "pitch": msg.pitch,
                "roll": msg.roll,
                "acc_x": msg.acc_x,
                "acc_y": msg.acc_y,
                "acc_z": msg.acc_z,
                "acc_yaw": msg.acc_yaw,
                "acc_pitch": msg.acc_pitch,
                "acc_roll": msg.acc_roll,
                "mag_x": msg.mag_x,
                "mag_y": msg.mag_y,
                "mag_z": msg.mag_z,
                "mag_strength": msg.mag_strength,
                "mouse_movement": msg.mouse_movement,
                "mouse_speed": msg.mouse_speed,
                "mouse_direction": msg.mouse_direction,
                "mouse_distance": msg.mouse_distance
            }
            
            # Store the latest data
            data_ros.append((
                timestamp_sec,
                msg.x, msg.y, msg.z, msg.yaw, msg.pitch, msg.roll, 
                msg.acc_x, msg.acc_y, msg.acc_z, msg.acc_yaw, msg.acc_pitch, msg.acc_roll, 
                msg.mag_x, msg.mag_y, msg.mag_z, msg.mag_strength, 
                msg.mouse_movement, msg.mouse_speed, msg.mouse_direction, msg.mouse_distance
            ))
            
            # Store data in recording buffer if we're recording
            if is_recording:
                recording_buffer.append(data_point)
            
            # Limit the amount of data stored in memory
            if len(data_ros) > 1000:
                data_ros.pop(0)

def ros_spin():
    """ROS node spinning function for a separate thread"""
    rclpy.init(args=None)
    node = EstimationListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Initialize FastAPI app
api = None
app = FastAPI()

# Main route for the dashboard
@app.get("/")
async def get_dashboard(request: Request):
    return api.get_dashboard(request)

# API endpoint for statistics
@app.get("/stats")
async def get_statistics():
    """Return current statistics about the system"""
    with data_ros_lock:
        data_count = len(data_ros)
    
    # Get database stats from our database module
    db_stats = list_recordings()
    recording_count = len(db_stats.get("recordings", [])) if db_stats.get("success", False) else 0
    
    return {
        "live_data_points": data_count,
        "saved_recordings": recording_count,
        "status": "running",
        "timestamp": datetime.now().isoformat()
    }

# API endpoint for commands
@app.post("/api/command")
async def api_command(message: APICommand):
    """Handle API commands from the frontend"""
    global is_recording, recording_buffer, current_recording_id
    response = {"success": False, "message": "Command not implemented"}
    
    if message.type == "set_interval":
        # Set the interval for data streaming (handled by client-side)
        response = {"success": True, "message": "Interval updated"}
    elif message.type == "start_recording":
        # Start recording data
        is_recording = True
        recording_buffer = []
        response = {"success": True, "message": "Recording started", "recording": True}
    elif message.type == "stop_recording":
        # Stop recording and save the recorded data
        is_recording = False
        
        # Create a new recording entry with default name
        rec_name = message.data.get("name", f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        rec_desc = message.data.get("description", "")
        
        # Save recording metadata
        save_result = save_recording(rec_name, {}, rec_desc)
        
        if save_result["success"]:
            current_recording_id = save_result["id"]
            # Save the estimation data to the database
            if recording_buffer:
                save_estimation_data(current_recording_id, recording_buffer)
            
            response = {
                "success": True, 
                "message": f"Recording stopped and saved as '{rec_name}'", 
                "recording": False, 
                "recording_id": current_recording_id
            }
        else:
            response = {"success": False, "message": "Failed to save recording", "error": save_result.get("error", "")}
    
    return response

# API endpoint for saving recordings
@app.post("/api/save_recording")
async def save_recording_handler(data: dict):
    """Save a recording to the database"""
    return save_recording(data.get("name", ""), data.get("data", {}), data.get("description", ""))

# API endpoint for listing recordings
@app.get("/api/list_recordings")
async def list_recordings_handler():
    """List all available recordings"""
    return list_recordings()

# API endpoint for getting a specific recording
@app.get("/api/get_recording/{recording_id}")
async def get_recording_handler(recording_id: int):
    """Get a specific recording by ID"""
    return get_recording(recording_id)

# API endpoint for getting estimation data
@app.get("/api/get_estimation_data/{recording_id}")
async def get_estimation_data_handler(recording_id: int):
    """Get all estimation data for a specific recording"""
    return get_estimation_data(recording_id)

# API endpoint for getting reference trajectories
@app.get("/api/get_reference_trajectories/{recording_id}")
async def get_reference_trajectories_handler(recording_id: int):
    """Get all reference trajectories for a specific recording"""
    return get_reference_trajectories(recording_id)

# API endpoint for saving a reference trajectory
@app.post("/api/save_reference_trajectory/{recording_id}")
async def save_reference_trajectory_handler(recording_id: int, data: dict):
    """Save a reference trajectory for a recording"""
    name = data.get("name", f"reference_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    points = data.get("points", [])
    
    if not points:
        return {"success": False, "error": "No trajectory points provided"}
    
    return save_reference_trajectory(recording_id, name, points)

# API endpoint for deleting a recording
@app.delete("/api/delete_recording/{recording_id}")
async def delete_recording_handler(recording_id: int):
    """Delete a recording by ID"""
    return delete_recording(recording_id)

# API endpoint for comparing recordings
@app.get("/api/compare_recordings/{recording_id1}/{recording_id2}")
async def compare_recordings_handler(recording_id1: int, recording_id2: int):
    """Compare two recordings and return statistics"""
    return compare_recordings(recording_id1, recording_id2)

# WebSocket endpoint for real-time data streaming
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global is_recording, current_recording_id
    await api.connection_manager.connect(websocket)
    update_interval = 0.1  # Default 10 Hz
    
    try:
        # Handle initial connection messages
        await websocket.send_text(json.dumps({
            "type": "connected",
            "recording": is_recording
        }))
        
        while True:
            # Check for incoming messages from the client
            data = None
            try:
                data_str = await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
                data = json.loads(data_str)
                
                # Handle interval change requests
                if data and data.get("type") == "set_interval":
                    new_interval = int(data.get("value", 1000)) / 1000.0  # Convert ms to seconds
                    update_interval = max(0.01, min(5.0, new_interval))  # Limit between 10ms and 5s
                
                # Handle recording commands
                elif data and data.get("type") == "start_recording":
                    is_recording = True
                    recording_buffer.clear()
                    await websocket.send_text(json.dumps({
                        "type": "recording_status",
                        "recording": True
                    }))
                
                elif data and data.get("type") == "stop_recording":
                    is_recording = False
                    # Save the recording (handled separately via API)
                    await websocket.send_text(json.dumps({
                        "type": "recording_status",
                        "recording": False
                    }))
                
                elif data and data.get("type") == "load_recording":
                    recording_id = data.get("recording_id")
                    if recording_id:
                        recording_data = get_recording(recording_id)
                        estimation_data = get_estimation_data(recording_id)
                        reference_data = get_reference_trajectories(recording_id)
                        
                        await websocket.send_text(json.dumps({
                            "type": "loaded_recording",
                            "recording_id": recording_id,
                            "recording": recording_data.get("recording", {}),
                            "estimations": estimation_data.get("estimations", []),
                            "reference_trajectories": reference_data.get("trajectories", {})
                        }))
            except asyncio.TimeoutError:
                pass
            
            # Send the latest data to the client
            latest_data = get_latest_data()
            if latest_data:
                latest_data["recording"] = is_recording
                if is_recording:
                    latest_data["recording_data_points"] = len(recording_buffer)
                await websocket.send_text(json.dumps(latest_data))
            
            # Sleep for the specified interval
            await asyncio.sleep(update_interval)
    
    except WebSocketDisconnect:
        api.connection_manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        api.connection_manager.disconnect(websocket)

def get_latest_data():
    """Get the latest data from the ROS messages"""
    with data_ros_lock:
        if not data_ros:
            return None
        
        latest = data_ros[-1]
        
        # Get the latest hardware monitor data if available
        hw_data = api.dashboard_ros.get_latest_hw_data() if api and api.dashboard_ros else None
        
        data = {
            "timestamp": latest[0],  # ROS timestamp in seconds
            "x": latest[1],
            "y": latest[2],
            "z": latest[3],
            "yaw": latest[4],
            "pitch": latest[5],
            "roll": latest[6],
            "acc_x": latest[7],
            "acc_y": latest[8],
            "acc_z": latest[9],
            "acc_yaw": latest[10],
            "acc_pitch": latest[11],
            "acc_roll": latest[12],
            "mag_x": latest[13],
            "mag_y": latest[14],
            "mag_z": latest[15],
            "mag_strength": latest[16],
            "mouse_movement": latest[17],
            "mouse_speed": latest[18],
            "mouse_direction": latest[19],
            "mouse_distance": latest[20]
        }
        
        # Add hardware monitor data if available
        if hw_data:
            data.update({
                "hw_cpu_usage": hw_data["cpu_usage"],
                "hw_memory_mb": hw_data["memory_mb"],
                "hw_disk_rx_mb": hw_data["disk_rx_mb"],
                "hw_disk_tx_mb": hw_data["disk_tx_mb"],
                "hw_network_rx_mb": hw_data["network_rx_mb"],
                "hw_network_tx_mb": hw_data["network_tx_mb"],
                "hw_power_consumption": hw_data["power_consumption"],
                "hw_temperature": hw_data["temperature"]
            })
        
        return data

def main():
    # Initialize ROS
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # Initialize the database
    init_db()
    
    # Setup FastAPI
    host = "0.0.0.0"
    port = 8000

    pkg_share_fp = get_package_share_directory('server')
    base_fp = os.path.join(pkg_share_fp, 'web')
    static_fp = os.path.join(base_fp, 'static')
    templates_fp = os.path.join(base_fp, 'templates')

    global app, api
    api = ServerAPI(app, templates_fp, static_fp)
    
    # Start the server
    print(f"Starting server on http://{host}:{port}")
    uvicorn.run(app, host=host, port=port)

if __name__ == "__main__":
    main()
