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
from server_pkg.server_db import init_db, save_recording, list_recordings, get_recording, delete_recording, compare_recordings

# Global variables for data management
data_ros = []
data_ros_lock = threading.Lock()

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
            # Store the latest data
            data_ros.append((
                msg.x, msg.y, msg.z, msg.yaw, msg.pitch, msg.roll, 
                msg.acc_x, msg.acc_y, msg.acc_z, msg.acc_yaw, msg.acc_pitch, msg.acc_roll, 
                msg.mag_x, msg.mag_y, msg.mag_z, msg.mag_strength, 
                msg.mouse_movement, msg.mouse_speed, msg.mouse_direction, msg.mouse_distance
            ))
            
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
    response = {"success": False, "message": "Command not implemented"}
    
    if message.type == "set_interval":
        # Set the interval for data streaming (handled by client-side)
        response = {"success": True, "message": "Interval updated"}
    
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
    await api.connection_manager.connect(websocket)
    update_interval = 0.1  # Default 10 Hz
    
    try:
        # Handle initial connection messages
        await websocket.send_text(json.dumps({"type": "connected"}))
        
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
            except asyncio.TimeoutError:
                pass
            
            # Send the latest data to the client
            latest_data = get_latest_data()
            if latest_data:
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
        
        return {
            "x": latest[0],
            "y": latest[1],
            "z": latest[2],
            "yaw": latest[3],
            "pitch": latest[4],
            "roll": latest[5],
            "acc_x": latest[6],
            "acc_y": latest[7],
            "acc_z": latest[8],
            "acc_yaw": latest[9],
            "acc_pitch": latest[10],
            "acc_roll": latest[11],
            "mag_x": latest[12],
            "mag_y": latest[13],
            "mag_z": latest[14],
            "mag_strength": latest[15],
            "mouse_movement": latest[16],
            "mouse_speed": latest[17],
            "mouse_direction": latest[18],
            "mouse_distance": latest[19]
        }

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
