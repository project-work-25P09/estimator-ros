import json
import os
import sqlite3
from datetime import datetime
from typing import Dict, List, Any, Optional

# Database path
DB_PATH = os.path.expanduser("~/estimator-ros/recordings.db")

def init_db():
    """Initialize the SQLite database for storing measurements"""
    # Create the database directory if it doesn't exist
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    
    # Create recordings table
    c.execute('''
    CREATE TABLE IF NOT EXISTS recordings (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        timestamp TEXT NOT NULL,
        description TEXT
    )
    ''')
    
    # Create measurements table for the actual data
    c.execute('''
    CREATE TABLE IF NOT EXISTS measurements (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        recording_id INTEGER,
        data_type TEXT NOT NULL,
        data_json TEXT NOT NULL,
        FOREIGN KEY (recording_id) REFERENCES recordings (id)
    )
    ''')
    
    # Create estimation_data table to store individual estimation messages
    c.execute('''
    CREATE TABLE IF NOT EXISTS estimation_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        recording_id INTEGER,
        timestamp REAL NOT NULL,
        x REAL, y REAL, z REAL,
        yaw REAL, pitch REAL, roll REAL,
        acc_x REAL, acc_y REAL, acc_z REAL,
        acc_yaw REAL, acc_pitch REAL, acc_roll REAL,
        mag_x REAL, mag_y REAL, mag_z REAL, mag_strength REAL,
        mouse_movement REAL, mouse_speed REAL, mouse_direction REAL, mouse_distance REAL,
        FOREIGN KEY (recording_id) REFERENCES recordings (id)
    )
    ''')
    
    # Create reference_trajectory table to store ground truth trajectory data
    c.execute('''
    CREATE TABLE IF NOT EXISTS reference_trajectory (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        recording_id INTEGER,
        name TEXT NOT NULL,
        timestamp REAL NOT NULL,
        x REAL, y REAL, z REAL,
        FOREIGN KEY (recording_id) REFERENCES recordings (id)
    )
    ''')
    
    conn.commit()
    conn.close()

def save_recording(name: str, data: Dict[str, List[Any]], description: str = "") -> Dict[str, Any]:
    """Save a recording to the database"""
    try:
        if not name:
            name = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            
        # Initialize db connection
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        
        # Insert the recording entry
        c.execute(
            "INSERT INTO recordings (name, timestamp, description) VALUES (?, ?, ?)",
            (name, datetime.now().isoformat(), description)
        )
        recording_id = c.lastrowid
        
        # Insert measurement data by type
        for data_type, data_items in data.items():
            if data_items:  # Only insert if we have data
                c.execute(
                    "INSERT INTO measurements (recording_id, data_type, data_json) VALUES (?, ?, ?)",
                    (recording_id, data_type, json.dumps(data_items))
                )
        
        conn.commit()
        conn.close()
        
        return {"success": True, "id": recording_id, "name": name}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def list_recordings() -> Dict[str, Any]:
    """List all available recordings"""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row  # Enable row factory to get dict-like results
        c = conn.cursor()
        
        c.execute("SELECT id, name, timestamp, description FROM recordings ORDER BY timestamp DESC")
        recordings = [dict(row) for row in c.fetchall()]
        
        conn.close()
        
        # Format timestamps for display
        for recording in recordings:
            try:
                timestamp = datetime.fromisoformat(recording["timestamp"])
                recording["date"] = timestamp.strftime("%Y-%m-%d %H:%M:%S")
            except:
                recording["date"] = recording["timestamp"]
        
        return {"success": True, "recordings": recordings}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def get_recording(recording_id: int) -> Dict[str, Any]:
    """Get a specific recording by ID"""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        c = conn.cursor()
        
        # Get recording metadata
        c.execute("SELECT id, name, timestamp, description FROM recordings WHERE id = ?", (recording_id,))
        recording_row = c.fetchone()
        
        if not recording_row:
            return {"success": False, "error": f"Recording with ID {recording_id} not found"}
            
        recording = dict(recording_row)
        
        # Get measurement data
        c.execute("SELECT data_type, data_json FROM measurements WHERE recording_id = ?", (recording_id,))
        measurements = c.fetchall()
        
        # Organize data by type
        recording_data = {}
        for data_type, data_json in measurements:
            recording_data[data_type] = json.loads(data_json)
        
        recording["data"] = recording_data
        
        conn.close()
        
        return {"success": True, "recording": recording}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def delete_recording(recording_id: int) -> Dict[str, Any]:
    """Delete a recording by ID"""
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        
        # First delete associated measurements
        c.execute("DELETE FROM measurements WHERE recording_id = ?", (recording_id,))
        
        # Then delete the recording itself
        c.execute("DELETE FROM recordings WHERE id = ?", (recording_id,))
        
        conn.commit()
        conn.close()
        
        return {"success": True, "message": f"Recording with ID {recording_id} deleted"}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def compare_recordings(recording_id1: int, recording_id2: int) -> Dict[str, Any]:
    """Compare two recordings and return statistics"""
    try:
        rec1 = get_recording(recording_id1)
        rec2 = get_recording(recording_id2)
        
        if not rec1["success"] or not rec2["success"]:
            return {"success": False, "error": "One or both recordings not found"}
            
        rec1_data = rec1["recording"]["data"]
        rec2_data = rec2["recording"]["data"]
        
        comparison = {
            "recording1": {
                "id": recording_id1,
                "name": rec1["recording"]["name"]
            },
            "recording2": {
                "id": recording_id2,
                "name": rec2["recording"]["name"]
            },
            "metrics": {}
        }
        
        # Compare position data if available
        if "position" in rec1_data and "position" in rec2_data:
            # Calculate statistics (example: average distance between positions)
            # This is a simplistic example - real implementation would need more sophisticated metrics
            positions1 = rec1_data["position"]
            positions2 = rec2_data["position"]
            
            min_len = min(len(positions1), len(positions2))
            
            # Simple Euclidean distance between corresponding points
            total_distance = 0
            for i in range(min_len):
                p1 = positions1[i]
                p2 = positions2[i]
                distance = ((p1["x"] - p2["x"])**2 + (p1["y"] - p2["y"])**2 + (p1["z"] - p2["z"])**2)**0.5
                total_distance += distance
                
            avg_distance = total_distance / min_len if min_len > 0 else 0
            
            comparison["metrics"]["position"] = {
                "avg_distance": avg_distance,
                "points_compared": min_len
            }
        
        return {"success": True, "comparison": comparison}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def save_estimation_data(recording_id: int, estimations: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Save a list of estimation messages to the database"""
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        
        # Insert all estimation messages
        for estimation in estimations:
            c.execute('''
                INSERT INTO estimation_data (
                    recording_id, timestamp, 
                    x, y, z, 
                    yaw, pitch, roll, 
                    acc_x, acc_y, acc_z, 
                    acc_yaw, acc_pitch, acc_roll, 
                    mag_x, mag_y, mag_z, mag_strength, 
                    mouse_movement, mouse_speed, mouse_direction, mouse_distance
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                recording_id, estimation["timestamp"],
                estimation.get("x", 0.0), estimation.get("y", 0.0), estimation.get("z", 0.0),
                estimation.get("yaw", 0.0), estimation.get("pitch", 0.0), estimation.get("roll", 0.0),
                estimation.get("acc_x", 0.0), estimation.get("acc_y", 0.0), estimation.get("acc_z", 0.0),
                estimation.get("acc_yaw", 0.0), estimation.get("acc_pitch", 0.0), estimation.get("acc_roll", 0.0),
                estimation.get("mag_x", 0.0), estimation.get("mag_y", 0.0), estimation.get("mag_z", 0.0), estimation.get("mag_strength", 0.0),
                estimation.get("mouse_movement", 0.0), estimation.get("mouse_speed", 0.0), 
                estimation.get("mouse_direction", 0.0), estimation.get("mouse_distance", 0.0)
            ))
        
        conn.commit()
        conn.close()
        
        return {"success": True, "message": f"Saved {len(estimations)} estimation messages"}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def save_reference_trajectory(recording_id: int, name: str, trajectory_points: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Save a reference trajectory to the database"""
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        
        # Insert all trajectory points
        for point in trajectory_points:
            c.execute('''
                INSERT INTO reference_trajectory (
                    recording_id, name, timestamp, x, y, z
                ) VALUES (?, ?, ?, ?, ?, ?)
            ''', (
                recording_id, name, point["timestamp"],
                point.get("x", 0.0), point.get("y", 0.0), point.get("z", 0.0)
            ))
        
        conn.commit()
        conn.close()
        
        return {"success": True, "message": f"Saved reference trajectory '{name}' with {len(trajectory_points)} points"}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def get_estimation_data(recording_id: int) -> Dict[str, Any]:
    """Get all estimation data for a specific recording"""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        c = conn.cursor()
        
        c.execute('''
            SELECT * FROM estimation_data 
            WHERE recording_id = ? 
            ORDER BY timestamp
        ''', (recording_id,))
        
        rows = c.fetchall()
        estimations = []
        
        for row in rows:
            estimations.append(dict(row))
        
        conn.close()
        
        return {"success": True, "estimations": estimations}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

def get_reference_trajectories(recording_id: int) -> Dict[str, Any]:
    """Get all reference trajectories for a specific recording"""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        c = conn.cursor()
        
        # First get the distinct trajectory names for this recording
        c.execute('''
            SELECT DISTINCT name FROM reference_trajectory 
            WHERE recording_id = ?
        ''', (recording_id,))
        
        names = [row["name"] for row in c.fetchall()]
        trajectories = {}
        
        # Then get the trajectory points for each name
        for name in names:
            c.execute('''
                SELECT * FROM reference_trajectory 
                WHERE recording_id = ? AND name = ? 
                ORDER BY timestamp
            ''', (recording_id, name))
            
            rows = c.fetchall()
            trajectories[name] = [dict(row) for row in rows]
        
        conn.close()
        
        return {"success": True, "trajectories": trajectories}
    
    except Exception as e:
        return {"success": False, "error": str(e)}