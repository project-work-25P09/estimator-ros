# Real-time IMU Data Visualization System

This system provides a real-time visualization of IMU data with persistent storage capabilities. It offers an efficient FastAPI-based server with a modern web frontend for visualizing and recording trajectory data.

## Features

- **Real-time Data Visualization**: Displays position, orientation, acceleration, magnetometer, and mouse data in real-time with minimal latency
- **Data Recording**: Save measurement sessions for later analysis
- **Trajectory Comparison**: Load previously recorded trajectories and compare them with current measurements
- **Adjustable Update Rates**: Configure the update frequency to balance between responsiveness and system load
- **Efficient Implementation**: Leverages FastAPI and direct WebSockets for optimal performance

## System Components

1. **Backend Server**: FastAPI-based server that:
   - Receives ROS messages from the IMU and other sensors
   - Streams data to the frontend via WebSockets
   - Handles data storage and retrieval via SQLite

2. **Frontend Interface**: Modern web interface that:
   - Visualizes data in real-time using Plotly.js
   - Provides controls for recording, pausing, and comparing trajectories
   - Offers responsive design for different screen sizes

3. **Data Storage**: SQLite database for persistent storage of:
   - Position trajectories
   - Orientation data
   - Acceleration readings
   - Magnetometer measurements
   - Mouse movement logs

## Getting Started

### Prerequisites

- ROS 2 (Humble or newer)
- Python 3.8+
- Web browser (Chrome, Firefox, Edge, etc.)

### Running the System

1. Source your ROS workspace:
   ```bash
   source /home/jtammisto/estimator-ros/install/setup.bash
   ```

2. Launch the IMU driver (if not already running):
   ```bash
   ros2 launch microstrain_inertial_driver microstrain.launch.py
   ```

3. Launch the server:
   ```bash
   ros2 launch server server.launch.py
   ```

4. Open a web browser and navigate to:
   ```
   http://localhost:8000
   ```

## Using the Interface

- **Interval Dropdown**: Select the update frequency (50ms to 5000ms)
- **Pause/Continue**: Temporarily pause data visualization without stopping data collection
- **Reset**: Clear all charts and start fresh
- **Start/Stop Recording**: Begin or end a recording session
- **Load Recording**: Open previously saved recordings to compare with current data

## Architecture

The system uses a modern web architecture:

- **Backend**: FastAPI (Python) for high-performance async HTTP and WebSocket handling
- **Database**: SQLite for lightweight yet robust data storage
- **Frontend**: Vanilla JavaScript with Plotly.js for visualization
- **Communication**: WebSocket for real-time data streaming

## Performance Considerations

- The WebSocket implementation provides much better performance than the previous Dash-based solution
- Direct use of Plotly.js (without Dash) significantly reduces frontend latency
- Lightweight SQLite database allows efficient data storage without complex setup
- Adjustable update rates help balance between real-time visibility and system load
