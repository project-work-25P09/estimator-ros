// Main JavaScript file for handling real-time plotting

// Global variables for data management
let measurementData = {
    position: [],
    orientation: [],
    acceleration: [],
    magnetometer: [],
    mouse: [],
    hardware: []
};

let charts = {};
let socket = null;
let isRecording = false;
let isPaused = false;
let isPlayingRecording = false;
let currentTrajectory = "trajectory1";
let currentRecordingId = null;
let recordingBuffer = [];
let referenceTrajectories = {};

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initCharts();
    initWebSocket();
    setupEventListeners();
});

function initWebSocket() {
    // Create WebSocket connection
    const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
    socket = new WebSocket(`${protocol}${window.location.host}/ws`);
    
    // Connection opened
    socket.addEventListener('open', function(event) {
        console.log('Connected to the WebSocket server');
        document.getElementById('connection-status').textContent = 'Connected';
        document.getElementById('connection-status').classList.add('connected');
    });
    
    // Listen for messages
    socket.addEventListener('message', function(event) {
        const data = JSON.parse(event.data);
        
        // Handle different message types
        if (data.type === "connected") {
            console.log('Connected to the WebSocket server');
            document.getElementById('connection-status').textContent = 'Connected';
            document.getElementById('connection-status').classList.add('connected');
            
            // Update recording status if provided
            if (data.recording !== undefined) {
                updateRecordingStatus(data.recording);
            }
        } else if (data.type === "recording_status") {
            updateRecordingStatus(data.recording);
        } else if (data.type === "loaded_recording") {
            handleLoadedRecording(data);
        } else {
            // Update recording status and counter regardless of pause state
            if (data.recording !== undefined) {
                updateRecordingStatus(data.recording);
                
                // Update recording counter if available
                if (data.recording && data.recording_data_points !== undefined) {
                    document.getElementById('recording-counter').textContent = 
                        `Recording: ${data.recording_data_points} data points`;
                }
            }
            
            // Only update charts and store data if not paused or playing a recording
            if (!isPaused && !isPlayingRecording) {
                updateCharts(data);
                updateDataStore(data);
            }
        }
    });
    
    // Connection closed
    socket.addEventListener('close', function(event) {
        console.log('Disconnected from the WebSocket server');
        document.getElementById('connection-status').textContent = 'Disconnected';
        document.getElementById('connection-status').classList.remove('connected');
        
        // Try to reconnect after a delay
        setTimeout(initWebSocket, 2000);
    });
    
    // Connection error
    socket.addEventListener('error', function(event) {
        console.error('WebSocket error:', event);
    });
}

function initCharts() {
    // Position 3D plot
    charts.position3D = new Plotly.newPlot('position-plot', [{
        type: 'scatter3d',
        mode: 'lines+markers',
        x: [],
        y: [],
        z: [],
        marker: {
            size: 4,
            color: '#58a6ff',
            opacity: 0.8
        },
        line: {
            width: 3,
            color: '#58a6ff',
            opacity: 0.7
        },
        name: 'Current Trajectory'
    }], {
        scene: {
            xaxis: { 
                title: 'X Position',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            yaxis: { 
                title: 'Y Position',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            zaxis: { 
                title: 'Z Position',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            aspectmode: 'cube',
            camera: { eye: { x: 1.9, y: 0.7, z: 0.1 } }
        },
        margin: { l: 0, r: 0, b: 0, t: 0 },
        paper_bgcolor: 'rgba(0, 0, 0, 0)',
        plot_bgcolor: 'rgba(0, 0, 0, 0)',
        font: { color: '#f0f6fc' }
    }, { responsive: true });
    
    // Orientation plot
    charts.orientation = new Plotly.newPlot('orientation-plot', [{
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Roll',
        line: { color: '#f85149', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Pitch',
        line: { color: '#238636', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Yaw',
        line: { color: '#58a6ff', width: 2 }
    }], {
        xaxis: { 
            title: 'Time (seconds)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis: { 
            title: 'Value (radians)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        legend: {
            orientation: 'h',
            yanchor: 'bottom',
            y: 1.02,
            xanchor: 'right',
            x: 1
        },
        margin: { l: 50, r: 20, b: 50, t: 20 },
        paper_bgcolor: 'rgba(0, 0, 0, 0)',
        plot_bgcolor: 'rgba(0, 0, 0, 0)',
        font: { color: '#f0f6fc' }
    }, { responsive: true });
    
    // Acceleration plot
    charts.acceleration = new Plotly.newPlot('acceleration-plot', [{
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Acc X',
        line: { color: '#f85149', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Acc Y',
        line: { color: '#238636', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Acc Z',
        line: { color: '#58a6ff', width: 2 }
    }], {
        xaxis: { 
            title: 'Time (seconds)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis: { 
            title: 'Acceleration (m/s²)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        legend: {
            orientation: 'h',
            yanchor: 'bottom',
            y: 1.02,
            xanchor: 'right',
            x: 1
        },
        margin: { l: 50, r: 20, b: 50, t: 20 },
        paper_bgcolor: 'rgba(0, 0, 0, 0)',
        plot_bgcolor: 'rgba(0, 0, 0, 0)',
        font: { color: '#f0f6fc' }
    }, { responsive: true });
    
    // Magnetometer plot
    charts.magnetometer = new Plotly.newPlot('magnetometer-plot', [{
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Mag X',
        line: { color: '#f85149', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Mag Y',
        line: { color: '#238636', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Mag Z',
        line: { color: '#58a6ff', width: 2 }
    }, {
        type: 'bar',
        x: [],
        y: [],
        name: 'Field',
        marker: { color: '#e3b341' },
        opacity: 0.4,
        yaxis: 'y2'
    }], {
        xaxis: { 
            title: 'Time (seconds)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis: { 
            title: 'Value (μT)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis2: {
            title: 'Field Strength',
            overlaying: 'y',
            side: 'right',
            showgrid: false
        },
        legend: {
            orientation: 'h',
            yanchor: 'bottom',
            y: 1.02,
            xanchor: 'right',
            x: 1
        },
        margin: { l: 50, r: 50, b: 50, t: 20 },
        paper_bgcolor: 'rgba(0, 0, 0, 0)',
        plot_bgcolor: 'rgba(0, 0, 0, 0)',
        font: { color: '#f0f6fc' }
    }, { responsive: true });
    
    // Initialize mouse plot
    charts.mouse = new Plotly.newPlot('mouse-plot', [{
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Integrated X',
        line: { color: '#238636', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Integrated Y',
        line: { color: '#f85149', width: 2 }
    }], {
        xaxis: { 
            title: 'Time (seconds)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis: { 
            title: 'Integrated Value',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e'
        },
        legend: {
            orientation: 'h',
            yanchor: 'bottom',
            y: 1.02,
            xanchor: 'right',
            x: 1
        },
        margin: { l: 50, r: 50, b: 50, t: 20 },
        paper_bgcolor: 'rgba(0, 0, 0, 0)',
        plot_bgcolor: 'rgba(0, 0, 0, 0)',
        font: { color: '#f0f6fc' }
    }, { responsive: true });
}

function updateCharts(data) {
    // Position data update for 3D visualization
    const positionUpdate = {
        x: [[data.x]],
        y: [[data.y]],
        z: [[data.z]]
    };
    
    Plotly.extendTraces('position-plot', positionUpdate, [0], 100);
    
    // Orientation data update
    const orientationUpdate = {
        x: [[data.timestamp], [data.timestamp], [data.timestamp]],
        y: [[data.roll], [data.pitch], [data.yaw]]
    };
    
    Plotly.extendTraces('orientation-plot', orientationUpdate, [0, 1, 2], 100);
    
    // Acceleration data update
    const accelerationUpdate = {
        x: [[data.timestamp], [data.timestamp], [data.timestamp]],
        y: [[data.acc_x], [data.acc_y], [data.acc_z]]
    };
    
    Plotly.extendTraces('acceleration-plot', accelerationUpdate, [0, 1, 2], 100);
    
    // Magnetometer data update
    const magnetometerUpdate = {
        x: [[data.timestamp], [data.timestamp], [data.timestamp], [data.timestamp]],
        y: [[data.mag_x], [data.mag_y], [data.mag_z], [data.mag_strength]]
    };
    
    Plotly.extendTraces('magnetometer-plot', magnetometerUpdate, [0, 1, 2, 3], 100);
    
    // Mouse data update
    const mouseUpdate = {
        x: [[data.timestamp], [data.timestamp]],
        y: [[data.mouse_integrated_x], [data.mouse_integrated_y]]
    };
    Plotly.extendTraces('mouse-plot', mouseUpdate, [0, 1], 100);
    
    // Update mouse numeric displays
    document.getElementById('mouse-integrated-x').textContent = data.mouse_integrated_x.toFixed(4);
    document.getElementById('mouse-integrated-y').textContent = data.mouse_integrated_y.toFixed(4);
    
    // Hardware monitor data update (if available)
    if (data.hw_cpu_usage !== undefined) {
        // Update hardware monitor numeric displays with animations
        updateHardwareValue('hw-cpu-usage', `${data.hw_cpu_usage.toFixed(2)}%`, data.hw_cpu_usage);
        updateHardwareValue('hw-memory', `${data.hw_memory_mb.toFixed(2)} MB`, data.hw_memory_mb / 1000); // Normalize for animation
        updateHardwareValue('hw-temperature', `${data.hw_temperature.toFixed(2)}°C`, data.hw_temperature / 100); // Normalize for animation
        updateHardwareValue('hw-network', `↓${data.hw_network_rx_mb.toFixed(2)} MB/s ↑${data.hw_network_tx_mb.toFixed(2)} MB/s`, 
            (data.hw_network_rx_mb + data.hw_network_tx_mb) / 10); // Combined for animation
    }
}

// Function to update hardware values with animations
function updateHardwareValue(elementId, text, value) {
    const element = document.getElementById(elementId);
    const parent = element.closest('.hw-monitor-value');
    
    // Set the new value
    element.textContent = text;
    
    // Add visual feedback based on value changes
    if (element.dataset.prevValue !== undefined) {
        const prevValue = parseFloat(element.dataset.prevValue);
        if (value > prevValue) {
            parent.classList.add('value-increase');
            setTimeout(() => parent.classList.remove('value-increase'), 1000);
        } else if (value < prevValue) {
            parent.classList.add('value-decrease');
            setTimeout(() => parent.classList.remove('value-decrease'), 1000);
        }
    }
    
    // Store current value for next comparison
    element.dataset.prevValue = value;
    
    // Apply color based on threshold for certain metrics
    if (elementId === 'hw-cpu-usage') {
        if (value > 80) {
            element.style.color = 'var(--error-color)';
        } else if (value > 50) {
            element.style.color = 'var(--warning-color)';
        } else {
            element.style.color = 'var(--success-color)';
        }
    } else if (elementId === 'hw-temperature') {
        if (value * 100 > 75) {
            element.style.color = 'var(--error-color)';
        } else if (value * 100 > 60) {
            element.style.color = 'var(--warning-color)';
        } else {
            element.style.color = 'var(--success-color)';
        }
    }
}

function updateDataStore(data) {
    // Only store data if we're recording
    if (isRecording) {
        // Store position data
        measurementData.position.push({
            x: data.x,
            y: data.y,
            z: data.z,
            timestamp: data.timestamp
        });
        
        // Store orientation data
        measurementData.orientation.push({
            roll: data.roll,
            pitch: data.pitch,
            yaw: data.yaw,
            timestamp: data.timestamp
        });
        
        // Store acceleration data
        measurementData.acceleration.push({
            x: data.acc_x,
            y: data.acc_y,
            z: data.acc_z,
            timestamp: data.timestamp
        });
        
        // Store magnetometer data
        measurementData.magnetometer.push({
            x: data.mag_x,
            y: data.mag_y,
            z: data.mag_z,
            field: data.mag_strength,
            timestamp: data.timestamp
        });
        
        // Store mouse integrated data
        measurementData.mouse.push({
            integrated_x: data.mouse_integrated_x,
            integrated_y: data.mouse_integrated_y,
            timestamp: data.timestamp
        });
        
        // Store hardware monitor data if available
        if (data.hw_cpu_usage !== undefined) {
            measurementData.hardware.push({
                cpu_usage: data.hw_cpu_usage,
                memory_mb: data.hw_memory_mb,
                disk_rx_mb: data.hw_disk_rx_mb,
                disk_tx_mb: data.hw_disk_tx_mb,
                network_rx_mb: data.hw_network_rx_mb,
                network_tx_mb: data.hw_network_tx_mb,
                power_consumption: data.hw_power_consumption,
                temperature: data.hw_temperature,
                timestamp: data.timestamp
            });
        }
    }
}

function setupEventListeners() {
    // Initialize theme from local storage or default to dark
    initializeTheme();
    
    // Theme toggle button
    document.getElementById('theme-toggle').addEventListener('click', function() {
        toggleTheme();
    });
    
    // Pause/Continue button
    document.getElementById('pause-button').addEventListener('click', function() {
        isPaused = !isPaused;
        this.innerHTML = isPaused ? '<i class="bi bi-play-fill"></i> Continue' : '<i class="bi bi-pause-fill"></i> Pause';
    });
    
    // Reset button
    document.getElementById('reset-button').addEventListener('click', function() {
        // Clear all the charts
        Plotly.deleteTraces('position-plot', 0);
        Plotly.addTraces('position-plot', {
            type: 'scatter3d',
            mode: 'lines+markers',
            x: [],
            y: [],
            z: [],
            marker: { size: 4, color: '#58a6ff', opacity: 0.8 },
            line: { width: 3, color: '#58a6ff', opacity: 0.7 },
            name: 'Current Trajectory'
        });
        
        Plotly.deleteTraces('orientation-plot', [0, 1, 2]);
        Plotly.addTraces('orientation-plot', [
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Roll', line: { color: '#f85149', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Pitch', line: { color: '#238636', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Yaw', line: { color: '#58a6ff', width: 2 } }
        ]);
        
        Plotly.deleteTraces('acceleration-plot', [0, 1, 2]);
        Plotly.addTraces('acceleration-plot', [
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Acc X', line: { color: '#f85149', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Acc Y', line: { color: '#238636', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Acc Z', line: { color: '#58a6ff', width: 2 } }
        ]);
        
        Plotly.deleteTraces('magnetometer-plot', [0, 1, 2, 3]);
        Plotly.addTraces('magnetometer-plot', [
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Mag X', line: { color: '#f85149', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Mag Y', line: { color: '#238636', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Mag Z', line: { color: '#58a6ff', width: 2 } },
            { type: 'bar', x: [], y: [], name: 'Field', marker: { color: '#e3b341' }, opacity: 0.4, yaxis: 'y2' }
        ]);
        
        Plotly.deleteTraces('mouse-plot', [0, 1]);
        Plotly.addTraces('mouse-plot', [
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Integrated X', line: { color: '#238636', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Integrated Y', line: { color: '#f85149', width: 2 } }
        ]);

        fetch('/api/reset_estimator', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
    });
    
    // Record button
    document.getElementById('record-button').addEventListener('click', function() {
        if (!isRecording) {
            // Start recording
            startRecording();
        } else {
            // Stop recording
            stopRecording();
        }
    });
    
    // Load recording button
    document.getElementById('load-button').addEventListener('click', function() {
        // Show the load recording dialog
        // document.getElementById('load-dialog').style.display = 'block';
        // Fetch available recordings
        // fetchRecordings();
        showRecordingsList();
    });
    
    // Close dialog button
    document.getElementById('close-dialog').addEventListener('click', function() {
        document.getElementById('load-dialog').style.display = 'none';
    });
    
    // Interval selection dropdown
    document.getElementById('interval-dropdown').addEventListener('change', function() {
        // Send the new interval to the server
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify({
                type: 'set_interval',
                value: parseInt(this.value)
            }));
        }
    });
    
    // Load recording from the dialog
    document.getElementById('load-recording-btn').addEventListener('click', function() {
        const selectedRecording = document.getElementById('recording-select').value;
        if (selectedRecording) {
            loadRecording(selectedRecording);
            document.getElementById('load-dialog').style.display = 'none';
        }
    });

    document.getElementById('apply-estimator-button').addEventListener('click', function() {
        const dropdown = document.getElementById('estimator-dropdown');
        const selectedEstimator = dropdown.value;
    
        fetch('/api/switch_estimator', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ estimator_name: selectedEstimator })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                alert(`Successfully switched to estimator: ${selectedEstimator}`);
            } else {
                alert(`Failed to switch estimator: ${data.message}`);
            }
        })
        .catch(error => {
            console.error('Error switching estimator:', error);
            alert('An error occurred while switching the estimator.');
        });
    });
}

function updateDataStore(data) {
    // Add data to measurement store (for later saving)
    const timestamp = data.timestamp;
    
    measurementData.position.push({
        timestamp: timestamp,
        x: data.x,
        y: data.y,
        z: data.z
    });
    
    measurementData.orientation.push({
        timestamp: timestamp,
        roll: data.roll,
        pitch: data.pitch,
        yaw: data.yaw
    });
    
    measurementData.acceleration.push({
        timestamp: timestamp,
        x: data.acc_x,
        y: data.acc_y,
        z: data.acc_z,
        yaw: data.acc_yaw,
        pitch: data.acc_pitch,
        roll: data.acc_roll
    });
    
    measurementData.magnetometer.push({
        timestamp: timestamp,
        x: data.mag_x,
        y: data.mag_y,
        z: data.mag_z,
        strength: data.mag_strength
    });
    
    measurementData.mouse.push({
        timestamp: timestamp,
        integrated_x: data.mouse_integrated_x,
        integrated_y: data.mouse_integrated_y
    });
    
    // Add hardware data if available
    if (data.hw_cpu_usage !== undefined) {
        measurementData.hardware.push({
            timestamp: timestamp,
            cpu_usage: data.hw_cpu_usage,
            memory_mb: data.hw_memory_mb,
            disk_rx_mb: data.disk_rx_mb,
            disk_tx_mb: data.disk_tx_mb,
            network_rx_mb: data.hw_network_rx_mb,
            network_tx_mb: data.hw_network_tx_mb,
            power_consumption: data.hw_power_consumption,
            temperature: data.hw_temperature
        });
    }
    
    // Limit the size of the arrays (to prevent memory issues)
    const maxSize = 5000;
    Object.keys(measurementData).forEach(key => {
        if (measurementData[key].length > maxSize) {
            measurementData[key] = measurementData[key].slice(-maxSize);
        }
    });
    
    // Update data displays
    document.getElementById('position-x').textContent = data.x.toFixed(4);
    document.getElementById('position-y').textContent = data.y.toFixed(4);
    document.getElementById('position-z').textContent = data.z.toFixed(4);
    
    // Update orientation displays with both radians and degrees
    document.getElementById('orientation-roll').textContent = 
        `${data.roll.toFixed(4)} rad (${(data.roll * 180 / Math.PI).toFixed(2)}°)`;
    document.getElementById('orientation-pitch').textContent = 
        `${data.pitch.toFixed(4)} rad (${(data.pitch * 180 / Math.PI).toFixed(2)}°)`;
    document.getElementById('orientation-yaw').textContent = 
        `${data.yaw.toFixed(4)} rad (${(data.yaw * 180 / Math.PI).toFixed(2)}°)`;
    
    // Update acceleration displays
    document.getElementById('acceleration-x').textContent = `${data.acc_x.toFixed(4)} m/s²`;
    document.getElementById('acceleration-y').textContent = `${data.acc_y.toFixed(4)} m/s²`;
    document.getElementById('acceleration-z').textContent = `${data.acc_z.toFixed(4)} m/s²`;
    
    // Update magnetometer displays
    document.getElementById('magnetometer-x').textContent = `${data.mag_x.toFixed(4)} μT`;
    document.getElementById('magnetometer-y').textContent = `${data.mag_y.toFixed(4)} μT`;
    document.getElementById('magnetometer-z').textContent = `${data.mag_z.toFixed(4)} μT`;
    document.getElementById('magnetometer-strength').textContent = `${data.mag_strength.toFixed(4)} μT`;
}

function updateRecordingStatus(recording) {
    isRecording = recording;
    const recordButton = document.getElementById('record-button');
    
    if (recording) {
        recordButton.innerHTML = '<i class="bi bi-stop-circle"></i> Stop Recording';
        recordButton.classList.add('recording');
        
        // Show recording indicator
        document.getElementById('recording-indicator').style.display = 'flex';
        
        // Initialize the recording counter to 0 when starting recording
        document.getElementById('recording-counter').textContent = 'Recording: 0 data points';
        
        // Hide the recording list if it's open
        const recordingListModal = document.getElementById('recording-list-modal');
        if (recordingListModal) {
            recordingListModal.style.display = 'none';
        }
    } else {
        recordButton.innerHTML = '<i class="bi bi-record-circle"></i> Start Recording';
        recordButton.classList.remove('recording');
        
        // Hide recording indicator
        document.getElementById('recording-indicator').style.display = 'none';
    }
}

function startRecording() {
    // Clear measurement data before starting new recording
    resetMeasurementData();
    
    // Send command to start recording
    if (socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({
            type: 'start_recording'
        }));
    }
}

function stopRecording() {
    // Prompt for recording name and description
    const name = prompt('Enter a name for this recording:', 
        `recording_${new Date().toISOString().replace(/[:.]/g, '-')}`);
    
    if (name === null) {
        // User canceled
        return;
    }
    
    const description = prompt('Enter a description (optional):', '');
    
    // Send command to stop recording
    if (socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({
            type: 'stop_recording',
            name: name,
            description: description || ''
        }));
        
        // Send to server API to save
        fetch('/api/command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                type: 'stop_recording',
                data: {
                    name: name,
                    description: description || ''
                }
            })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                alert(`Recording saved as: ${name}`);
                currentRecordingId = data.recording_id;
            } else {
                alert(`Error saving recording: ${data.message || 'Unknown error'}`);
            }
        })
        .catch(error => {
            console.error('Error saving recording:', error);
            alert('Error saving recording. See console for details.');
        });
    }
}

function resetMeasurementData() {
    // Reset all measurement data arrays
    Object.keys(measurementData).forEach(key => {
        measurementData[key] = [];
    });
    
    recordingBuffer = [];
}

function resetCharts() {
    // Reset all charts to empty state
    Plotly.deleteTraces('position-plot', 0);
    Plotly.addTraces('position-plot', {
        type: 'scatter3d',
        mode: 'lines+markers',
        x: [],
        y: [],
        z: [],
        marker: {
            size: 4,
            color: '#58a6ff',
            opacity: 0.8
        },
        line: {
            width: 3,
            color: '#58a6ff',
            opacity: 0.7
        },
        name: 'Current Trajectory'
    });
    
    // Reset other charts
    ['orientation-plot', 'acceleration-plot', 'magnetometer-plot', 'mouse-plot'].forEach(plot => {
        const traces = document.getElementById(plot).data;
        for (let i = 0; i < traces.length; i++) {
            Plotly.deleteTraces(plot, 0);
        }
    });
    
    // Re-initialize charts
    initCharts();
    
    // Reset measurement data
    resetMeasurementData();
}

function showRecordingsList() {
    console.log("showRecordingsList called");
    // Fetch the list of recordings from the server
    fetch('/api/list_recordings')
    .then(response => response.json())
    .then(data => {
        console.log("/api/list_recordings response:", data);
        if (data.success && Array.isArray(data.recordings)) {
            if (data.recordings.length === 0) {
                alert('No recordings found in the database.');
            }
            displayRecordingsList(data.recordings);
        } else {
            alert(`Error loading recordings: ${data.error || 'Unknown error'}`);
        }
    })
    .catch(error => {
        console.error('Error loading recordings:', error);
        alert('Error loading recordings. See console for details.');
    });
}

function displayRecordingsList(recordings) {
    console.log("displayRecordingsList called with", recordings);
    // Create or update modal dialog
    let modal = document.getElementById('recording-list-modal');
    
    if (!modal) {
        modal = document.createElement('div');
        modal.id = 'recording-list-modal';
        modal.className = 'modal';
        document.body.appendChild(modal);
    }
    
    // Create modal content
    let html = `
        <div class="modal-content">
            <div class="modal-header">
                <h2>Saved Recordings</h2>
                <span class="close-button" onclick="document.getElementById('recording-list-modal').style.display='none'">&times;</span>
            </div>
            <div class="modal-body">
                <table class="recordings-table">
                    <thead>
                        <tr>
                            <th>Name</th>
                            <th>Date</th>
                            <th>Description</th>
                            <th>Actions</th>
                        </tr>
                    </thead>
                    <tbody>
    `;
    
    if (recordings.length === 0) {
        html += `
            <tr>
                <td colspan="4" class="no-recordings">No recordings found</td>
            </tr>
        `;
    } else {
        recordings.forEach(rec => {
            html += `
                <tr>
                    <td>${rec.name}</td>
                    <td>${rec.date}</td>
                    <td>${rec.description || '-'}</td>
                    <td>
                        <button onclick="loadRecording(${rec.id})">Load</button>
                        <button onclick="deleteRecording(${rec.id})">Delete</button>
                    </td>
                </tr>
            `;
        });
    }
    
    html += `
                    </tbody>
                </table>
            </div>
        </div>
    `;
    
    modal.innerHTML = html;
    modal.style.display = 'block';
}

function loadRecording(recordingId) {
    if (socket.readyState === WebSocket.OPEN) {
        // Reset current view
        resetCharts();
        
        // Pause realtime updates
        isPaused = true;
        document.getElementById('pause-button').innerHTML = '<i class="bi bi-play-fill"></i> Resume';
        
        // Set the playback mode
        isPlayingRecording = true;
        currentRecordingId = recordingId;
        
        // Request recording data via websocket
        socket.send(JSON.stringify({
            type: 'load_recording',
            recording_id: recordingId
        }));
        
        // Close the modal
        document.getElementById('recording-list-modal').style.display = 'none';
    }
}

function handleLoadedRecording(data) {
    // Store the recording data
    const estimations = data.estimations || [];
    referenceTrajectories = data.reference_trajectories || {};
    
    // Show toast notification
    showToast(`Loaded recording: ${data.recording.name || 'Unnamed'} (${estimations.length} points)`);
    
    // Plot all estimation data points
    if (estimations.length > 0) {
        plotRecordedTrajectory(estimations);
    }
    
    // Add reference trajectories if available
    for (const [name, points] of Object.entries(referenceTrajectories)) {
        if (points && points.length > 0) {
            addReferenceTrajectory(name, points);
        }
    }
    
    // Update UI to show we're in playback mode
    document.getElementById('recording-playback-indicator').style.display = 'flex';
    document.getElementById('recording-playback-name').textContent = 
        data.recording.name || `Recording #${data.recording_id}`;
}

function plotRecordedTrajectory(estimations) {
    // Extract data for each chart
    const timestamps = estimations.map(e => e.timestamp);
    const xPositions = estimations.map(e => e.x);
    const yPositions = estimations.map(e => e.y);
    const zPositions = estimations.map(e => e.z);
    
    const rolls = estimations.map(e => e.roll);
    const pitches = estimations.map(e => e.pitch);
    const yaws = estimations.map(e => e.yaw);
    
    const accXs = estimations.map(e => e.acc_x);
    const accYs = estimations.map(e => e.acc_y);
    const accZs = estimations.map(e => e.acc_z);
    
    const magXs = estimations.map(e => e.mag_x);
    const magYs = estimations.map(e => e.mag_y);
    const magZs = estimations.map(e => e.mag_z);
    const magStrengths = estimations.map(e => e.mag_strength);
    
    // Mouse integrated values
    const mouseIntegratedXs = estimations.map(e => e.mouse_integrated_x);
    const mouseIntegratedYs = estimations.map(e => e.mouse_integrated_y);
    
    // Plot 3D position
    Plotly.deleteTraces('position-plot', 0);
    Plotly.addTraces('position-plot', {
        type: 'scatter3d',
        mode: 'lines+markers',
        x: xPositions,
        y: yPositions,
        z: zPositions,
        marker: {
            size: 4,
            color: '#58a6ff',
            opacity: 0.8
        },
        line: {
            width: 3,
            color: '#58a6ff',
            opacity: 0.7
        },
        name: 'Estimated Trajectory'
    });
    
    // Plot orientation data
    Plotly.deleteTraces('orientation-plot', [0, 1, 2]);
    Plotly.addTraces('orientation-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: rolls,
            name: 'Roll',
            line: { color: '#f85149', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: pitches,
            name: 'Pitch',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: yaws,
            name: 'Yaw',
            line: { color: '#58a6ff', width: 2 }
        }
    ]);
    
    // Plot acceleration data
    Plotly.deleteTraces('acceleration-plot', [0, 1, 2]);
    Plotly.addTraces('acceleration-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: accXs,
            name: 'Acc X',
            line: { color: '#f85149', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: accYs,
            name: 'Acc Y',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: accZs,
            name: 'Acc Z',
            line: { color: '#58a6ff', width: 2 }
        }
    ]);
    
    // Plot magnetometer data
    Plotly.deleteTraces('magnetometer-plot', [0, 1, 2, 3]);
    Plotly.addTraces('magnetometer-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: magXs,
            name: 'Mag X',
            line: { color: '#f85149', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: magYs,
            name: 'Mag Y',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: magZs,
            name: 'Mag Z',
            line: { color: '#58a6ff', width: 2 }
        },
        {
            type: 'bar',
            x: timestamps,
            y: magStrengths,
            name: 'Field',
            marker: { color: '#e3b341' },
            opacity: 0.4,
            yaxis: 'y2'
        }
    ]);
    
    // Plot mouse integrated data
    Plotly.deleteTraces('mouse-plot', [0, 1]);
    Plotly.addTraces('mouse-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: mouseIntegratedXs,
            name: 'Integrated X',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: mouseIntegratedYs,
            name: 'Integrated Y',
            line: { color: '#f85149', width: 2 }
        }
    ]);
    
    // Update last values in the displays
    if (estimations.length > 0) {
        const lastEstimation = estimations[estimations.length - 1];
        // Position
        document.getElementById('position-x').textContent = lastEstimation.x.toFixed(4);
        document.getElementById('position-y').textContent = lastEstimation.y.toFixed(4);
        document.getElementById('position-z').textContent = lastEstimation.z.toFixed(4);
        // Orientation
        document.getElementById('orientation-roll').textContent = 
            `${lastEstimation.roll.toFixed(4)} rad (${(lastEstimation.roll * 180 / Math.PI).toFixed(2)}°)`;
        document.getElementById('orientation-pitch').textContent = 
            `${lastEstimation.pitch.toFixed(4)} rad (${(lastEstimation.pitch * 180 / Math.PI).toFixed(2)}°)`;
        document.getElementById('orientation-yaw').textContent = 
            `${lastEstimation.yaw.toFixed(4)} rad (${(lastEstimation.yaw * 180 / Math.PI).toFixed(2)}°)`;
        // Acceleration
        document.getElementById('acceleration-x').textContent = `${lastEstimation.acc_x.toFixed(4)} m/s²`;
        document.getElementById('acceleration-y').textContent = `${lastEstimation.acc_y.toFixed(4)} m/s²`;
        document.getElementById('acceleration-z').textContent = `${lastEstimation.acc_z.toFixed(4)} m/s²`;
        // Magnetometer
        document.getElementById('magnetometer-x').textContent = `${lastEstimation.mag_x.toFixed(4)} μT`;
        document.getElementById('magnetometer-y').textContent = `${lastEstimation.mag_y.toFixed(4)} μT`;
        document.getElementById('magnetometer-z').textContent = `${lastEstimation.mag_z.toFixed(4)} μT`;
        document.getElementById('magnetometer-strength').textContent = `${lastEstimation.mag_strength.toFixed(4)} μT`;
        // Mouse integrated
        document.getElementById('mouse-integrated-x').textContent = lastEstimation.mouse_integrated_x.toFixed(4);
        document.getElementById('mouse-integrated-y').textContent = lastEstimation.mouse_integrated_y.toFixed(4);
    }
}

function addReferenceTrajectory(name, points) {
    // Extract position data
    const xPositions = points.map(p => p.x);
    const yPositions = points.map(p => p.y);
    const zPositions = points.map(p => p.z);
    
    // Add the reference trajectory to the 3D plot
    Plotly.addTraces('position-plot', {
        type: 'scatter3d',
        mode: 'lines',
        x: xPositions,
        y: yPositions,
        z: zPositions,
        line: {
            width: 4,
            color: '#fc6d26', // Different color for reference
            opacity: 0.9
        },
        name: `Reference: ${name}`
    });
}

function deleteRecording(recordingId) {
    if (confirm('Are you sure you want to delete this recording? This action cannot be undone.')) {
        fetch(`/api/delete_recording/${recordingId}`, {
            method: 'DELETE'
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                alert('Recording deleted successfully');
                // Refresh the recordings list
                showRecordingsList();
            } else {
                alert(`Error deleting recording: ${data.error || 'Unknown error'}`);
            }
        })
        .catch(error => {
            console.error('Error deleting recording:', error);
            alert('Error deleting recording. See console for details.');
        });
    }
}

function showToast(message) {
    // Create toast container if it doesn't exist
    let toast = document.getElementById('toast');
    if (!toast) {
        toast = document.createElement('div');
        toast.id = 'toast';
        document.body.appendChild(toast);
    }
    
    // Set message and show toast
    toast.textContent = message;
    toast.className = 'show';
    
    // Hide toast after 3 seconds
    setTimeout(() => {
        toast.className = '';
    }, 3000);
}

// Theme management functions
function initializeTheme() {
    // Check for saved theme preference or use device preference
    const savedTheme = localStorage.getItem('theme');
    if (savedTheme) {
        document.body.classList.toggle('light-theme', savedTheme === 'light');
        updateThemeIcon(savedTheme === 'light');
    } else {
        // Use device preference
        const prefersDarkScheme = window.matchMedia('(prefers-color-scheme: dark)').matches;
        document.body.classList.toggle('light-theme', !prefersDarkScheme);
        updateThemeIcon(!prefersDarkScheme);
    }
    
    // Update plot styles to match theme
    updatePlotStyles();
}

function toggleTheme() {
    const isLightTheme = document.body.classList.toggle('light-theme');
    localStorage.setItem('theme', isLightTheme ? 'light' : 'dark');
    updateThemeIcon(isLightTheme);
    
    // Update plot styles to match theme
    updatePlotStyles();
}

function updateThemeIcon(isLightTheme) {
    const themeIcon = document.querySelector('#theme-toggle i');
    themeIcon.className = isLightTheme ? 'bi bi-sun-fill' : 'bi bi-moon-fill';
}

function updatePlotStyles() {
    const isLightTheme = document.body.classList.contains('light-theme');
    
    // Colors for light and dark themes
    const gridColor = isLightTheme ? '#dfe5ec' : '#30363d';
    const zeroLineColor = isLightTheme ? '#57606a' : '#8b949e';
    const fontColor = isLightTheme ? '#24292f' : '#f0f6fc';
    
    // Update each plot with new theme colors
    const updateLayout = {
        xaxis: { 
            gridcolor: gridColor,
            zerolinecolor: zeroLineColor
        },
        yaxis: { 
            gridcolor: gridColor,
            zerolinecolor: zeroLineColor 
        },
        font: { color: fontColor }
    };
    
    // Apply to each plot
    const plotIds = ['orientation-plot', 'acceleration-plot', 'magnetometer-plot', 'mouse-plot'];
    plotIds.forEach(id => {
        Plotly.relayout(id, updateLayout);
    });
    
    // Special update for 3D plot
    Plotly.relayout('position-plot', {
        scene: {
            xaxis: { 
                gridcolor: gridColor,
                zerolinecolor: zeroLineColor
            },
            yaxis: { 
                gridcolor: gridColor,
                zerolinecolor: zeroLineColor
            },
            zaxis: { 
                gridcolor: gridColor,
                zerolinecolor: zeroLineColor
            }
        },
        font: { color: fontColor }
    });
}
