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
let currentTrajectory = "trajectory1";

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
        
        if (!isPaused) {
            updateCharts(data);
            updateDataStore(data);
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
                title: 'X Axis',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            yaxis: { 
                title: 'Y Axis',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            zaxis: { 
                title: 'Z Axis',
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
            title: 'Time',
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
            title: 'Time',
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
            title: 'Time',
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
        name: 'Direction',
        line: { color: '#238636', width: 2 }
    }, {
        type: 'scatter',
        mode: 'lines',
        x: [],
        y: [],
        name: 'Speed',
        line: { color: '#f85149', width: 2 },
        yaxis: 'y2'
    }, {
        type: 'bar',
        x: [],
        y: [],
        name: 'Distance',
        marker: { color: '#58a6ff' },
        opacity: 0.2,
        yaxis: 'y2'
    }], {
        xaxis: { 
            title: 'Time',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis: { 
            title: 'Direction (deg)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e'
        },
        yaxis2: {
            title: 'Speed (m/s) & Distance (m)',
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
}

function updateCharts(data) {
    // Position data update
    const positionUpdate = {
        x: [[data.x]],
        y: [[data.y]],
        z: [[data.z]]
    };
    
    Plotly.extendTraces('position-plot', positionUpdate, [0], 100);
    
    // Orientation data update
    const orientationUpdate = {
        x: [[measurementData.orientation.length], [measurementData.orientation.length], [measurementData.orientation.length]],
        y: [[data.roll], [data.pitch], [data.yaw]]
    };
    
    Plotly.extendTraces('orientation-plot', orientationUpdate, [0, 1, 2], 100);
    
    // Acceleration data update
    const accelerationUpdate = {
        x: [[measurementData.acceleration.length], [measurementData.acceleration.length], [measurementData.acceleration.length]],
        y: [[data.acc_x], [data.acc_y], [data.acc_z]]
    };
    
    Plotly.extendTraces('acceleration-plot', accelerationUpdate, [0, 1, 2], 100);
    
    // Magnetometer data update
    const magnetometerUpdate = {
        x: [[measurementData.magnetometer.length], [measurementData.magnetometer.length], [measurementData.magnetometer.length], [measurementData.magnetometer.length]],
        y: [[data.mag_x], [data.mag_y], [data.mag_z], [data.mag_strength]]
    };
    
    Plotly.extendTraces('magnetometer-plot', magnetometerUpdate, [0, 1, 2, 3], 100);
    
    // Mouse data update
    const mouseUpdate = {
        x: [[measurementData.mouse.length], [measurementData.mouse.length], [measurementData.mouse.length]],
        y: [[data.mouse_direction], [data.mouse_speed], [data.mouse_distance]]
    };
    
    Plotly.extendTraces('mouse-plot', mouseUpdate, [0, 1, 2], 100);
    
    // Update mouse numeric displays
    document.getElementById('mouse-speed').textContent = data.mouse_speed.toFixed(4);
    document.getElementById('mouse-direction').textContent = `${data.mouse_direction.toFixed(4)}°`;
    document.getElementById('mouse-distance').textContent = data.mouse_distance.toFixed(4);
    
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
            timestamp: new Date().toISOString()
        });
        
        // Store orientation data
        measurementData.orientation.push({
            roll: data.roll,
            pitch: data.pitch,
            yaw: data.yaw,
            timestamp: new Date().toISOString()
        });
        
        // Store acceleration data
        measurementData.acceleration.push({
            x: data.acc_x,
            y: data.acc_y,
            z: data.acc_z,
            timestamp: new Date().toISOString()
        });
        
        // Store magnetometer data
        measurementData.magnetometer.push({
            x: data.mag_x,
            y: data.mag_y,
            z: data.mag_z,
            field: data.mag_strength,
            timestamp: new Date().toISOString()
        });
        
        // Store mouse data
        measurementData.mouse.push({
            movement: data.mouse_movement,
            speed: data.mouse_speed,
            direction: data.mouse_direction,
            distance: data.mouse_distance,
            timestamp: new Date().toISOString()
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
                timestamp: new Date().toISOString()
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
        
        Plotly.deleteTraces('mouse-plot', [0, 1, 2]);
        Plotly.addTraces('mouse-plot', [
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Direction', line: { color: '#238636', width: 2 } },
            { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Speed', line: { color: '#f85149', width: 2 }, yaxis: 'y2' },
            { type: 'bar', x: [], y: [], name: 'Distance', marker: { color: '#58a6ff' }, opacity: 0.2, yaxis: 'y2' }
        ]);
    });
    
    // Record button
    document.getElementById('record-button').addEventListener('click', function() {
        isRecording = !isRecording;
        this.innerHTML = isRecording ? 
            '<i class="bi bi-stop-circle"></i> Stop Recording' : 
            '<i class="bi bi-record-circle"></i> Start Recording';
        this.classList.toggle('recording', isRecording);
        
        if (!isRecording && measurementData.position.length > 0) {
            // Save the recorded data
            saveRecording();
        }
    });
    
    // Load recording button
    document.getElementById('load-button').addEventListener('click', function() {
        // Show the load recording dialog
        document.getElementById('load-dialog').style.display = 'block';
        
        // Fetch available recordings
        fetchRecordings();
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
}

function saveRecording() {
    // Create a name for the recording based on timestamp
    const timestamp = new Date().toISOString().replace(/:/g, '-');
    const recordingName = `recording_${timestamp}`;
    
    // Send data to the server for saving
    fetch('/api/save_recording', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            name: recordingName,
            data: measurementData
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            alert(`Recording saved as "${recordingName}"`);
            // Reset the measurement data
            measurementData = {
                position: [],
                orientation: [],
                acceleration: [],
                magnetometer: [],
                mouse: [],
                hardware: []
            };
        } else {
            alert('Failed to save recording: ' + data.error);
        }
    })
    .catch(error => {
        console.error('Error saving recording:', error);
        alert('Error saving recording. See console for details.');
    });
}

function fetchRecordings() {
    fetch('/api/list_recordings')
        .then(response => response.json())
        .then(data => {
            const select = document.getElementById('recording-select');
            select.innerHTML = '';
            
            if (data.recordings && data.recordings.length > 0) {
                data.recordings.forEach(recording => {
                    const option = document.createElement('option');
                    option.value = recording.id;
                    option.textContent = `${recording.name} (${recording.date})`;
                    select.appendChild(option);
                });
                
                document.getElementById('load-recording-btn').disabled = false;
            } else {
                const option = document.createElement('option');
                option.value = '';
                option.textContent = 'No recordings available';
                select.appendChild(option);
                
                document.getElementById('load-recording-btn').disabled = true;
            }
        })
        .catch(error => {
            console.error('Error fetching recordings:', error);
            alert('Error fetching recordings. See console for details.');
        });
}

function loadRecording(recordingId) {
    fetch(`/api/get_recording/${recordingId}`)
        .then(response => response.json())
        .then(data => {
            if (data.success && data.recording) {
                // Display the reference trajectory on the position plot
                Plotly.deleteTraces('position-plot', 0);
                
                // Get position data or use empty arrays if missing
                const positionData = data.recording.data && data.recording.data.position ? data.recording.data.position : [];
                const xData = positionData.length > 0 ? positionData.map(p => p.x || 0) : [];
                const yData = positionData.length > 0 ? positionData.map(p => p.y || 0) : [];
                const zData = positionData.length > 0 ? positionData.map(p => p.z || 0) : [];
                
                Plotly.addTraces('position-plot', [
                    {
                        type: 'scatter3d',
                        mode: 'lines+markers',
                        x: xData,
                        y: yData,
                        z: zData,
                        marker: { size: 3, color: 'green' },
                        line: { width: 2, color: 'green' },
                        name: 'Reference Trajectory'
                    },
                    {
                        type: 'scatter3d',
                        mode: 'lines+markers',
                        x: [],
                        y: [],
                        z: [],
                        marker: { size: 3, color: 'blue' },
                        line: { width: 2, color: 'blue' },
                        name: 'Current Trajectory'
                    }
                ]);
                
                // Store reference data for comparison
                window.referenceData = data.recording;
                
                alert(`Loaded recording: ${data.recording.name}`);
            } else {
                alert('Failed to load recording: ' + (data.error || 'Unknown error'));
            }
        })
        .catch(error => {
            console.error('Error loading recording:', error);
            alert('Error loading recording. See console for details.');
        });
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
