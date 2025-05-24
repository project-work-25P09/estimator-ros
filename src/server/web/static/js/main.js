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
let firstTimestamp = null; // Track the first timestamp for relative time calculations

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
            size: 2,
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
                zerolinecolor: '#58a6ff',
                nticks: 10,
                showgrid: true,
                showline: true
            },
            yaxis: { 
                title: 'Y Position',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff',
                nticks: 10,
                showgrid: true,
                showline: true
            },
            zaxis: { 
                title: 'Z Position',
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff',
                nticks: 10,
                showgrid: true,
                showline: true
            },
            aspectmode: 'cube',
            camera: { eye: { x: 1.9, y: 0.7, z: 0.1 } }
        },
        margin: { l: 0, r: 0, b: 0, t: 0 },
        paper_bgcolor: 'rgba(0, 0, 0, 0)',
        plot_bgcolor: 'rgba(0, 0, 0, 0)',
        font: { color: '#f0f6fc' }
    }, { responsive: true });
    
    // Position 2D plot (new)
    charts.position2D = new Plotly.newPlot('position-2d-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: [],
            y: [],
            name: 'Position X',
            line: { color: '#f85149', width: 2 }
        }, 
        {
            type: 'scatter',
            mode: 'lines',
            x: [],
            y: [],
            name: 'Position Y',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: [],
            y: [],
            name: 'Position Z',
            line: { color: '#58a6ff', width: 2 }
        }
    ], {
        xaxis: { 
            title: 'Time (seconds)',
            gridcolor: '#30363d',
            zerolinecolor: '#8b949e' 
        },
        yaxis: { 
            title: 'Position (m)',
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
            title: 'Angle (degrees)',
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
            title: 'Field Strength (μT)',
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
            title: 'Relative position (m)',
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
    // Set the first timestamp if not already set
    if (firstTimestamp === null) {
        firstTimestamp = data.timestamp;
    }
    
    // Calculate relative time (seconds since first data point)
    const relativeTime = data.timestamp - firstTimestamp;
    
    // Position data update for 3D visualization
    const positionUpdate = {
        x: [[data.x]],
        y: [[data.y]],
        z: [[data.z]]
    };
    
    Plotly.extendTraces('position-plot', positionUpdate, [0], 10000);
    
    // Update the 3D plot with equal axis scaling
    updatePosition3DPlotWithEqualAxes();
    
    // Position 2D data update
    const position2DUpdate = {
        x: [[relativeTime], [relativeTime], [relativeTime]],
        y: [[data.x], [data.y], [data.z]]
    };
    
    Plotly.extendTraces('position-2d-plot', position2DUpdate, [0, 1, 2], 100);
    
    // Orientation data update - convert radians to degrees
    const orientationUpdate = {
        x: [[relativeTime], [relativeTime], [relativeTime]],
        y: [
            [data.roll * 180 / Math.PI], // Convert roll to degrees
            [data.pitch * 180 / Math.PI], // Convert pitch to degrees
            [data.yaw * 180 / Math.PI] // Convert yaw to degrees
        ]
    };
    
    Plotly.extendTraces('orientation-plot', orientationUpdate, [0, 1, 2], 100);
    
    // Acceleration data update
    const accelerationUpdate = {
        x: [[relativeTime], [relativeTime], [relativeTime]],
        y: [[data.acc_x], [data.acc_y], [data.acc_z]]
    };
    
    Plotly.extendTraces('acceleration-plot', accelerationUpdate, [0, 1, 2], 100);
    
    // Magnetometer data update
    const magnetometerUpdate = {
        x: [[relativeTime], [relativeTime], [relativeTime], [relativeTime]],
        y: [[data.mag_x], [data.mag_y], [data.mag_z], [data.mag_strength]]
    };
    
    Plotly.extendTraces('magnetometer-plot', magnetometerUpdate, [0, 1, 2, 3], 100);
    
    // Mouse data update
    const mouseUpdate = {
        x: [[relativeTime], [relativeTime]],
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
    if (themeIcon) {
        themeIcon.className = isLightTheme ? 'bi bi-sun-fill' : 'bi bi-moon-fill';
    }
}

function updatePlotStyles() {
    const isLightTheme = document.body.classList.contains('light-theme');
    
    // Colors for light and dark themes
    const gridColor = isLightTheme ? '#dfe5ec' : '#30363d';
    const zeroLineColor = isLightTheme ? '#57606a' : '#8b949e';
    const fontColor = isLightTheme ? '#24292f' : '#f0f6fc';
    
    // Update each plot with new theme colors
    const plotIds = ['orientation-plot', 'acceleration-plot', 'magnetometer-plot', 'mouse-plot', 'position-2d-plot'];
    plotIds.forEach(id => {
        try {
            Plotly.relayout(id, {
                xaxis: { 
                    gridcolor: gridColor,
                    zerolinecolor: zeroLineColor
                },
                yaxis: { 
                    gridcolor: gridColor,
                    zerolinecolor: zeroLineColor 
                },
                font: { color: fontColor }
            });
        } catch (e) {
            console.warn(`Could not update styles for plot ${id}:`, e);
        }
    });
    
    // Special update for 3D plot
    try {
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
    } catch (e) {
        console.warn('Could not update styles for 3D plot:', e);
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
        fetch('/api/reset_estimator', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        
        // Reset the first timestamp to null so a new one will be captured
        firstTimestamp = null;
        
        // Clear reference trajectories
        referenceTrajectories = {};

        setTimeout(() => {
            // Clear all the charts
            // Need to delete all traces from the position plot, not just index 0
            const positionPlot = document.getElementById('position-plot');
            if (positionPlot && positionPlot.data) {
                // Delete all traces from position-plot by removing them one by one from the end
                while (positionPlot.data.length > 0) {
                    Plotly.deleteTraces('position-plot', positionPlot.data.length - 1);
                }
            }
            
            // Add a fresh trace for the new trajectory
            Plotly.addTraces('position-plot', {
                type: 'scatter3d',
                mode: 'lines+markers',
                x: [],
                y: [],
                z: [],
                marker: { size: 2, color: '#58a6ff', opacity: 0.8 },
                line: { width: 3, color: '#58a6ff', opacity: 0.7 },
                name: 'Current Trajectory'
            });
            
            // Reset 3D plot axis ranges
            resetPosition3DPlotScaling();
            
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
            
            Plotly.deleteTraces('position-2d-plot', [0, 1, 2]);
            Plotly.addTraces('position-2d-plot', [
                { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Position X', line: { color: '#f85149', width: 2 } },
                { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Position Y', line: { color: '#238636', width: 2 } },
                { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Position Z', line: { color: '#58a6ff', width: 2 } }
            ]);
            
            Plotly.deleteTraces('mouse-plot', [0, 1]);
            Plotly.addTraces('mouse-plot', [
                { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Integrated X', line: { color: '#238636', width: 2 } },
                { type: 'scatter', mode: 'lines', x: [], y: [], name: 'Integrated Y', line: { color: '#f85149', width: 2 } }
            ]);
        }, 1000);
    });
    
    // Reset Measurements Only button
    document.getElementById('reset-measurements-button').addEventListener('click', function() {
        fetch('/api/reset_estimator', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });
        
        // Reset just the measurements while keeping reference trajectories
        resetMeasurementsOnly();
    });
    
    // Load Reference from CSV button
    document.getElementById('load-reference-csv-button').addEventListener('click', function() {
        loadReferenceFromCSV();
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
        showRecordingsList();
    });
    
    // Close dialog button
    document.getElementById('close-dialog').addEventListener('click', function() {
        document.getElementById('load-dialog').style.display = 'none';
    });
    
    // Close reference dialog button
    document.getElementById('close-reference-dialog').addEventListener('click', function() {
        document.getElementById('reference-dialog').style.display = 'none';
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

        setTimeout(() => {
            resetMeasurementData();
            resetCharts();
        }, 100);
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
    
    // Update data displays with null checks
    const updateElement = (id, value) => {
        const element = document.getElementById(id);
        if (element) element.textContent = value;
    };
    
    // Position updates
    updateElement('position-x', data.x.toFixed(4));
    updateElement('position-y', data.y.toFixed(4));
    updateElement('position-z', data.z.toFixed(4));
    
    // Orientation updates with both radians and degrees
    updateElement('orientation-roll', 
        `${data.roll.toFixed(4)} rad (${(data.roll * 180 / Math.PI).toFixed(2)}°)`);
    updateElement('orientation-pitch', 
        `${data.pitch.toFixed(4)} rad (${(data.pitch * 180 / Math.PI).toFixed(2)}°)`);
    updateElement('orientation-yaw', 
        `${data.yaw.toFixed(4)} rad (${(data.yaw * 180 / Math.PI).toFixed(2)}°)`);
    
    // Acceleration updates
    updateElement('acceleration-x', `${data.acc_x.toFixed(4)} m/s²`);
    updateElement('acceleration-y', `${data.acc_y.toFixed(4)} m/s²`);
    updateElement('acceleration-z', `${data.acc_z.toFixed(4)} m/s²`);
    
    // Magnetometer updates
    updateElement('magnetometer-x', `${data.mag_x.toFixed(4)} μT`);
    updateElement('magnetometer-y', `${data.mag_y.toFixed(4)} μT`);
    updateElement('magnetometer-z', `${data.mag_z.toFixed(4)} μT`);
    updateElement('magnetometer-strength', `${data.mag_strength.toFixed(4)} μT`);
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
    
    // Reset the first timestamp to null so a new one will be captured
    firstTimestamp = null;
}

function resetCharts(keepReferenceTrajectories = false) {
    // Reset all charts to empty state
    // Need to delete all traces from the position plot, not just index 0
    const positionPlot = document.getElementById('position-plot');
    if (positionPlot && positionPlot.data) {
        if (keepReferenceTrajectories) {
            // Only delete the measurement trace (index 0) when keeping reference trajectories
            Plotly.deleteTraces('position-plot', 0);
        } else {
            // Delete all traces from position-plot by removing them one by one
            while (positionPlot.data.length > 0) {
                Plotly.deleteTraces('position-plot', positionPlot.data.length - 1);
            }
        }
    }
    
    Plotly.addTraces('position-plot', {
        type: 'scatter3d',
        mode: 'lines+markers',
        x: [],
        y: [],
        z: [],
        marker: {
            size: 2,
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
    
    // Reset 3D plot axis ranges with grid lines
    resetPosition3DPlotScaling();
    
    // Reset other charts
    ['orientation-plot', 'acceleration-plot', 'magnetometer-plot', 'mouse-plot', 'position-2d-plot'].forEach(plot => {
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

function loadRecording(recordingId, keepReferenceTrajectories = false) {
    if (socket.readyState === WebSocket.OPEN) {
        // Store reference trajectories if needed
        const savedReferenceTrajectories = keepReferenceTrajectories ? {...referenceTrajectories} : {};
        
        // Reset current view, preserving reference trajectories if specified
        resetCharts(keepReferenceTrajectories);
        
        // Restore reference trajectories if needed
        if (keepReferenceTrajectories) {
            referenceTrajectories = savedReferenceTrajectories;
        }
        
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
        
    // Reset the first timestamp since we're loading a new recording
    firstTimestamp = null;
    
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

// Function to update 3D position plot with equal axis scaling
function updatePosition3DPlotWithEqualAxes() {
    const plot = document.getElementById('position-plot');
    if (!plot || !plot.data || plot.data.length === 0) return;
    
    // Get all x, y, z points from the current trace
    const xValues = plot.data[0].x;
    const yValues = plot.data[0].y;
    const zValues = plot.data[0].z;
    
    if (!xValues || !xValues.length) return;
    
    // Calculate the maximum distance from origin (0,0,0) in any direction
    const xMaxDistance = Math.max(Math.abs(Math.min(...xValues)), Math.abs(Math.max(...xValues)));
    const yMaxDistance = Math.max(Math.abs(Math.min(...yValues)), Math.abs(Math.max(...yValues)));
    const zMaxDistance = Math.max(Math.abs(Math.min(...zValues)), Math.abs(Math.max(...zValues)));
    
    // Find the maximum distance in any dimension to ensure equal scaling
    let maxDistance = Math.max(xMaxDistance, yMaxDistance, zMaxDistance);
    
    // Add padding (20% of the max distance)
    const padding = maxDistance * 0.2;
    maxDistance += padding;
    
    // Ensure we have some minimum scale even with points very close to origin
    const minScale = 0.1;
    maxDistance = Math.max(maxDistance, minScale);
    
    // Set the new layout with equal axis ranges centered at origin (0,0,0)
    const newLayout = {
        scene: {
            aspectmode: 'cube',
            xaxis: { 
                range: [-maxDistance, maxDistance],
                autorange: false,
                nticks: 10,
                showgrid: true,
                showline: true,
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            yaxis: { 
                range: [-maxDistance, maxDistance],
                autorange: false,
                nticks: 10,
                showgrid: true,
                showline: true,
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            zaxis: { 
                range: [-maxDistance, maxDistance],
                autorange: false,
                nticks: 10,
                showgrid: true,
                showline: true,
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            }
        }
    };
    
    // Update the layout
    Plotly.relayout('position-plot', newLayout);
}

// Function to reset 3D position plot scaling
function resetPosition3DPlotScaling() {
    // Reset to a small default scale with auto-range enabled
    const resetLayout = {
        scene: {
            aspectmode: 'cube',
            xaxis: { 
                autorange: true,
                nticks: 10,
                showgrid: true,
                showline: true,
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            yaxis: { 
                autorange: true,
                nticks: 10,
                showgrid: true,
                showline: true,
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            },
            zaxis: { 
                autorange: true,
                nticks: 10,
                showgrid: true,
                showline: true,
                gridcolor: '#30363d',
                zerolinecolor: '#58a6ff'
            }
        }
    };
    
    Plotly.relayout('position-plot', resetLayout);
}

// Update plotRecordedTrajectory function to maintain equal scaling
function plotRecordedTrajectory(estimations) {
    // Extract data for each chart
    // Convert absolute timestamps to relative (seconds from first timestamp)
    const startTime = estimations.length > 0 ? estimations[0].timestamp : 0;
    const timestamps = estimations.map(e => e.timestamp - startTime);
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
    // Delete all existing traces from the position plot
    const positionPlot = document.getElementById('position-plot');
    if (positionPlot && positionPlot.data) {
        // Delete all traces from position-plot by removing them one by one
        while (positionPlot.data.length > 0) {
            Plotly.deleteTraces('position-plot', positionPlot.data.length - 1);
        }
    }
    
    Plotly.addTraces('position-plot', {
        type: 'scatter3d',
        mode: 'lines+markers',
        x: xPositions,
        y: yPositions,
        z: zPositions,
        marker: {
            size: 2,
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
    
    // Apply equal axis scaling to the loaded trajectory
    updatePosition3DPlotWithEqualAxes();
    
    // Plot position 2D data
    Plotly.deleteTraces('position-2d-plot', [0, 1, 2]);
    Plotly.addTraces('position-2d-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: xPositions,
            name: 'Position X',
            line: { color: '#f85149', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: yPositions,
            name: 'Position Y',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: zPositions,
            name: 'Position Z',
            line: { color: '#58a6ff', width: 2 }
        }
    ]);
    
    // Plot orientation data - converting to degrees
    Plotly.deleteTraces('orientation-plot', [0, 1, 2]);
    Plotly.addTraces('orientation-plot', [
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: rolls.map(r => r * 180 / Math.PI), // Convert rolls to degrees
            name: 'Roll',
            line: { color: '#f85149', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: pitches.map(p => p * 180 / Math.PI), // Convert pitches to degrees
            name: 'Pitch',
            line: { color: '#238636', width: 2 }
        },
        {
            type: 'scatter',
            mode: 'lines',
            x: timestamps,
            y: yaws.map(y => y * 180 / Math.PI), // Convert yaws to degrees
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
        
        // Helper function to safely update text content
        const updateElement = (id, value) => {
            const element = document.getElementById(id);
            if (element) element.textContent = value;
        };
        
        // Position
        updateElement('position-x', lastEstimation.x.toFixed(4));
        updateElement('position-y', lastEstimation.y.toFixed(4));
        updateElement('position-z', lastEstimation.z.toFixed(4));
        // Orientation
        updateElement('orientation-roll', 
            `${lastEstimation.roll.toFixed(4)} rad (${(lastEstimation.roll * 180 / Math.PI).toFixed(2)}°)`);
        updateElement('orientation-pitch', 
            `${lastEstimation.pitch.toFixed(4)} rad (${(lastEstimation.pitch * 180 / Math.PI).toFixed(2)}°)`);
        updateElement('orientation-yaw', 
            `${lastEstimation.yaw.toFixed(4)} rad (${(lastEstimation.yaw * 180 / Math.PI).toFixed(2)}°)`);
        // Acceleration
        updateElement('acceleration-x', `${lastEstimation.acc_x.toFixed(4)} m/s²`);
        updateElement('acceleration-y', `${lastEstimation.acc_y.toFixed(4)} m/s²`);
        updateElement('acceleration-z', `${lastEstimation.acc_z.toFixed(4)} m/s²`);
        // Magnetometer
        updateElement('magnetometer-x', `${lastEstimation.mag_x.toFixed(4)} μT`);
        updateElement('magnetometer-y', `${lastEstimation.mag_y.toFixed(4)} μT`);
        updateElement('magnetometer-z', `${lastEstimation.mag_z.toFixed(4)} μT`);
        updateElement('magnetometer-strength', `${lastEstimation.mag_strength.toFixed(4)} μT`);
        // Mouse integrated
        updateElement('mouse-integrated-x', lastEstimation.mouse_integrated_x.toFixed(4));
        updateElement('mouse-integrated-y', lastEstimation.mouse_integrated_y.toFixed(4));
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
            color: '#ff0000', // Red color for reference trajectory
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

// Reference Trajectory Management Functions
function loadReferenceFromCSV() {
    // For embedded device, display a list of available reference trajectory files
    // instead of using file input dialog which may not work on all devices
    fetch('/api/list_reference_files')
    .then(response => response.json())
    .then(data => {
        if (data.success && data.files) {
            if (data.files.length === 0) {
                showToast('No reference trajectory files found');
                return;
            }
            
            // Display a dialog to select which file to load
            showReferenceFileList(data.files);
        } else {
            showToast(`Error loading reference files: ${data.error || 'Unknown error'}`);
        }
    })
    .catch(error => {
        console.error('Error listing reference files:', error);
        showToast('Error loading reference files from server');
    });
}

function showReferenceFileList(files) {
    // Get reference to dialog element or create one
    let dialog = document.getElementById('reference-file-dialog');
    
    if (!dialog) {
        dialog = document.createElement('div');
        dialog.id = 'reference-file-dialog';
        dialog.className = 'dialog';
        
        dialog.innerHTML = `
            <div class="dialog-content">
                <div class="dialog-header">
                    <h3><i class="bi bi-file-earmark-text"></i> Reference CSV Files</h3>
                    <button id="close-reference-file-dialog" class="close-button">&times;</button>
                </div>
                <div class="dialog-body">
                    <table class="reference-table" style="width: 100%;">
                        <thead>
                            <tr>
                                <th>Name</th>
                                <th>Modified</th>
                                <th>Actions</th>
                            </tr>
                        </thead>
                        <tbody id="reference-file-tbody"></tbody>
                    </table>
                </div>
            </div>
        `;
        
        document.body.appendChild(dialog);
        
        // Add close button event handler
        document.getElementById('close-reference-file-dialog').addEventListener('click', function() {
            dialog.style.display = 'none';
        });
    }
    
    // Get tbody element and clear it
    const tbody = document.getElementById('reference-file-tbody');
    tbody.innerHTML = '';
    
    // Add rows for each file
    files.forEach(file => {
        const row = document.createElement('tr');
        
        // Name cell
        const nameCell = document.createElement('td');
        nameCell.textContent = file.name;
        
        // Modified date cell
        const modifiedCell = document.createElement('td');
        modifiedCell.textContent = file.modified;
        
        // Actions cell
        const actionsCell = document.createElement('td');
        const loadBtn = document.createElement('button');
        loadBtn.textContent = 'Load';
        loadBtn.onclick = function() {
            loadReferenceFile(file.name);
            dialog.style.display = 'none';
        };
        actionsCell.appendChild(loadBtn);
        
        // Add cells to row
        row.appendChild(nameCell);
        row.appendChild(modifiedCell);
        row.appendChild(actionsCell);
        
        // Add row to table
        tbody.appendChild(row);
    });
    
    // Display the dialog
    dialog.style.display = 'block';
}

function loadReferenceFile(filename) {
    fetch(`/api/load_reference_file/${filename}`)
    .then(response => response.json())
    .then(data => {
        if (data.success && data.points) {
            // Add the reference trajectory directly to the UI
            const name = data.name;
            addReferenceTrajectoryToUI(name, data.points);
            showToast(`Loaded reference trajectory: ${name} (${data.points.length} points)`);
            
            // If we have a recording ID, save the reference to the database
            if (currentRecordingId) {
                saveReferenceTrajectoryToDB(name, data.points);
            }
        } else {
            showToast(`Error loading reference file: ${data.error || 'Unknown error'}`);
        }
    })
    .catch(error => {
        console.error('Error loading reference file:', error);
        showToast('Error loading reference file from server');
    });
}

function saveReferenceTrajectoryToDB(name, points) {
    // Save the reference trajectory to the database
    fetch(`/api/save_reference_trajectory/${currentRecordingId}`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            name: name,
            points: points
        })
    })
    .then(response => response.json())
    .catch(error => {
        console.error('Error saving reference trajectory:', error);
    });
}

function showReferenceTrajectoryList(trajectories) {
    // Get reference to dialog element
    const dialog = document.getElementById('reference-dialog');
    const tbody = document.getElementById('reference-tbody');
    
    // Clear existing rows
    tbody.innerHTML = '';
    
    // List all reference trajectories
    for (const [name, points] of Object.entries(trajectories)) {
        const row = document.createElement('tr');
        
        // Name cell
        const nameCell = document.createElement('td');
        nameCell.textContent = name;
        
        // Points cell
        const pointsCell = document.createElement('td');
        pointsCell.textContent = `${points.length} points`;
        
        // Actions cell
        const actionsCell = document.createElement('td');
        const loadBtn = document.createElement('button');
        loadBtn.textContent = 'Load';
        loadBtn.onclick = function() {
            loadReferenceTrajectory(name, points);
        };
        actionsCell.appendChild(loadBtn);
        
        // Add cells to row
        row.appendChild(nameCell);
        row.appendChild(pointsCell);
        row.appendChild(actionsCell);
        
        // Add row to table
        tbody.appendChild(row);
    }
    
    // Add close button event handler if not already set
    const closeBtn = document.getElementById('close-reference-dialog');
    if (!closeBtn.onclick) {
        closeBtn.onclick = function() {
            dialog.style.display = 'none';
        };
    }
    
    // Display the dialog
    dialog.style.display = 'block';
}

function loadReferenceTrajectory(name, points) {
    // Add the selected reference trajectory to the UI
    addReferenceTrajectoryToUI(name, points);
    
    // Close the dialog
    document.getElementById('reference-dialog').style.display = 'none';
}

function addReferenceTrajectoryToUI(name, points) {
    // Store the reference trajectory in our local object
    referenceTrajectories[name] = points;
    
    // Add the reference trajectory to the 3D plot
    addReferenceTrajectory(name, points);
    
    showToast(`Reference trajectory "${name}" loaded with ${points.length} points.`);
}

// Toast notification function
function showToast(message) {
    // Check if toast element exists, create if not
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
    setTimeout(function() { 
        toast.className = toast.className.replace('show', ''); 
    }, 3000);
}

function resetMeasurementsOnly() {
    // Store existing reference trajectories
    const savedReferenceTrajectories = {...referenceTrajectories};
    
    // Reset measurement data
    resetMeasurementData();
    
    // Reset first timestamp
    firstTimestamp = null;
    
    // Clear all plots while keeping reference trajectories
    resetCharts(true);
    
    // Restore the reference trajectories object
    referenceTrajectories = savedReferenceTrajectories;    
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
    const plotIds = ['orientation-plot', 'acceleration-plot', 'magnetometer-plot', 'mouse-plot', 'position-2d-plot'];
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
