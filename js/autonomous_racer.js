/**
 * Vehicle class representing the autonomous racing car.
 * Implements vehicle dynamics and the Stanley controller for path tracking.
 */
class Vehicle {
    /**
     * Create a new vehicle instance.
     * @param {number} x - Initial x position in pixels
     * @param {number} y - Initial y position in pixels
     * @param {number} heading - Initial heading angle in radians
     */
    constructor(x, y, heading = 0) {
        // State variables
        this.x = x;          // x position in pixels
        this.y = y;          // y position in pixels
        this.heading = heading;  // yaw angle in radians
        this.vx = 50;        // Initial forward velocity
        this.vy = 0;         // velocity in y direction (local frame) in pixels/s
        this.yawRate = 0;    // angular velocity in radians/s
        this.previousSteeringAngle = 0;  // Previous steering angle for damping
        
        // Vehicle parameters
        this.maxSpeed = 400;  // pixels per second
        this.maxYawRate = Math.PI;  // radians per second
        this.wheelBase = 30;  // pixels
        
        // Stanley controller parameters - matching C++ implementation
        this.kDist = 5.0;   // Cross track error gain (kDist in C++)
        this.kAng = 5.0;    // Heading error gain (kAng in C++)
        this.kSoft = 1.0;    // Softening constant
        this.kDamp = 1.0;    // Damping gain for velocity scaling
        this.kYaw = 0.05;    // Yaw rate error gain
        this.kSteer = 0.0;   // Steering angle error gain
        this.kAG = 0.0;      // Steady state heading error gain
        
        // Error tracking
        this.currentCrossTrackError = 0;
        this.currentHeadingError = 0;
        
        // Simulation parameters
        this.dt = 1/60;  // Back to 60 FPS
    }

    reset() {
        // Reset dynamic state variables
        this.vx = 50;
        this.vy = 0;
        this.yawRate = 0;
    }

    // Normalize angle to [-π, π]
    normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    update() {
        // Update position using the motion model
        this.x += this.vx * Math.cos(this.heading) * this.dt;
        this.y += this.vx * Math.sin(this.heading) * this.dt;
        this.heading = this.normalizeAngle(this.heading + this.yawRate * this.dt);
        
        // Ensure minimum speed is maintained
        const minSpeed = this.maxSpeed * 0.1;
        this.vx = Math.max(minSpeed, this.vx * 0.99);
    }

    // Stanley controller implementation
    control(targetPoint) {
        // Calculate cross track error using vector projection
        const dx = targetPoint.x - this.x;
        const dy = targetPoint.y - this.y;
        
        // Get path heading and direction vector
        const pathHeading = targetPoint.orientation || Math.atan2(dy, dx);
        const pathDirX = Math.cos(pathHeading);
        const pathDirY = Math.sin(pathHeading);
        
        // Cross track error using vector projection (matching C++ implementation)
        this.currentCrossTrackError = pathDirX * dy - pathDirY * dx;
        
        // Heading error (matching C++ implementation)
        this.currentHeadingError = -this.normalizeAngle(this.heading - pathHeading);  // Negated the heading error
        
        // Calculate trajectory curvature (approximate from path)
        const curvature = targetPoint.curvature || 0;
        const trajectoryYawRate = this.vx * curvature;
        const headingErrorSS = this.kAG * this.vx * trajectoryYawRate;
        
        // Stanley control law (matching C++ implementation)
        let steeringAngle = this.kAng * (this.currentHeadingError - headingErrorSS);
        
        // Cross track error term
        steeringAngle += Math.atan2(this.kDist * this.currentCrossTrackError, 
                                   this.kSoft + this.kDamp * Math.abs(this.vx));
        
        // Yaw rate error term
        steeringAngle += this.kYaw * (this.yawRate - trajectoryYawRate);
        
        // Steering angle error term (only when moving)
        if (Math.abs(this.vx) > 0.25) {
            steeringAngle += this.kSteer * (this.previousSteeringAngle - this.yawRate);
        }
        
        // Limit steering angle
        steeringAngle = Math.max(-this.maxYawRate, Math.min(this.maxYawRate, steeringAngle));
        
        // Update previous steering angle
        this.previousSteeringAngle = this.yawRate;
        
        // Set yaw rate based on steering angle and velocity
        this.yawRate = (this.vx / this.wheelBase) * Math.tan(steeringAngle);
        
        // Maintain constant velocity
        this.vx = this.maxSpeed;
        
        return Math.hypot(dx, dy);
    }
}

/**
 * Track class handling track generation and path following logic.
 * Uses cubic splines for smooth track boundaries and centerline.
 */
class Track {
    /**
     * Create a new track instance.
     * @param {HTMLCanvasElement} canvas - Canvas element for dimension reference
     */
    constructor(canvas) {
        this.canvas = canvas;
        this.waypoints = [];
        this.splinePoints = [];
        this.leftBoundary = [];
        this.rightBoundary = [];
        this.trackWidth = 8;
        this.currentWaypointIndex = 0;
        this.isLoaded = false;
        
        // Scale and offset parameters for track visualization
        this.scale = 40;
        this.offsetX = this.canvas.width / 2;
        this.offsetY = this.canvas.height / 2;

        // Lookahead parameters
        this.minLookahead = 30;
        this.lookaheadFactor = 0.0;  // Changed from 1.0 to 0.0
    }

    async initialize() {
        await this.loadTrackFromCSV('tracks/default.csv');
        this.isLoaded = true;
    }

    async loadTrackFromCSV(filename) {
        try {
            const response = await fetch(filename);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.text();
            this.loadTrackFromData(data);
        } catch (error) {
            console.error('Error loading track:', error);
            // Create a simple default track
            const defaultTrack = [
                [0, 0], [2, 0], [4, 1], [5, 2], [5, 4],
                [4, 5], [2, 6], [0, 6], [-2, 6], [-4, 5],
                [-5, 4], [-5, 2], [-4, 1], [-2, 0], [0, 0]
            ].map(([x, y]) => `${x},${y}`).join('\n');
            this.loadTrackFromData(defaultTrack);
        }
    }

    loadTrackFromData(data) {
        // Clear existing track data
        this.waypoints = [];
        this.splinePoints = [];
        this.leftBoundary = [];
        this.rightBoundary = [];
        
        // Parse the track data
        const lines = data.trim().split('\n');
        this.waypoints = lines.map(line => {
            const [x, y] = line.split(',').map(Number);
            return { x, y };
        });

        // Process track in sequence
        this.smoothCenterline();
        this.generateSplinePoints();
        this.generateTrackBoundaries();
        this.adjustTrackTransform();
    }

    smoothCenterline() {
        if (this.waypoints.length < 3) return;

        const points = [...this.waypoints];
        const n = points.length;
        const smoothed = [];
        const windowSize = 5;  // Use 5 points for smoothing
        const halfWindow = Math.floor(windowSize / 2);

        // Add padding points for smooth wrapping
        const paddedPoints = [
            ...points.slice(-halfWindow),
            ...points,
            ...points.slice(0, halfWindow)
        ];

        // Apply Gaussian smoothing
        for (let i = 0; i < n; i++) {
            let sumX = 0, sumY = 0;
            let totalWeight = 0;

            for (let j = -halfWindow; j <= halfWindow; j++) {
                const weight = Math.exp(-0.3 * (j * j));
                const point = paddedPoints[i + halfWindow + j];
                sumX += point.x * weight;
                sumY += point.y * weight;
                totalWeight += weight;
            }

            smoothed.push({
                x: sumX / totalWeight,
                y: sumY / totalWeight
            });
        }

        // Update waypoints with smoothed points
        this.waypoints = smoothed;
        // Ensure the track is closed
        this.waypoints.push({...this.waypoints[0]});
    }

    generateSplinePoints() {
        if (this.waypoints.length < 2) return;

        try {
            const n = this.waypoints.length - 1;  // Last point is duplicate of first
            const x = this.waypoints.map(p => p.x);
            const y = this.waypoints.map(p => p.y);

            // Create splines for x and y coordinates
            const splineX = new CubicSpline(Array.from({length: n + 1}, (_, i) => i), x);
            const splineY = new CubicSpline(Array.from({length: n + 1}, (_, i) => i), y);

            // Sample points along the spline
            const numSamples = n * 10;  // 10 points between each waypoint
            this.splinePoints = [];

            for (let i = 0; i <= numSamples; i++) {
                const param = (i * n) / numSamples;
                this.splinePoints.push({
                    x: splineX.at(param),
                    y: splineY.at(param)
                });
            }
        } catch (error) {
            console.error('Error creating spline:', error);
            this.splinePoints = [...this.waypoints];
        }
    }

    generateTrackBoundaries() {
        if (this.splinePoints.length < 2) return;

        const halfWidth = this.trackWidth / 2;
        this.leftBoundary = [];
        this.rightBoundary = [];

        // Add last point to beginning for smooth start
        const points = [
            this.splinePoints[this.splinePoints.length - 1],
            ...this.splinePoints,
            this.splinePoints[0]
        ];

        // Generate boundaries using 3 points for smooth normals
        for (let i = 1; i < points.length - 1; i++) {
            const prev = points[i - 1];
            const curr = points[i];
            const next = points[i + 1];
            
            // Calculate average direction vector
            const dx1 = curr.x - prev.x;
            const dy1 = curr.y - prev.y;
            const dx2 = next.x - curr.x;
            const dy2 = next.y - curr.y;
            
            // Average the vectors
            const dx = (dx1 + dx2) / 2;
            const dy = (dy1 + dy2) / 2;
            const length = Math.sqrt(dx * dx + dy * dy);
            
            if (length === 0) continue;
            
            // Calculate normal vector
            const normalX = -dy / length;
            const normalY = dx / length;
            
            // Add boundary points
            this.leftBoundary.push({
                x: curr.x + normalX * halfWidth,
                y: curr.y + normalY * halfWidth
            });
            
            this.rightBoundary.push({
                x: curr.x - normalX * halfWidth,
                y: curr.y - normalY * halfWidth
            });
        }

        // Close the boundaries by copying first point
        this.leftBoundary.push({...this.leftBoundary[0]});
        this.rightBoundary.push({...this.rightBoundary[0]});
    }

    adjustTrackTransform() {
        // Find track bounds
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        
        this.waypoints.forEach(point => {
            minX = Math.min(minX, point.x);
            maxX = Math.max(maxX, point.x);
            minY = Math.min(minY, point.y);
            maxY = Math.max(maxY, point.y);
        });
        
        // Calculate scale to fit track in canvas with padding
        const padding = 50;
        const trackWidth = maxX - minX;
        const trackHeight = maxY - minY;
        const scaleX = (this.canvas.width - 2 * padding) / trackWidth;
        const scaleY = (this.canvas.height - 2 * padding) / trackHeight;
        
        // Use the smaller scale to maintain aspect ratio
        this.scale = Math.min(scaleX, scaleY);
        
        // Center the track
        const centerX = (minX + maxX) / 2;
        const centerY = (minY + maxY) / 2;
        this.offsetX = this.canvas.width / 2 - centerX * this.scale;
        this.offsetY = this.canvas.height / 2 - centerY * this.scale;
    }

    transformPoint(x, y) {
        return {
            x: x * this.scale + this.offsetX,
            y: y * this.scale + this.offsetY
        };
    }

    update(vehicle) {
        // Calculate desired lookahead distance based on speed and factor
        const speedFactor = vehicle.vx / vehicle.maxSpeed;
        const baseLookahead = this.minLookahead + (speedFactor * 50);
        const lookaheadDistance = baseLookahead * this.lookaheadFactor;
        
        // Ensure we have points to work with
        if (!this.splinePoints || this.splinePoints.length < 2) return;

        // Find the closest point on spline to start search
        let minDist = Infinity;
        let closestIdx = 0;
        for (let i = 0; i < this.splinePoints.length; i++) {
            const point = this.splinePoints[i];
            const transformed = this.transformPoint(point.x, point.y);
            const dist = Math.hypot(transformed.x - vehicle.x, transformed.y - vehicle.y);
            if (dist < minDist) {
                minDist = dist;
                closestIdx = i;
            }
        }

        // Create a continuous array of points for lookahead
        const numPoints = this.splinePoints.length;
        const beforePoints = this.splinePoints.slice(Math.max(0, closestIdx - Math.floor(numPoints/4)));
        const afterPoints = this.splinePoints.slice(0, Math.min(Math.floor(numPoints/2), numPoints));
        const extendedPoints = [...beforePoints, ...this.splinePoints, ...afterPoints];
        
        // Adjust closestIdx for the extended array
        const offset = Math.max(0, closestIdx - Math.floor(numPoints/4));
        const adjustedClosestIdx = closestIdx - offset;
        
        // Search forward from closest point to find target point at lookahead distance
        let targetIdx = adjustedClosestIdx;
        let accumulatedDist = 0;
        let prevPoint = this.transformPoint(
            extendedPoints[adjustedClosestIdx].x,
            extendedPoints[adjustedClosestIdx].y
        );
        
        // Search through extended points
        for (let i = adjustedClosestIdx + 1; i < extendedPoints.length - 1; i++) {
            const nextPoint = this.transformPoint(
                extendedPoints[i].x,
                extendedPoints[i].y
            );
            
            const segmentDist = Math.hypot(
                nextPoint.x - prevPoint.x,
                nextPoint.y - prevPoint.y
            );
            
            if (accumulatedDist + segmentDist >= lookaheadDistance) {
                targetIdx = i;
                break;
            }
            
            accumulatedDist += segmentDist;
            prevPoint = nextPoint;
            targetIdx = i;
        }
        
        // Get target point and calculate orientation using three points for smoothness
        const targetPoint = this.transformPoint(
            extendedPoints[targetIdx].x,
            extendedPoints[targetIdx].y
        );

        // Get points before and after target for smooth orientation and curvature
        const prevIdx = Math.max(0, targetIdx - 1);
        const nextIdx = Math.min(extendedPoints.length - 1, targetIdx + 1);
        
        const prevTransformed = this.transformPoint(
            extendedPoints[prevIdx].x,
            extendedPoints[prevIdx].y
        );
        
        const nextTransformed = this.transformPoint(
            extendedPoints[nextIdx].x,
            extendedPoints[nextIdx].y
        );

        // Calculate orientation using three points for smoothness
        const dx1 = targetPoint.x - prevTransformed.x;
        const dy1 = targetPoint.y - prevTransformed.y;
        const dx2 = nextTransformed.x - targetPoint.x;
        const dy2 = nextTransformed.y - targetPoint.y;
        
        // Average the vectors for smooth orientation
        const dx = (dx1 + dx2) / 2;
        const dy = (dy1 + dy2) / 2;
        targetPoint.orientation = Math.atan2(dy, dx);

        // Calculate approximate curvature using three points
        const a = Math.hypot(dx1, dy1);  // Distance to prev point
        const b = Math.hypot(dx2, dy2);  // Distance to next point
        const c = Math.hypot(nextTransformed.x - prevTransformed.x, 
                           nextTransformed.y - prevTransformed.y);  // Distance prev to next
        
        // Use Menger curvature formula if points form a valid triangle
        if (a > 0 && b > 0 && c > 0) {
            const s = (a + b + c) / 2;  // Semi-perimeter
            const area = Math.sqrt(s * (s - a) * (s - b) * (s - c));  // Triangle area
            targetPoint.curvature = 4 * area / (a * b * c);  // Menger curvature
            
            // Determine sign of curvature based on cross product
            const cross = dx1 * dy2 - dy1 * dx2;
            if (cross < 0) targetPoint.curvature = -targetPoint.curvature;
        } else {
            targetPoint.curvature = 0;
        }
        
        // Map back to original index for track display
        this.currentWaypointIndex = ((closestIdx + (targetIdx - adjustedClosestIdx)) % numPoints + numPoints) % numPoints;
        
        // Update control
        vehicle.control(targetPoint);
    }
}

/**
 * Main game class handling simulation, rendering, and UI.
 */
class Game {
    constructor() {
        // Initialize canvas and renderer
        this.canvas = document.getElementById('gameCanvas');
        this.renderer = new Renderer(this.canvas);
        
        // Simulation parameters
        this.fixedDt = 1/60;     // Fixed 60Hz simulation
        this.accumulator = 0;     // Time accumulator for fixed timestep
        this.lastTime = null;     // Last frame timestamp
        this.maxFrameTime = 0.25; // Maximum time step (prevents spiral of death)
        this.isRunning = false;   // Simulation running state
        this.frameCount = 0;      // Frame counter
        this.lastFrameTime = null;
        this.simulationTime = 0;
        
        // Initialize game objects
        this.track = new Track(this.canvas);
        this.initializeGame();
        
        // Setup UI
        this.setupResetButton();
        this.initializeChart();

        // Chart data
        this.velocityData = [];
        this.maxDataPoints = 600; // 10 seconds at 60Hz
    }

    initializeChart() {
        // Initialize heading error chart
        const headingCtx = document.getElementById('headingChart').getContext('2d');
        this.headingChart = new Chart(headingCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Heading Error (rad)',
                    data: [],
                    borderColor: '#3498db',
                    tension: 0.4,
                    borderWidth: 2,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                animation: false,
                scales: {
                    y: {
                        beginAtZero: false,
                        grace: '10%',
                        ticks: {
                            callback: function(value) {
                                return value.toFixed(2);
                            }
                        }
                    },
                    x: {
                        display: false  // Hide x-axis completely
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top'
                    }
                }
            }
        });

        // Initialize cross track error chart
        const crossTrackCtx = document.getElementById('crossTrackChart').getContext('2d');
        this.crossTrackChart = new Chart(crossTrackCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Cross Track Error (px)',
                    data: [],
                    borderColor: '#e74c3c',
                    tension: 0.4,
                    borderWidth: 2,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                animation: false,
                scales: {
                    y: {
                        beginAtZero: false,
                        grace: '10%',
                        ticks: {
                            callback: function(value) {
                                return value.toFixed(1);
                            }
                        }
                    },
                    x: {
                        display: false  // Hide x-axis completely
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top'
                    }
                }
            }
        });

        // Initialize error data arrays and time tracking
        this.headingErrors = [];
        this.crossTrackErrors = [];
        this.errorTimes = [];  // Store time values for x-axis
        this.maxDataPoints = 600;  // 10 seconds at 60Hz
    }

    updateChart() {
        if (!this.headingChart || !this.crossTrackChart || !this.vehicle || !this.track) return;

        // Get errors directly from vehicle
        const headingError = this.vehicle.currentHeadingError;
        const crossTrackError = this.vehicle.currentCrossTrackError;

        // Add new data points
        this.headingErrors.push(headingError);
        this.crossTrackErrors.push(crossTrackError);

        // Keep only the last 600 points (10 seconds at 60fps)
        if (this.headingErrors.length > 600) {
            this.headingErrors = this.headingErrors.slice(-600);
            this.crossTrackErrors = this.crossTrackErrors.slice(-600);
        }

        // Update heading error chart
        this.headingChart.data.labels = Array(this.headingErrors.length).fill('');
        this.headingChart.data.datasets[0].data = this.headingErrors;
        this.headingChart.update('none');

        // Update cross track error chart
        this.crossTrackChart.data.labels = Array(this.crossTrackErrors.length).fill('');
        this.crossTrackChart.data.datasets[0].data = this.crossTrackErrors;
        this.crossTrackChart.update('none');
    }

    setupResetButton() {
        const resetButton = document.getElementById('resetButton');
        resetButton.addEventListener('click', () => this.resetSimulation());
    }

    async initializeGame() {
        try {
            // Wait for track to load
            await this.track.initialize();
            
            // Now initialize vehicle
            this.initializeVehicle();
            
            // Start game loop
            this.isRunning = true;
            this.lastTime = null;  // Reset time tracking
            this.lastFrameTime = null;
            this.frameCount = 0;
            this.accumulator = 0;
            requestAnimationFrame(this.gameLoop);
        } catch (error) {
            console.error('Error in game initialization:', error);
        }
    }

    initializeVehicle() {
        // Only initialize if track is loaded
        if (!this.track.isLoaded || !this.track.waypoints || this.track.waypoints.length === 0) {
            console.error('Track not loaded properly');
            return;
        }

        // Reset waypoint index
        this.track.currentWaypointIndex = 0;
        
        // Get the first waypoint for position
        const startX = this.track.waypoints[0].x * this.track.scale + this.track.offsetX;
        const startY = this.track.waypoints[0].y * this.track.scale + this.track.offsetY;
        
        // Get the next waypoint for heading
        const nextPoint = this.track.waypoints[1];
        const nextX = nextPoint.x * this.track.scale + this.track.offsetX;
        const nextY = nextPoint.y * this.track.scale + this.track.offsetY;
        
        // Calculate initial heading
        const heading = Math.atan2(nextY - startY, nextX - startX);
        
        // Create vehicle with initial velocity
        this.vehicle = new Vehicle(startX, startY, heading);
        
        // Setup parameter controls
        this.setupControls();
    }

    setupControls() {
        // Speed control
        const speedSlider = document.querySelector('.controls input[type="range"]:nth-child(2)');
        speedSlider.value = this.vehicle.maxSpeed;
        document.getElementById('speedValue').textContent = this.vehicle.maxSpeed;
        speedSlider.addEventListener('input', (e) => {
            this.vehicle.maxSpeed = parseFloat(e.target.value);
            document.getElementById('speedValue').textContent = e.target.value;
        });
        
        // Cross track error gain
        const crossSlider = document.querySelector('.controls div:nth-child(2) input');
        crossSlider.value = this.vehicle.kDist;
        document.getElementById('crossValue').textContent = this.vehicle.kDist;
        crossSlider.addEventListener('input', (e) => {
            this.vehicle.kDist = parseFloat(e.target.value);
            document.getElementById('crossValue').textContent = e.target.value;
        });
        
        // Heading gain
        const headingSlider = document.querySelector('.controls div:nth-child(3) input');
        headingSlider.value = this.vehicle.kAng;
        document.getElementById('headingValue').textContent = this.vehicle.kAng;
        headingSlider.addEventListener('input', (e) => {
            this.vehicle.kAng = parseFloat(e.target.value);
            document.getElementById('headingValue').textContent = e.target.value;
        });

        // Add lookahead control if it doesn't exist
        const controlsDiv = document.querySelector('.controls');
        const existingLookahead = document.getElementById('lookaheadDiv');
        
        if (!existingLookahead) {
            const lookaheadDiv = document.createElement('div');
            lookaheadDiv.id = 'lookaheadDiv';
            lookaheadDiv.style.marginBottom = '10px';
            
            const lookaheadLabel = document.createElement('label');
            lookaheadLabel.textContent = 'Lookahead Factor: ';
            lookaheadLabel.style.display = 'inline-block';
            lookaheadLabel.style.width = '150px';
            
            const lookaheadValue = document.createElement('span');
            lookaheadValue.id = 'lookaheadValue';
            lookaheadValue.textContent = '0.0';
            lookaheadValue.style.marginLeft = '10px';
            
            const lookaheadSlider = document.createElement('input');
            lookaheadSlider.type = 'range';
            lookaheadSlider.min = '0.0';
            lookaheadSlider.max = '2.0';
            lookaheadSlider.step = '0.1';
            lookaheadSlider.value = '0.0';
            lookaheadSlider.addEventListener('input', (e) => {
                this.track.lookaheadFactor = parseFloat(e.target.value);
                lookaheadValue.textContent = e.target.value;
            });
            
            lookaheadDiv.appendChild(lookaheadLabel);
            lookaheadDiv.appendChild(lookaheadSlider);
            lookaheadDiv.appendChild(lookaheadValue);
            
            // Add to controls div
            controlsDiv.appendChild(lookaheadDiv);
            
            // Move reset button to the end
            const resetButton = document.getElementById('resetButton');
            if (resetButton) {
                resetButton.remove();
                controlsDiv.appendChild(resetButton);
            }
        } else {
            // Update existing lookahead slider
            const lookaheadSlider = existingLookahead.querySelector('input');
            lookaheadSlider.value = '0.0';
            document.getElementById('lookaheadValue').textContent = '0.0';
            this.track.lookaheadFactor = 0.0;
        }
    }

    simulationStep() {
        if (!this.track || !this.vehicle) return;

        // Increment frame counter by 1 each step
        this.simulationTime++;

        // First update controller
        this.track.update(this.vehicle);
        // Then update vehicle physics
        this.vehicle.update();
        // Update velocity chart
        this.updateChart();
    }

    gameLoop = (currentTime) => {
        currentTime = currentTime / 1000;

        if (this.lastTime === null) {
            this.lastTime = currentTime;
            this.lastFrameTime = currentTime;
            requestAnimationFrame(this.gameLoop);
            return;
        }

        try {
            const frameTime = Math.min(currentTime - this.lastTime, this.maxFrameTime);
            this.lastTime = currentTime;
            
            if (frameTime > 0) {
                this.accumulator += frameTime;
                
                let stepsTaken = 0;
                while (this.accumulator >= this.fixedDt && stepsTaken < 10) {
                    this.simulationStep();
                    this.accumulator -= this.fixedDt;
                    stepsTaken++;
                }

                if (stepsTaken >= 10) {
                    console.warn('Simulation falling behind, resetting accumulator');
                    this.accumulator = 0;
                }
            }
            
            // Use renderer to draw the scene
            this.renderer.render(this);
            
        } catch (error) {
            this.accumulator = 0;
        }
        
        if (this.isRunning) {
            requestAnimationFrame(this.gameLoop);
        }
    }

    resetSimulation() {
        try {
            // Store current parameters
            const currentParams = {
                maxSpeed: this.vehicle.maxSpeed,
                kDist: this.vehicle.kDist,
                kAng: this.vehicle.kAng,
                lookaheadFactor: this.track.lookaheadFactor
            };

            // Reset track state
            this.track.currentWaypointIndex = 0;
            
            // Re-initialize vehicle at start position
            this.initializeVehicle();

            // Restore parameters
            this.vehicle.maxSpeed = currentParams.maxSpeed;
            this.vehicle.kDist = currentParams.kDist;
            this.vehicle.kAng = currentParams.kAng;
            this.track.lookaheadFactor = currentParams.lookaheadFactor;

            // Reset error arrays
            this.headingErrors = [];
            this.crossTrackErrors = [];

            // Clear charts
            this.headingChart.data.labels = [];
            this.headingChart.data.datasets[0].data = [];
            this.crossTrackChart.data.labels = [];
            this.crossTrackChart.data.datasets[0].data = [];
            this.headingChart.update('none');
            this.crossTrackChart.update('none');

            // Update UI to reflect preserved values
            document.querySelector('.controls input[type="range"]:nth-child(2)').value = currentParams.maxSpeed;
            document.getElementById('speedValue').textContent = currentParams.maxSpeed;
            
            document.querySelector('.controls div:nth-child(2) input').value = currentParams.kDist;
            document.getElementById('crossValue').textContent = currentParams.kDist;
            
            document.querySelector('.controls div:nth-child(3) input').value = currentParams.kAng;
            document.getElementById('headingValue').textContent = currentParams.kAng;
            
            const lookaheadSlider = document.getElementById('lookaheadDiv').querySelector('input');
            lookaheadSlider.value = currentParams.lookaheadFactor;
            document.getElementById('lookaheadValue').textContent = currentParams.lookaheadFactor;

            // Reset simulation time
            this.lastTime = null;
            this.lastFrameTime = null;
            this.accumulator = 0;
            this.frameCount = 0;
            
            // Ensure simulation is running
            if (!this.isRunning) {
                this.isRunning = true;
                requestAnimationFrame(this.gameLoop);
            }
            
        } catch (error) {
            console.error('Error in reset:', error);
        }
    }
}

// Initialize game when window loads
window.onload = () => {
    window.game = new Game();
}; 