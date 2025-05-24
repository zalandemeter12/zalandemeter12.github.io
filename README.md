# Autonomous Racing Simulation

A browser-based autonomous racing simulation using the Stanley controller for path tracking. The simulation demonstrates how a vehicle can autonomously follow a predefined racing line while maintaining speed and minimizing tracking errors.

## Features
- Real-time simulation of vehicle dynamics
- Stanley controller implementation for path tracking
- Live visualization of heading and cross-track errors
- Adjustable parameters for speed and controller gains
- Smooth track generation using cubic splines

## Running the Simulation
1. Open `index.html` in a modern web browser
2. Use the sliders to adjust:
   - Maximum speed
   - Cross track error gain
   - Heading error gain
   - Lookahead factor
3. Click "Reset" to restart the simulation

## Implementation Details
- Vehicle dynamics simulated at 60Hz
- Path tracking using Stanley controller
- Error visualization using Chart.js
- Track boundaries generated with cubic splines