class Renderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
    }

    clear() {
        this.ctx.fillStyle = '#ffffff';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }

    drawVehicle(vehicle) {
        this.ctx.save();
        this.ctx.translate(vehicle.x, vehicle.y);
        this.ctx.rotate(vehicle.heading);
        
        // Draw vehicle body
        this.ctx.fillStyle = '#3498db';
        this.ctx.strokeStyle = '#2980b9';
        this.ctx.lineWidth = 2;
        
        // Draw vehicle shape
        this.ctx.beginPath();
        this.ctx.moveTo(-15, -10);
        this.ctx.lineTo(15, -10);
        this.ctx.lineTo(25, 0);
        this.ctx.lineTo(15, 10);
        this.ctx.lineTo(-15, 10);
        this.ctx.closePath();
        this.ctx.fill();
        this.ctx.stroke();
        
        // Draw direction indicator
        this.ctx.fillStyle = '#2980b9';
        this.ctx.beginPath();
        this.ctx.moveTo(15, -5);
        this.ctx.lineTo(25, 0);
        this.ctx.lineTo(15, 5);
        this.ctx.closePath();
        this.ctx.fill();
        
        this.ctx.restore();
    }

    drawTrack(track) {
        if (!track.splinePoints || track.splinePoints.length < 2) return;

        // Draw track boundaries
        this.ctx.beginPath();
        const firstLeft = track.transformPoint(track.leftBoundary[0].x, track.leftBoundary[0].y);
        this.ctx.moveTo(firstLeft.x, firstLeft.y);
        
        // Draw left boundary
        track.leftBoundary.forEach(point => {
            const transformed = track.transformPoint(point.x, point.y);
            this.ctx.lineTo(transformed.x, transformed.y);
        });
        
        // Draw right boundary in reverse
        [...track.rightBoundary].reverse().forEach(point => {
            const transformed = track.transformPoint(point.x, point.y);
            this.ctx.lineTo(transformed.x, transformed.y);
        });
        
        this.ctx.closePath();
        this.ctx.fillStyle = '#f5f5f5';
        this.ctx.fill();
        this.ctx.strokeStyle = '#95a5a6';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // Draw centerline
        this.ctx.beginPath();
        this.ctx.strokeStyle = '#bdc3c7';
        this.ctx.setLineDash([5, 5]);
        this.ctx.lineWidth = 1;
        
        const firstCenter = track.transformPoint(track.splinePoints[0].x, track.splinePoints[0].y);
        this.ctx.moveTo(firstCenter.x, firstCenter.y);
        
        track.splinePoints.forEach(point => {
            const transformed = track.transformPoint(point.x, point.y);
            this.ctx.lineTo(transformed.x, transformed.y);
        });
        
        this.ctx.stroke();
        this.ctx.setLineDash([]);

        // Draw current target point
        if (track.currentWaypointIndex < track.splinePoints.length) {
            const point = track.splinePoints[track.currentWaypointIndex];
            const transformed = track.transformPoint(point.x, point.y);
            this.ctx.fillStyle = '#e74c3c';
            this.ctx.beginPath();
            this.ctx.arc(transformed.x, transformed.y, 5, 0, 2 * Math.PI);
            this.ctx.fill();
        }
    }

    render(game) {
        this.clear();
        if (game.track) {
            this.drawTrack(game.track);
        }
        if (game.vehicle) {
            this.drawVehicle(game.vehicle);
        }
    }
} 