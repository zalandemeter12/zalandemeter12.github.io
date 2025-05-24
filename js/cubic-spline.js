class CubicSpline {
    constructor(xs, ys) {
        if (xs.length !== ys.length) {
            throw new Error('xs and ys must have the same length');
        }
        
        this.xs = xs;
        this.ys = ys;
        this.n = xs.length;
        
        // Calculate coefficients
        this.calculateCoefficients();
    }
    
    calculateCoefficients() {
        const n = this.n - 1;
        
        // Calculate h (differences in x)
        const h = new Array(n);
        for (let i = 0; i < n; i++) {
            h[i] = this.xs[i + 1] - this.xs[i];
        }
        
        // Calculate alpha
        const alpha = new Array(n);
        for (let i = 1; i < n; i++) {
            alpha[i] = 3/h[i] * (this.ys[i + 1] - this.ys[i]) - 3/h[i-1] * (this.ys[i] - this.ys[i-1]);
        }
        
        // Calculate coefficients l, mu, and z
        const l = new Array(n + 1);
        const mu = new Array(n + 1);
        const z = new Array(n + 1);
        
        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;
        
        for (let i = 1; i < n; i++) {
            l[i] = 2 * (this.xs[i+1] - this.xs[i-1]) - h[i-1] * mu[i-1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
        }
        
        l[n] = 1;
        z[n] = 0;
        
        // Calculate coefficients b, c, and d
        this.c = new Array(n + 1);
        this.b = new Array(n);
        this.d = new Array(n);
        
        this.c[n] = 0;
        
        for (let j = n - 1; j >= 0; j--) {
            this.c[j] = z[j] - mu[j] * this.c[j + 1];
            this.b[j] = (this.ys[j + 1] - this.ys[j]) / h[j] - h[j] * (this.c[j + 1] + 2 * this.c[j]) / 3;
            this.d[j] = (this.c[j + 1] - this.c[j]) / (3 * h[j]);
        }
    }
    
    at(x) {
        // Find the interval that contains x
        let i = 0;
        while (i < this.n - 1 && x > this.xs[i + 1]) {
            i++;
        }
        
        // Calculate the relative x position
        const dx = x - this.xs[i];
        
        // Calculate the value using the cubic polynomial
        return this.ys[i] + 
               this.b[i] * dx + 
               this.c[i] * dx * dx + 
               this.d[i] * dx * dx * dx;
    }
} 