package models;

import math.Matrix;
import math.Vector;

public class CTEKFModel implements NonlinearMotionModel {

    private final double q;   // process noise intensity

    public CTEKFModel(double q) {
        this.q = q;
    }

    // Build a 5x5 transition matrix for given omega, dt
    private Matrix F_from_omega(double omega, double dt) {
        double w  = omega;
        double wd = w * dt;

        // If turn rate is very small, fall back to CV
        if (Math.abs(w) < 1e-6) {
            return new Matrix(new double[][] {
                    {1.0, 0.0, dt,  0.0, 0.0},
                    {0.0, 1.0, 0.0, dt,  0.0},
                    {0.0, 0.0, 1.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 1.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 1.0}
            });
        }

        double sin_wd = Math.sin(wd);
        double cos_wd = Math.cos(wd);

        double s_over_w           = sin_wd / w;
        double one_minus_c_over_w = (1.0 - cos_wd) / w;

        return new Matrix(new double[][] {
                {1.0, 0.0,  s_over_w,            -one_minus_c_over_w, 0.0},
                {0.0, 1.0,  one_minus_c_over_w,   s_over_w,           0.0},
                {0.0, 0.0,  cos_wd,              -sin_wd,             0.0},
                {0.0, 0.0,  sin_wd,               cos_wd,             0.0},
                {0.0, 0.0,  0.0,                  0.0,                1.0}
        });
    }

    @Override
    public Vector f(Vector x, double dt) {
        double omega = x.get(4);
        Matrix F = F_from_omega(omega, dt);
        return F.multiply(x);
    }

    @Override
    public Matrix F(Vector x, double dt) {
        // Numerical Jacobian via finite differences
        int n = x.size();
        Matrix J = new Matrix(n, n);

        Vector fx = f(x, dt);
        double eps = 1e-5;

        for (int j = 0; j < n; j++) {
            Vector xPert = new Vector(n);
            for (int i = 0; i < n; i++) {
                xPert.set(i, x.get(i));
            }
            xPert.set(j, x.get(j) + eps);

            Vector fPert = f(xPert, dt);

            for (int i = 0; i < n; i++) {
                double deriv = (fPert.get(i) - fx.get(i)) / eps;
                J.set(i, j, deriv);
            }
        }

        return J;
    }

    @Override
    public Matrix Q(double dt) {
        double dt2 = dt * dt;

        // Very simple diagonal Q: tune later
        return new Matrix(new double[][] {
                {q * dt2, 0.0,      0.0,     0.0,     0.0},
                {0.0,     q * dt2,  0.0,     0.0,     0.0},
                {0.0,     0.0,      q * dt,  0.0,     0.0},
                {0.0,     0.0,      0.0,     q * dt,  0.0},
                {0.0,     0.0,      0.0,     0.0,     q * dt}
        });
    }
}
