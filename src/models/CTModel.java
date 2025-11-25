package models;

import math.Matrix;

public class CTModel implements MotionModel {

    private final double q;      // process noise intensity
    private final double omega;  // fixed turn rate (rad/s)

    public CTModel(double q, double omega) {
        this.q = q;
        this.omega = omega;
    }

    @Override
    public Matrix getF(double dt) {
        double w = omega;
        double wd = w * dt;

        // If turn rate is very small, fall back to CV
        if (Math.abs(w) < 1e-6) {
            return new Matrix(new double[][] {
                    {1.0, 0.0, dt,  0.0},
                    {0.0, 1.0, 0.0, dt },
                    {0.0, 0.0, 1.0, 0.0},
                    {0.0, 0.0, 0.0, 1.0}
            });
        }

        double sin_wd = Math.sin(wd);
        double cos_wd = Math.cos(wd);

        double s_over_w  = sin_wd / w;
        double one_minus_c_over_w = (1.0 - cos_wd) / w;

        return new Matrix(new double[][] {
                {1.0, 0.0,  s_over_w,        -one_minus_c_over_w},
                {0.0, 1.0,  one_minus_c_over_w,  s_over_w      },
                {0.0, 0.0,  cos_wd,          -sin_wd          },
                {0.0, 0.0,  sin_wd,           cos_wd          }
        });
    }

    @Override
    public Matrix getQ(double dt) {
        double dt2 = dt * dt;

        // Simple diagonal process noise as a placeholder
        return new Matrix(new double[][] {
                {q * dt2, 0.0,      0.0,     0.0},
                {0.0,     q * dt2,  0.0,     0.0},
                {0.0,     0.0,      q * dt,  0.0},
                {0.0,     0.0,      0.0,     q * dt}
        });
    }
}
