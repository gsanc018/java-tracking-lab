package models;

import math.Matrix;

public class CA1DModel implements MotionModel {

    private final double q;

    public CA1DModel(double q) {
        this.q = q;
    }

    @Override
    public Matrix getF(double dt) {
        double dt2 = dt * dt;
        return new Matrix(new double[][] {
                {1.0, dt, 0.5 * dt2},
                {0.0, 1.0, dt},
                {0.0, 0.0, 1.0}
        });
    }

    @Override
    public Matrix getQ(double dt) {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        return new Matrix(new double[][] {
                {dt5/20.0 * q, dt4/8.0 * q, dt3/6.0 * q},
                {dt4/8.0 * q,  dt3/3.0 * q, dt2/2.0 * q},
                {dt3/6.0 * q,  dt2/2.0 * q, dt * q}
        });
    }
}
