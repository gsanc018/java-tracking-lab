package models;

import math.Matrix;

public class CV1DModel implements MotionModel {

    private final double q;

    public CV1DModel(double q) {
        this.q = q;
    }

    @Override
    public Matrix getF(double dt) {
        return new Matrix(new double[][] {
                {1.0, dt},
                {0.0, 1.0}
        });
    }

    @Override
    public Matrix getQ(double dt) {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        return new Matrix(new double[][] {
                {dt3/3.0 * q, dt2/2.0 * q},
                {dt2/2.0 * q, dt * q}
        });
    }
}
