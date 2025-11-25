package models;

import math.Matrix;

public class CVModel implements MotionModel {

    private final double q;  // process noise intensity

    public CVModel(double q) {
        this.q = q;
    }

    @Override
    public Matrix getF(double dt) {
        return new Matrix(new double[][] {
                {1, 0, dt, 0},
                {0, 1, 0, dt},
                {0, 0, 1,  0},
                {0, 0, 0,  1}
        });
    }

    @Override
    public Matrix getQ(double dt) {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        return new Matrix(new double[][] {
                {dt4/4 * q,     0,        dt3/2 * q,    0},
                {0,        dt4/4 * q,      0,       dt3/2 * q},
                {dt3/2 * q,     0,         dt2 * q,     0},
                {0,        dt3/2 * q,      0,       dt2 * q}
        });
    }
}
