package models;

import math.Matrix;

public class CAModel implements MotionModel {

    private final double q;  // process noise intensity

    public CAModel(double q) {
        this.q = q;
    }

    @Override
    public Matrix getF(double dt) {

        double dt2 = dt * dt;
        double half_dt2 = 0.5 * dt2;

        return new Matrix(new double[][] {
                {1, 0, dt, 0,  half_dt2,   0},
                {0, 1,  0, dt, 0,       half_dt2},
                {0, 0,  1, 0,    dt,       0},
                {0, 0,  0, 1,    0,        dt},
                {0, 0,  0, 0,    1,        0},
                {0, 0,  0, 0,    0,        1}
        });
    }

    @Override
    public Matrix getQ(double dt) {

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        return new Matrix(new double[][] {
                {dt5/20*q,   0,          dt4/8*q,    0,         dt3/6*q,   0},
                {0,       dt5/20*q,      0,       dt4/8*q,       0,      dt3/6*q},
                {dt4/8*q,   0,          dt3/3*q,    0,         dt2/2*q,   0},
                {0,       dt4/8*q,       0,       dt3/3*q,       0,      dt2/2*q},
                {dt3/6*q,   0,          dt2/2*q,    0,          dt*q,     0},
                {0,       dt3/6*q,       0,       dt2/2*q,       0,        dt*q}
        });
    }
}
