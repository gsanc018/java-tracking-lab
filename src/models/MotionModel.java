package models;

import math.Matrix;

public interface MotionModel {
    Matrix getF(double dt);
    Matrix getQ(double dt);
}
