package models;

import math.Matrix;
import math.Vector;

public interface NonlinearMotionModel {
    Vector f(Vector x, double dt);      // nonlinear state propagation
    Matrix F(Vector x, double dt);      // Jacobian of f wrt x
    Matrix Q(double dt);                // process noise
}
