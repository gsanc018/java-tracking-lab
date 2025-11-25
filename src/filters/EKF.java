package filters;

import math.Matrix;
import math.Vector;
import models.NonlinearMotionModel;

public class EKF implements Filter {

    private Vector x;
    private Matrix P;
    private final NonlinearMotionModel model;
    private final Matrix H;
    private final Matrix R;

    public EKF(Vector x0,
               Matrix P0,
               NonlinearMotionModel model,
               Matrix H,
               Matrix R) {

        this.x = x0;
        this.P = P0;
        this.model = model;
        this.H = H;
        this.R = R;
    }

    @Override
    public void predict(double dt) {
        // Nonlinear propagation
        Vector xPred = model.f(x, dt);
        Matrix F = model.F(x, dt);
        Matrix Q = model.Q(dt);

        x = xPred;

        Matrix FP = F.multiply(P);
        Matrix Ft = F.transpose();
        P = FP.multiply(Ft).add(Q);
    }

    @Override
    public void update(Vector z) {
        // y = z - Hx
        Vector Hx = H.multiply(x);
        Vector y = z.subtract(Hx);

        // S = H P H' + R
        Matrix HP = H.multiply(P);
        Matrix Ht = H.transpose();
        Matrix S = HP.multiply(Ht).add(R);

        // K = P H' S^{-1}
        Matrix PHt = P.multiply(Ht);
        Matrix S_inv = S.inverse();
        Matrix K = PHt.multiply(S_inv);

        // x = x + K y
        Vector Ky = K.multiply(y);
        x = x.add(Ky);

        // P = (I - K H) P
        Matrix KH = K.multiply(H);
        Matrix I = Matrix.identity(P.rows());
        Matrix IminusKH = I.add(KH.multiply(-1.0));
        P = IminusKH.multiply(P);
    }

    @Override
    public Vector getState() {
        return x;
    }

    public Matrix getCovariance() {
        return P;
    }
}
