package filters;

import math.Matrix;
import math.Vector;
import models.MotionModel;

public class KalmanFilter implements Filter {

    private Vector x;           // State vector
    private Matrix P;           // Covariance matrix
    private MotionModel model;  // Motion model (CV, CA, CT, etc.)
    private Matrix H;           // Measurement matrix
    private Matrix R;           // Measurement noise

    public KalmanFilter(Vector initialState,
                        Matrix initialCovariance,
                        MotionModel model,
                        Matrix H,
                        Matrix R) {

        this.x = initialState;
        this.P = initialCovariance;
        this.model = model;
        this.H = H;
        this.R = R;
    }

    @Override
    public void predict(double dt) {
        // Get F(dt) and Q(dt) from the motion model
        Matrix F = model.getF(dt);
        Matrix Q = model.getQ(dt);

        // x = F * x
        x = F.multiply(x);

        // P = F * P * F' + Q
        Matrix FP = F.multiply(P);
        Matrix Ft = F.transpose();
        P = FP.multiply(Ft).add(Q);
    }

    @Override
    public void update(Vector z) {
        // y = z - Hx (innovation)
        Vector Hx = H.multiply(x);
        Vector y = z.subtract(Hx);

        // S = HPH' + R
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
        Matrix IminusKH = I.add(KH.multiply(-1));

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
