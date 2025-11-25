package tracking;

import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CA1DModel;
import models.CV1DModel;

public class VerticalChannel {

    public enum Mode { CV, CA }

    private Mode mode;
    private KalmanFilter filter;

    private final CV1DModel cvModel;
    private final CA1DModel caModel;

    private final Matrix H_cv;   // 1D measurement for CV
    private final Matrix H_ca;   // 1D measurement for CA
    private final Matrix R;

    public VerticalChannel(Vector x0_cv,
                           Matrix P0_cv,
                           double q_cv,
                           double q_ca,
                           Matrix H_cv,
                           Matrix H_ca,
                           Matrix R) {

        this.cvModel = new CV1DModel(q_cv);
        this.caModel = new CA1DModel(q_ca);

        this.H_cv = H_cv;
        this.H_ca = H_ca;
        this.R = R;

        // Start in CV mode by default
        this.mode = Mode.CV;
        this.filter = new KalmanFilter(x0_cv, P0_cv, cvModel, H_cv, R);
    }

    public void predict(double dt) {
        filter.predict(dt);
    }

    public void update(Vector z) {
        filter.update(z);
    }

    public Mode getMode() {
        return mode;
    }

    public Vector getState() {
        return filter.getState();
    }

    public Matrix getCovariance() {
        return filter.getCovariance();
    }

    // Estimate acceleration from CA model
    private double estimateAcceleration() {
        if (mode == Mode.CA) {
            return filter.getState().get(2);
        }
        // CV has no acceleration, treat as zero
        return 0.0;
    }

    // Check if we should switch from CA → CV
    public void maybeSwitchToCV(double threshold) {
        if (mode == Mode.CA) {
            double az = estimateAcceleration();
            if (Math.abs(az) < threshold) {
                switchCAtoCV();
            }
        }
    }

    // Check if we should switch from CV → CA
    public void maybeSwitchToCA(double threshold) {
        if (mode == Mode.CV) {
            double az = estimateAcceleration(); // always 0 here
            // You would detect accel from residuals or vertical rate-of-change
            if (Math.abs(az) > threshold) {
                switchCVtoCA();
            }
        }
    }

    // --- THE IMPORTANT PART: covariance/state mapping ---

    private void switchCAtoCV() {
        Vector x_ca = filter.getState();       // [z, vz, az]
        Matrix P_ca = filter.getCovariance();  // 3x3

        // Projection S: pick z and vz only
        Matrix S = new Matrix(new double[][] {
                {1, 0, 0},
                {0, 1, 0}
        });

        Vector x_cv = S.multiply(x_ca);                         // 2x1
        Matrix P_cv = S.multiply(P_ca).multiply(S.transpose()); // 2x2

        this.filter = new KalmanFilter(x_cv, P_cv, cvModel, H_cv, R);
        this.mode = Mode.CV;
    }

    private void switchCVtoCA() {
        Vector x_cv = filter.getState();      // [z, vz]
        Matrix P_cv = filter.getCovariance(); // 2x2

        // Expansion T: embed into 3D
        Matrix T = new Matrix(new double[][] {
                {1, 0},
                {0, 1},
                {0, 0}
        });

        Vector x_ca = T.multiply(x_cv); // [z, vz, 0]

        Matrix P_ca = T.multiply(P_cv).multiply(T.transpose());
        // Add uncertainty for az
        P_ca.set(2, 2, P_ca.get(2, 2) + 10.0);

        this.filter = new KalmanFilter(x_ca, P_ca, caModel, H_ca, R);
        this.mode = Mode.CA;
    }
}
