package tracking;

import filters.Filter;
import filters.KalmanFilter;
import filters.EKF;
import math.Matrix;
import math.Vector;
import models.CVModel;
import models.CTEKFModel;

public class HorizontalChannel {

    public enum Mode { CV, CT }

    private Mode mode;

    private KalmanFilter cvFilter;
    private EKF ctFilter;

    // Active filter reference (KF or EKF)
    private Filter activeFilter;

    // Motion models
    private final CVModel cvModel;
    private final CTEKFModel ctModel;

    // Measurement matrices
    private final Matrix H_cv; // 2x4
    private final Matrix H_ct; // 2x5
    private final Matrix R;    // 2x2

    public HorizontalChannel(Vector x0_cv,
                             Matrix P0_cv,
                             double q_cv,
                             double q_ct,
                             Matrix H_cv,
                             Matrix H_ct,
                             Matrix R) {

        this.cvModel = new CVModel(q_cv);
        this.ctModel = new CTEKFModel(q_ct);

        this.H_cv = H_cv;
        this.H_ct = H_ct;
        this.R = R;

        this.cvFilter = new KalmanFilter(x0_cv, P0_cv, cvModel, H_cv, R);

        this.activeFilter = cvFilter;
        this.mode = Mode.CV;
    }

    public void predict(double dt) {
        activeFilter.predict(dt);
    }

    public void update(Vector z) {
        activeFilter.update(z);
    }

    public Vector getState() {
        return activeFilter.getState();
    }

    public Matrix getCovariance() {
        return activeFilter.getCovariance();
    }

    public Mode getMode() {
        return mode;
    }

    // -------------------------
    // TURN RATE ESTIMATION
    // -------------------------
    private double estimateTurnRate() {
        if (mode == Mode.CT) {
            // omega is the 5th state variable in CT-EKF
            return ctFilter.getState().get(4);
        }
        return 0.0;
    }

    // -------------------------
    // ADAPTIVE SWITCHING
    // -------------------------
    public void maybeSwitchToCT(double threshold) {
        if (mode == Mode.CV) {
            // TODO: compute from curvature or residuals later
            double fauxTurnRate = 0.0;

            if (Math.abs(fauxTurnRate) > threshold) {
                switchCVtoCT();
            }
        }
    }

    public void maybeSwitchToCV(double threshold) {
        if (mode == Mode.CT) {
            double w = estimateTurnRate();
            if (Math.abs(w) < threshold) {
                switchCTtoCV();
            }
        }
    }

    // -------------------------
    // STATE DIMENSION SWITCHING
    // -------------------------

    // 4D CV → 5D CT
    private void switchCVtoCT() {

        Vector x_cv = cvFilter.getState();      // [x, y, vx, vy]
        Matrix P_cv = cvFilter.getCovariance(); // 4x4

        // Expansion matrix T (5x4)
        Matrix T = new Matrix(new double[][] {
                {1, 0, 0, 0},  // x
                {1, 0, 0, 0},  // y
                {0, 1, 0, 0},  // vx
                {0, 0, 1, 0},  // vy
                {0, 0, 0, 0}   // omega initialized to 0
        });

        // Actually, the correct expansion should be:
        T = new Matrix(new double[][] {
                {1, 0, 0, 0}, // x
                {0, 1, 0, 0}, // y
                {0, 0, 1, 0}, // vx
                {0, 0, 0, 1}, // vy
                {0, 0, 0, 0}  // omega
        });

        Vector x_ct = T.multiply(x_cv);
        Matrix P_ct = T.multiply(P_cv).multiply(T.transpose());

        // Add initial uncertainty for ω
        P_ct.set(4, 4, 0.01);

        this.ctFilter = new EKF(x_ct, P_ct, ctModel, H_ct, R);
        this.activeFilter = ctFilter;
        this.mode = Mode.CT;
    }

    // 5D CT → 4D CV
    private void switchCTtoCV() {

        Vector x_ct = ctFilter.getState();      // [x, y, vx, vy, ω]
        Matrix P_ct = ctFilter.getCovariance(); // 5x5

        // Projection matrix S (4x5)
        Matrix S = new Matrix(new double[][] {
                {1, 0, 0, 0, 0}, // x
                {0, 1, 0, 0, 0}, // y
                {0, 0, 1, 0, 0}, // vx
                {0, 0, 0, 1, 0}  // vy
        });

        Vector x_cv = S.multiply(x_ct);
        Matrix P_cv = S.multiply(P_ct).multiply(S.transpose());

        this.cvFilter = new KalmanFilter(x_cv, P_cv, cvModel, H_cv, R);
        this.activeFilter = cvFilter;
        this.mode = Mode.CV;
    }

    // ---------------------------------
    // Manual mode control for demo
    // ---------------------------------
    public void forceCT() {
        if (mode == Mode.CV) {
            switchCVtoCT();
        }
    }

    public void forceCV() {
        if (mode == Mode.CT) {
            switchCTtoCV();
        }
    }

}
