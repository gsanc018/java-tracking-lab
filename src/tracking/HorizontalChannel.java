package tracking;

import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CVModel;
import models.CTModel;
import models.MotionModel;

public class HorizontalChannel {

    public enum Mode { CV, CT }

    private Mode mode;
    private KalmanFilter filter;

    private final CVModel cvModel;
    private final CTModel ctModel;

    private final Matrix H_xy;
    private final Matrix R;

    public HorizontalChannel(Vector x0,
                             Matrix P0,
                             double q_cv,
                             double q_ct,
                             double omega_ct,
                             Matrix H_xy,
                             Matrix R) {

        this.cvModel = new CVModel(q_cv);
        this.ctModel = new CTModel(q_ct, omega_ct);

        this.H_xy = H_xy;
        this.R = R;

        // Start in CV mode
        this.mode = Mode.CV;
        this.filter = new KalmanFilter(x0, P0, cvModel, H_xy, R);
    }

    public void predict(double dt) {
        filter.predict(dt);
    }

    public void update(Vector z_xy) {
        filter.update(z_xy);
    }

    public Vector getState() {
        return filter.getState();
    }

    public Matrix getCovariance() {
        return filter.getCovariance();
    }

    public Mode getMode() {
        return mode;
    }

    // Track turn rate from CT state
    private double estimateTurnRate(Vector x) {
        // Your CT model isn't EKF with omega in state,
        // so estimate turn rate from vx, vy curvature:
        double vx = x.get(2);
        double vy = x.get(3);

        double speed = Math.sqrt(vx*vx + vy*vy);
        if (speed < 1e-6) return 0;

        // Fake basic turn-rate estimator:
        // You’ll upgrade this later when you add full CT EKF.
        return 0; 
    }

    public void maybeSwitchToCT(double turnRate, double threshold) {
        if (mode == Mode.CV && Math.abs(turnRate) > threshold) {
            switchToCT();
        }
    }

    public void maybeSwitchToCV(double turnRate, double threshold) {
        if (mode == Mode.CT && Math.abs(turnRate) < threshold) {
            switchToCV();
        }
    }

    private void switchToCT() {
        this.filter.setMotionModel(ctModel);
        this.mode = Mode.CT;
    }

    private void switchToCV() {
        this.filter.setMotionModel(cvModel);
        this.mode = Mode.CV;
    }
}
