package tracking;

import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CVModel;
import models.CTModel;
import models.MotionModel;

public class HorizontalChannel {

    public enum Mode { CV, CT }

    private final KalmanFilter filter;
    private final MotionModel cvModel;
    private final MotionModel ctModel;
    private Mode mode;

    public HorizontalChannel(Vector initialState,
                             Matrix initialCovariance,
                             double qCv,
                             double qCt,
                             double omegaCt,
                             Matrix Hxy,
                             Matrix Rxy) {

        this.cvModel = new CVModel(qCv);
        this.ctModel = new CTModel(qCt, omegaCt);

        // start in CV mode
        this.mode = Mode.CV;

        this.filter = new KalmanFilter(initialState,
                                       initialCovariance,
                                       cvModel,
                                       Hxy,
                                       Rxy);
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

    public Mode getMode() {
        return mode;
    }

    // very simple switching placeholder – you’ll refine this later
    public void maybeSwitchToCT(double turnRateEstimate, double threshold) {
        if (mode == Mode.CV && Math.abs(turnRateEstimate) > threshold) {
            mode = Mode.CT;
            filter.setMotionModel(ctModel);
        }
    }

    public void maybeSwitchToCV(double turnRateEstimate, double threshold) {
        if (mode == Mode.CT && Math.abs(turnRateEstimate) < threshold) {
            mode = Mode.CV;
            filter.setMotionModel(cvModel);
        }
    }
}
