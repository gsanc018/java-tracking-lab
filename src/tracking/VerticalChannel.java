package tracking;

import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CA1DModel;
import models.CV1DModel;
import models.MotionModel;

public class VerticalChannel {

    public enum Mode { CV, CA }

    private final KalmanFilter filter;
    private final MotionModel cvModel;
    private final MotionModel caModel;
    private Mode mode;

    public VerticalChannel(Vector initialState,
                           Matrix initialCovariance,
                           double qCv,
                           double qCa,
                           Matrix Hz,
                           Matrix Rz) {

        this.cvModel = new CV1DModel(qCv);
        this.caModel = new CA1DModel(qCa);

        // start in CA (like “complex first, simplify if accel≈0”)
        this.mode = Mode.CA;

        this.filter = new KalmanFilter(initialState,
                                       initialCovariance,
                                       caModel,
                                       Hz,
                                       Rz);
    }

    public void predict(double dt) {
        filter.predict(dt);
    }

    public void update(Vector z_z) {
        filter.update(z_z);
    }

    public Vector getState() {
        return filter.getState();
    }

    public Mode getMode() {
        return mode;
    }

    // simple switching logic using estimated vertical accel
    public void maybeSwitchToCV(double accelEstimate, double threshold) {
        if (mode == Mode.CA && Math.abs(accelEstimate) < threshold) {
            mode = Mode.CV;
            filter.setMotionModel(cvModel);
        }
    }

    public void maybeSwitchToCA(double accelEstimate, double threshold) {
        if (mode == Mode.CV && Math.abs(accelEstimate) > threshold) {
            mode = Mode.CA;
            filter.setMotionModel(caModel);
        }
    }
}
