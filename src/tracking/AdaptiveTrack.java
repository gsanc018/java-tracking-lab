package tracking;

import math.Vector;
import math.Matrix;

public class AdaptiveTrack {

    private final int id;

    private final HorizontalChannel horizontal;
    private final VerticalChannel vertical;

    private int lastUpdateTime;

    public AdaptiveTrack(int id,
                         HorizontalChannel horizontal,
                         VerticalChannel vertical) {

        this.id = id;
        this.horizontal = horizontal;
        this.vertical = vertical;
        this.lastUpdateTime = 0;
    }

    public int getId() {
        return id;
    }

    // ------------------------------------
    // PREDICT — propagate both channels
    // ------------------------------------
    public void predict(double dt) {
        horizontal.predict(dt);
        vertical.predict(dt);
    }

    // ------------------------------------
    // UPDATE — feed new horizontal + vertical measurements
    // ------------------------------------
    public void update(Vector measXY, Vector measZ, int timeSec) {

        if (measXY != null) {
            horizontal.update(measXY);
        }

        if (measZ != null) {
            vertical.update(measZ);
        }

        this.lastUpdateTime = timeSec;
    }

    // ------------------------------------
    // GETTERS FOR STATES AND MODES
    // ------------------------------------

    public Vector getHorizontalState() {
        return horizontal.getState(); // returns 4D CV or 5D CT depending on mode
    }

    public Vector getVerticalState() {
        return vertical.getState();   // returns 2D CV or 3D CA depending on mode
    }

    public HorizontalChannel.Mode getHorizontalMode() {
        return horizontal.getMode();
    }

    public VerticalChannel.Mode getVerticalMode() {
        return vertical.getMode();
    }

    public int getLastUpdateTime() {
        return lastUpdateTime;
    }

    // ------------------------------------
    // Combined full-state getter (optional)
    // ------------------------------------
    public Vector getFullState() {
        // Horizontal: [x, y, vx, vy, (omega?)]
        Vector h = horizontal.getState();
        // Vertical: [z, vz, (az?)]
        Vector v = vertical.getState();

        // Combine into one big vector
        int n = h.size() + v.size();
        Vector out = new Vector(n);

        int index = 0;
        for (int i = 0; i < h.size(); i++) {
            out.set(index++, h.get(i));
        }
        for (int i = 0; i < v.size(); i++) {
            out.set(index++, v.get(i));
        }

        return out;
    }
}
