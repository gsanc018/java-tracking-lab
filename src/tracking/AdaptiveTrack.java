package tracking;

import math.Vector;
import math.Matrix;

public class AdaptiveTrack {

    private final HorizontalChannel horizontal;
    private final VerticalChannel vertical;

    private int id;
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

    public void predict(double dt) {
        horizontal.predict(dt);
        vertical.predict(dt);
    }

    public void update(Vector measXY, Vector measZ, int time) {
        if (measXY != null) {
            horizontal.update(measXY);
        }
        if (measZ != null) {
            vertical.update(measZ);
        }
        this.lastUpdateTime = time;
    }

    public Vector getHorizontalState() {
        return horizontal.getState();
    }

    public Vector getVerticalState() {
        return vertical.getState();
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
}
