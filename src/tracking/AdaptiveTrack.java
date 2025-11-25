package tracking;

import math.Vector;

public class AdaptiveTrack {

    private final HorizontalChannel horizontal;
    private final VerticalChannel vertical;

    public AdaptiveTrack(HorizontalChannel horizontal, VerticalChannel vertical) {
        this.horizontal = horizontal;
        this.vertical = vertical;
    }

    public void predict(double dt) {
        horizontal.predict(dt);
        vertical.predict(dt);
    }

    public void update(Vector z_xy, Vector z_z) {
        if (z_xy != null) horizontal.update(z_xy);
        if (z_z != null) vertical.update(z_z);
    }

    public Vector getHorizontalState() {
        return horizontal.getState();
    }

    public Vector getVerticalState() {
        return vertical.getState();
    }
}
