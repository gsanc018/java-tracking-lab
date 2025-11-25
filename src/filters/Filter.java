package filters;

import math.Vector;

public interface Filter {
    void predict(double dt);
    void update(Vector measurement);
    Vector getState();
}
