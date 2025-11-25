package tracking;

import filters.Filter;

public class Track {

    private Filter filter;

    public Track(Filter filter) {
        this.filter = filter;
    }

    public void predict(double dt) {
        filter.predict(dt);
    }

    public void update(math.Vector z) {
        filter.update(z);
    }

    public math.Vector getState() {
        return filter.getState();
    }
}
