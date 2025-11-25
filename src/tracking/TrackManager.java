package tracking;

import java.util.ArrayList;
import java.util.List;

public class TrackManager {

    private List<Track> tracks = new ArrayList<>();

    public void addTrack(Track track) {
        tracks.add(track);
    }

    public void predictAll(double dt) {
        for (Track t : tracks) {
            t.predict(dt);
        }
    }

    public void updateTrack(int index, math.Vector z) {
        tracks.get(index).update(z);
    }

    public Track getTrack(int index) {
        return tracks.get(index);
    }
}
