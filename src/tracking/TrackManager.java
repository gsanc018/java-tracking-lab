package tracking;

import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CVModel;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class TrackManager {

    private static int nextTrackId = 1;
    private final List<SimpleTrack> tracks = new ArrayList<>();

    // Gating threshold (distance)
    private final double gateThreshold;
    private final int maxCoast;

    public TrackManager(double gateThreshold, int maxCoast) {
        this.gateThreshold = gateThreshold;
        this.maxCoast = maxCoast;
    }

    // Inner class representing ONE active track
    public static class SimpleTrack {
        public int id;
        public Track track;
        public int coastCount = 0;

        public SimpleTrack(int id, Track track) {
            this.id = id;
            this.track = track;
        }
    }

    // Predict all tracks
    public void predictAll(double dt) {
        for (SimpleTrack t : tracks) {
            t.track.predict(dt);
        }
    }

    // Try to update an existing track
    private SimpleTrack findMatchingTrack(Vector z) {
        for (SimpleTrack t : tracks) {
            Vector est = t.track.getState();
            double dx = est.get(0) - z.get(0);
            double dy = est.get(1) - z.get(1);
            double dist = Math.sqrt(dx*dx + dy*dy);

            if (dist < gateThreshold) {
                return t;
            }
        }
        return null;
    }

    // Process one measurement at time t
    public void processMeasurement(Vector z, double dt) {

        // 1. Predict
        predictAll(dt);

        // 2. Try to match to an existing track
        SimpleTrack match = findMatchingTrack(z);

        if (match != null) {
            // update
            match.track.update(z);
            match.coastCount = 0;
            System.out.println("    Updated Track " + match.id);
        } else {
            // create a new track
            int id = nextTrackId++;
            Track newTrack = makeNewTrack(z);
            SimpleTrack st = new SimpleTrack(id, newTrack);
            tracks.add(st);
            System.out.println("    Created Track " + id);
        }

        // 3. Coast tracks with no update
        for (SimpleTrack t : tracks) {
            if (t.track.getState().get(0) != z.get(0) &&
                t.track.getState().get(1) != z.get(1)) {
                t.coastCount++;
            }
        }

        // 4. Delete tracks that have coasted too long
        Iterator<SimpleTrack> it = tracks.iterator();
        while (it.hasNext()) {
            SimpleTrack t = it.next();
            if (t.coastCount > maxCoast) {
                System.out.println("    Track " + t.id + " deleted (coasted too long)");
                it.remove();
            }
        }
    }

    private Track makeNewTrack(Vector z) {
        Vector x0 = new Vector(new double[] {
                z.get(0),  // x
                z.get(1),  // y
                0.0,       // vx (unknown)
                0.0        // vy (unknown)
        });

        Matrix P0 = Matrix.identity(4).multiply(1000.0);

        CVModel model = new CVModel(1.0);

        Matrix H = new Matrix(new double[][] {
                {1, 0, 0, 0},
                {0, 1, 0, 0}
        });

        Matrix R = new Matrix(new double[][] {
                {4, 0},
                {0, 4}
        });

        KalmanFilter kf = new KalmanFilter(x0, P0, model, H, R);
        return new Track(kf);
    }
}
