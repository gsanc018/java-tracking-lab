import math.Vector;
import tracking.TrackManager;

public class MainMultiTarget {

    public static void main(String[] args) {

        double dt = 1.0;

        TrackManager tm = new TrackManager(
                20.0,   // gating threshold (distance)
                3       // max coast frames
        );

        for (int t = 0; t <= 15; t++) {
            System.out.println("Time " + t);

            // Generate fake measurements
            Vector meas = null;

            if (t >= 1 && t <= 10) {
                // Target A: moves 10 m/s in x
                double x = 10 * (t - 1);
                double y = 0;
                meas = new Vector(new double[] {x, y});
            }

            if (t >= 5 && t <= 15) {
                // Target B: moves 5 m/s in y
                double x = 100; // far away horizontally so gating won't confuse
                double y = 5 * (t - 5);
                Vector measB = new Vector(new double[] {x, y});

                // If target A also exists, send both measurements
                if (meas != null) {
                    System.out.println("  Processing A");
                    tm.processMeasurement(meas, dt);
                    System.out.println("  Processing B");
                    tm.processMeasurement(measB, dt);
                } else {
                    meas = measB;
                }
            }

            // Only one target present
            if (meas != null) {
                tm.processMeasurement(meas, dt);
            }

            System.out.println();
        }
    }
}
