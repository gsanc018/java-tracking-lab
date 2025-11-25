import math.Matrix;
import math.Vector;
import tracking.AdaptiveTrack;
import tracking.HorizontalChannel;
import tracking.VerticalChannel;

public class MainAdaptive {

    public static void main(String[] args) {

        double dt = 1.0;

        // =======================
        // INITIAL HORIZONTAL STATE
        // =======================
        Vector x0h = new Vector(new double[] {
                0.0, 0.0,   // x, y
                20.0, 0.0   // vx, vy
        });
        Matrix P0h = Matrix.identity(4).multiply(500.0);

        double q_cv_h = 1.0;
        double q_ct_h = 1.0;
        double omega_ct = Math.toRadians(8.0); // turning rate

        Matrix Hh = new Matrix(new double[][] {
                {1, 0, 0, 0},
                {0, 1, 0, 0}
        });
        Matrix Rh = new Matrix(new double[][] {
                {9, 0},
                {0, 9}
        });

        HorizontalChannel horiz = new HorizontalChannel(
                x0h, P0h, q_cv_h, q_ct_h, omega_ct, Hh, Rh
        );

        // =====================
        // INITIAL VERTICAL STATE
        // =====================
        Vector x0v = new Vector(new double[] {
                100.0,   // z
                0.0      // vz
        });
        Matrix P0v = Matrix.identity(2).multiply(200.0);

        double q_cv_v = 1.0;
        double q_ca_v = 1.0;

        Matrix Hv_cv = new Matrix(new double[][] {
                {1, 0}
        });
        Matrix Hv_ca = new Matrix(new double[][] {
                {1, 0, 0}
        });

        Matrix Rv = new Matrix(new double[][] {{
                16
        }});

        VerticalChannel vert = new VerticalChannel(
                x0v, P0v,
                q_cv_v, q_ca_v,
                Hv_cv, Hv_ca,
                Rv
        );

        // =====================
        // AdaptiveTrack creation
        // =====================
        AdaptiveTrack track = new AdaptiveTrack(1, horiz, vert);

        System.out.println("=== STARTING ADAPTIVE FILTER DEMO ===\n");

        // Simulate 20 seconds of measurements
        for (int t = 0; t < 20; t++) {

            System.out.println("Time " + t);

            // Horizontal truth: straight, then turning after t >= 8
            double trueX = track.getHorizontalState().get(0)
                           + track.getHorizontalState().get(2) * dt;

            double trueY = track.getHorizontalState().get(1)
                           + track.getHorizontalState().get(3) * dt;

            if (t >= 8) {
                // introduce a turn after time 8
                trueY += 3; // simulated turn
            }

            Vector measXY = new Vector(new double[] {
                    trueX + randomNoise(3),
                    trueY + randomNoise(3)
            });

            // Vertical truth: accelerate upward after t >= 5
            double trueZ = track.getVerticalState().get(0)
                           + track.getVerticalState().get(1) * dt;

            if (t >= 5 && t < 10) {
                trueZ += 2; // climbing
            }

            Vector measZ = new Vector(new double[]{
                    trueZ + randomNoise(4)
            });

            // PREDICT
            track.predict(dt);

            // ADAPTIVE SWITCH LOGIC
            // (super simplified for demo)
            horiz.maybeSwitchToCT(1.0, 0.5);
            horiz.maybeSwitchToCV(0.0, 0.5);

            vert.maybeSwitchToCA(0.1);
            vert.maybeSwitchToCV(0.1);

            // UPDATE
            track.update(measXY, measZ, t);

            // Print states
            System.out.printf("   Horizontal=(%.2f, %.2f) mode=%s%n",
                    track.getHorizontalState().get(0),
                    track.getHorizontalState().get(1),
                    track.getHorizontalMode());

            System.out.printf("   Vertical z=%.2f mode=%s%n",
                    track.getVerticalState().get(0),
                    track.getVerticalMode());

            System.out.println();
        }

        System.out.println("=== END ADAPTIVE DEMO ===");
    }

    private static double randomNoise(double scale) {
        return (Math.random() - 0.5) * scale;
    }
}
