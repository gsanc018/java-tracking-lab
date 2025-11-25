import math.Matrix;
import math.Vector;
import tracking.AdaptiveTrack;
import tracking.HorizontalChannel;
import tracking.VerticalChannel;

public class MainAdaptive {

    public static void main(String[] args) {

        double dt = 1.0;

        // ====================================
        // HORIZONTAL INITIAL STATE (CV mode)
        // ====================================
        // [x, y, vx, vy]
        Vector x0h = new Vector(new double[] {
                0.0,   // x
                0.0,   // y
                50.0,  // vx
                0.0    // vy
        });

        Matrix P0h = Matrix.identity(4).multiply(500.0);

        double q_cv_h = 1.0;
        double q_ct_h = 0.5;  // process noise for CT

        // H for CV: measure x,y from [x,y,vx,vy]
        Matrix H_cv = new Matrix(new double[][] {
                {1, 0, 0, 0},
                {0, 1, 0, 0}
        });

        // H for CT: measure x,y from [x,y,vx,vy,omega]
        Matrix H_ct = new Matrix(new double[][] {
                {1, 0, 0, 0, 0},
                {0, 1, 0, 0, 0}
        });

        Matrix R_h = new Matrix(new double[][] {
                {25, 0},
                {0, 25}
        });

        HorizontalChannel horiz = new HorizontalChannel(
                x0h, P0h,
                q_cv_h, q_ct_h,
                H_cv, H_ct,
                R_h
        );

        // ====================================
        // VERTICAL INITIAL STATE (simple CV)
        // ====================================
        // [z, vz]
        Vector x0v = new Vector(new double[] {
                0.0,   // z
                0.0    // vz
        });

        Matrix P0v = Matrix.identity(2).multiply(200.0);

        double q_cv_v = 1.0;
        double q_ca_v = 1.0;

        Matrix H_v_cv = new Matrix(new double[][] {
                {1, 0}
        });

        Matrix H_v_ca = new Matrix(new double[][] {
                {1, 0, 0}
        });

        Matrix R_v = new Matrix(new double[][] {
                {16}
        });

        VerticalChannel vert = new VerticalChannel(
                x0v, P0v,
                q_cv_v, q_ca_v,
                H_v_cv, H_v_ca,
                R_v
        );

        // ====================================
        // Build adaptive track
        // ====================================
        AdaptiveTrack track = new AdaptiveTrack(1, horiz, vert);

        // ====================================
        // "Truth" model for demonstration
        // ====================================
        double trueX = 0.0;
        double trueY = 0.0;
        double trueHeading = 0.0;                // radians
        double trueSpeed = 50.0;                 // m/s
        double omegaTrue = Math.toRadians(5.0);  // 5 deg/s

        double trueZ = 0.0;
        double trueVz = 0.0;

        System.out.println("=== ADAPTIVE TRACK DEMO (CV ↔ CT, vertical CV only) ===\n");

        for (int t = 0; t < 25; t++) {

            System.out.println("Time " + t);

            // ---------- Truth propagation (horizontal) ----------
            if (t < 10) {
                // Straight line
                trueX += trueSpeed * dt;
                // trueY unchanged
            } else {
                // Constant turn: heading increases by omegaTrue
                trueHeading += omegaTrue * dt;
                trueX += trueSpeed * dt * Math.cos(trueHeading);
                trueY += trueSpeed * dt * Math.sin(trueHeading);
            }

            // ---------- Truth propagation (vertical) ----------
            if (t >= 5 && t < 15) {
                // Simple climb: constant vertical speed
                trueVz = 10.0;  // m/s
            } else {
                trueVz = 0.0;
            }
            trueZ += trueVz * dt;

            // ---------- Generate noisy measurements ----------
            Vector measXY = new Vector(new double[] {
                    trueX + noise(5.0),
                    trueY + noise(5.0)
            });

            Vector measZ = new Vector(new double[] {
                    trueZ + noise(4.0)
            });

            // ---------- PREDICT ----------
            track.predict(dt);

            // ---------- MANUAL SWITCHES FOR DEMO ----------
            if (t == 10) {
                System.out.println("   >>> Forcing HORIZONTAL CT mode at t=10");
                horiz.forceCT();
            }
            if (t == 18) {
                System.out.println("   >>> Forcing HORIZONTAL CV mode at t=18");
                horiz.forceCV();
            }

            // (Vertical is left in CV for now, you can add forceCA/forceCV later)

            // ---------- UPDATE ----------
            track.update(measXY, measZ, t);

            // ---------- PRINT STATES ----------
            Vector hState = track.getHorizontalState();
            Vector vState = track.getVerticalState();

            System.out.printf("   Est Horizontal: x=%.2f, y=%.2f, mode=%s%n",
                    hState.get(0),
                    hState.get(1),
                    track.getHorizontalMode());

            System.out.printf("   Est Vertical:   z=%.2f, mode=%s%n",
                    vState.get(0),
                    track.getVerticalMode());

            System.out.printf("   True: x=%.2f, y=%.2f, z=%.2f%n",
                    trueX, trueY, trueZ);

            System.out.println();
        }

        System.out.println("=== END ADAPTIVE DEMO ===");
    }

    private static double noise(double scale) {
        return (Math.random() - 0.5) * scale;
    }
}
